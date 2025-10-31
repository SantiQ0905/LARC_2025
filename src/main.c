#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "LFR_PID_FAST";

/* ---------- PINS ---------- */
#define PIN_MUX_S0  4
#define PIN_MUX_S1  5
#define PIN_MUX_S2  0
#define PIN_MUX_Y   1   // ADC1_CH1

#define PIN_MA1     18
#define PIN_MA2     19
#define PIN_MB1     20
#define PIN_MB2     21

#define PIN_BUTTON  14
#define PIN_DIP_1   6
#define PIN_DIP_2   7

/* ---------- PWM ---------- */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)
#define DT_MS           18          // Ciclo más rápido para mejor respuesta

/* Giro abierto cuando SOLO ve I0 */
#define TURN_45_MS      170         // Giro ultrarrápido y brusco
#define TURN_SPEED      PWM_MAX_DUTY  // Giro a potencia máxima absoluta

/* ---------- SENSOR CONFIG ---------- */
#define LINE_THRESHOLD      1650
#define STOP_BLACK_MIN      7
#define STOP_CONFIRM_COUNT  3

/* ---------- VELOCIDAD / PID ---------- */
/* VELOCIDAD EXTREMA - ANTICIPACIÓN MÁXIMA */
#define BASE_SPEED      980         // Velocidad casi al máximo
#define KP_BASE         520.0f      // Proporcional MUY agresivo para giros bruscos
#define KI_BASE         0.2f        // Integral casi nula (máxima estabilidad)
#define KD_BASE         320.0f      // Derivativa EXTREMA para anticipación total

/* ---------- UART ---------- */
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     16
#define UART_BAUD       115200

typedef struct {
    float kP, kI, kD;
    float integral;
    float prev_error;
} PID_t;

/* ---------- PROTOS ---------- */
static void telemetry_init(void);
static void telemetry_send(int tick, float error, int left, int right, int blacks);
static inline void mux_select(uint8_t ch);
static inline int clamp_int(int v, int lo, int hi);
static void motor_set(ledc_channel_t pwm_chan, gpio_num_t dir, int speed);
static void pid_init(PID_t *p, float kp, float ki, float kd);

/* ---------- UART ---------- */
static void telemetry_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void telemetry_send(int tick, float error, int left, int right, int blacks) {
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%d,%.2f,%d,%d,%d\n",
                     tick, error, left, right, blacks);
    uart_write_bytes(UART_PORT_NUM, buf, n);
}

static inline void mux_select(uint8_t ch) {
    gpio_set_level(PIN_MUX_S0, (ch >> 0) & 1);
    gpio_set_level(PIN_MUX_S1, (ch >> 1) & 1);
    gpio_set_level(PIN_MUX_S2, (ch >> 2) & 1);
    esp_rom_delay_us(5);
}

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void motor_set(ledc_channel_t pwm_chan, gpio_num_t dir, int speed) {
    if (speed >= 0) {
        gpio_set_level(dir, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, speed);
    } else {
        gpio_set_level(dir, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, -speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_chan);
}

static void pid_init(PID_t *p, float kp, float ki, float kd) {
    p->kP = kp;
    p->kI = ki;
    p->kD = kd;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
}

/* ===================================================== */
void app_main(void) {
    ESP_LOGI(TAG, "Starting Line Follower V3 FAST (toggle + final STOP)");

    telemetry_init();

    /* GPIO salida */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_MUX_S0) |
            (1ULL << PIN_MUX_S1) |
            (1ULL << PIN_MUX_S2) |
            (1ULL << PIN_MA2)    |
            (1ULL << PIN_MB2),
        .pull_up_en = 0
    };
    gpio_config(&out_cfg);

    /* Entradas: botón y DIP */
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL << PIN_BUTTON) |
            (1ULL << PIN_DIP_1)  |
            (1ULL << PIN_DIP_2),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&in_cfg);

    /* ADC */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc1);
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc1, adc_ch, &ch_cfg);

    /* PWM motores */
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .duty_resolution = PWM_RES_BITS
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t chA = {
        .gpio_num   = PIN_MA1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1;
    chB.channel  = LEDC_CHANNEL_1;
    ledc_channel_config(&chA);
    ledc_channel_config(&chB);

    /* PID */
    PID_t pid;
    pid_init(&pid, KP_BASE, KI_BASE, KD_BASE);

    /* Leer DIP switches para configurar velocidad */
    bool dip1 = gpio_get_level(PIN_DIP_1);  // 0=ON, 1=OFF (con pull-up)
    bool dip2 = gpio_get_level(PIN_DIP_2);  // 0=ON, 1=OFF (con pull-up)
    
    int base_speed;
    int stop_confirm_threshold;
    if (!dip1 && !dip2) {
        // Ambos ON → velocidad máxima
        base_speed = 980;
        stop_confirm_threshold = STOP_CONFIRM_COUNT;  // Stop normal (3)
        ESP_LOGI(TAG, "DIP Mode: ON-ON → Speed: 980, Stop: %d", stop_confirm_threshold);
    } else if (dip1 && dip2) {
        // Ambos OFF → velocidad media, stop más lento
        base_speed = 850;
        stop_confirm_threshold = 6;  // Se tarda el doble en hacer stop
        ESP_LOGI(TAG, "DIP Mode: OFF-OFF → Speed: 850, Stop: %d", stop_confirm_threshold);
    } else {
        // Otros modos → usar velocidad por defecto
        base_speed = BASE_SPEED;
        stop_confirm_threshold = STOP_CONFIRM_COUNT;
        ESP_LOGI(TAG, "DIP Mode: Mixed → Speed: %d, Stop: %d", BASE_SPEED, stop_confirm_threshold);
    }

    int weights[8]   = {-7, -4, -2, -1, 1, 2, 4, 7};  // Pesos más extremos para mejor detección
    float dt         = DT_MS / 1000.0f;
    int tick         = 0;
    int stop_count   = 0;
    int last_dir     = 1;

    bool running     = false;
    bool last_btn    = true;
    bool final_stop  = false;

    while (1) {
        /* ==== BOTÓN (toggle) ==== */
        bool btn_now = gpio_get_level(PIN_BUTTON);
        if (last_btn && !btn_now) {
            vTaskDelay(pdMS_TO_TICKS(40));
            if (gpio_get_level(PIN_BUTTON) == 0) {
                if (final_stop) final_stop = false;
                running = !running;
                ESP_LOGI(TAG, "BUTTON TOGGLE -> running=%d", running);
            }
            while (gpio_get_level(PIN_BUTTON) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        last_btn = btn_now;

        if (!running || final_stop) {
            motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* ===== LECTURA DE SENSORES ===== */
        int blk[8] = {0};
        int black_count = 0;
        int sum = 0, act = 0;

        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int val = 0;
            for (int k = 0; k < 3; k++) {
                int tmp;
                adc_oneshot_read(adc1, adc_ch, &tmp);
                val += tmp;
            }
            val /= 3;
            blk[i] = (val < LINE_THRESHOLD);

            if (blk[i]) {
                int w = weights[i];
                if (i == 0) w *= 2;    // sensor 0 agresivo
                sum += w;
                act++;
                black_count++;
            }
        }

        /* ===== STOP FINAL ===== */
        if (black_count >= STOP_BLACK_MIN) {
            stop_count++;
        } else {
            stop_count = 0;
        }

        if (stop_count >= stop_confirm_threshold) {
            ESP_LOGW(TAG, "FINAL STOP DETECTED → moving slightly forward into box");

            int creep_speed   = 700;    // Entrada rápida y decidida
            int creep_time_ms = 700;    // Tiempo mínimo necesario

            motor_set(LEDC_CHANNEL_0, PIN_MA2, creep_speed);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, creep_speed);
            vTaskDelay(pdMS_TO_TICKS(creep_time_ms));

            motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);

            final_stop = true;
            pid.integral  = 0.0f;
            pid.prev_error = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        /* ===== SOLO SENSOR 0 ===== */
        if (blk[0] && black_count == 1) {
            ESP_LOGW(TAG, "SHARP TURN: sensor0 only → spin to RIGHT");
            int ts = clamp_int(TURN_SPEED, 0, PWM_MAX_DUTY);
            motor_set(LEDC_CHANNEL_0, PIN_MA2, -ts);
            motor_set(LEDC_CHANNEL_1, PIN_MB2,  ts);
            vTaskDelay(pdMS_TO_TICKS(TURN_45_MS));
            motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
            pid.integral  = 0.0f;
            pid.prev_error = 0.0f;
            continue;
        }

        /* ===== CÁLCULO DE ERROR ===== */
        float pos   = (act > 0) ? ((float)sum / act) : 5 * last_dir;
        float error = -pos;
        last_dir    = (error > 0) ? 1 : -1;

        /* ===== PID DINÁMICO (ANTICIPACIÓN EXTREMA + GIROS BRUSCOS) ===== */
        float kp_dynamic = KP_BASE;
        float kd_dynamic = KD_BASE;

        if (fabsf(error) > 0.8f) {         // Anticipación ULTRA temprana
            kp_dynamic *= 1.55f;           // Giro más brusco
            kd_dynamic *= 1.80f;           // Anticipación aumentada
        }
        if (fabsf(error) > 2.5f) {         // Curvas cerradas - corrección inmediata
            kp_dynamic *= 1.90f;           // Giro MUY brusco
            kd_dynamic *= 2.20f;           // Máxima anticipación
        }

        /* PID core */
        pid.integral += error * dt;
        float deriv = (error - pid.prev_error) / dt;
        pid.prev_error = error;

        float correction = kp_dynamic * error
                         + KI_BASE * pid.integral
                         + kd_dynamic * deriv;

        int L = clamp_int(base_speed - (int)correction,
                          -PWM_MAX_DUTY, PWM_MAX_DUTY);
        int R = clamp_int(base_speed + (int)correction,
                          -PWM_MAX_DUTY, PWM_MAX_DUTY);

        /* Reducción más agresiva en curvas para mayor control */
        if (black_count <= 2 && fabsf(error) < 1.5f) {
            // Solo mantiene velocidad en rectas
            L = (int)(L * 0.97f);
            R = (int)(R * 0.97f);
        } else if (fabsf(error) > 2.5f) {
            // Reduce velocidad en curvas cerradas para mejor control
            L = (int)(L * 0.88f);
            R = (int)(R * 0.88f);
        }

        motor_set(LEDC_CHANNEL_0, PIN_MA2, L);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, R);

        if (tick++ % 5 == 0) {
            telemetry_send(tick, error, L, R, black_count);
        }

        vTaskDelay(pdMS_TO_TICKS(DT_MS));
    }
}