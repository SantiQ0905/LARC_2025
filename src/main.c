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

static const char *TAG = "LFR_PID_V3";

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
#define PWM_FREQ_HZ 20000
#define PWM_RES_BITS 10
#define PWM_MAX_DUTY ((1 << PWM_RES_BITS) - 1)
#define DT_MS 20

/* ---------- SENSOR CONFIG ---------- */
#define LINE_THRESHOLD 1650
#define STOP_BLACK_MIN 7
#define STOP_CONFIRM_COUNT 3  // número de lecturas consecutivas para detenerse

/* ---------- MODOS ---------- */
#define BASE_SPEED  950
#define KP_BASE     135.0f
#define KI_BASE     3.5f
#define KD_BASE     60.0f

/* ---------- UART ---------- */
#define UART_PORT_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_BAUD 115200

/* ---------- FUNCIONES AUX ---------- */
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
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void telemetry_send(int tick, float error, int left, int right, int blacks) {
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%d,%.2f,%d,%d,%d\n", tick, error, left, right, blacks);
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

/* ---------- PID ---------- */
typedef struct {
    float kP, kI, kD;
    float integral;
    float prev_error;
} PID_t;

static void pid_init(PID_t *p, float kp, float ki, float kd) {
    p->kP = kp; p->kI = ki; p->kD = kd;
    p->integral = 0.0f; p->prev_error = 0.0f;
}

/* ---------- MAIN ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "Starting Line Follower V3");

    telemetry_init();

    // Config GPIO
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_MUX_S0) | (1ULL << PIN_MUX_S1) | (1ULL << PIN_MUX_S2) |
            (1ULL << PIN_MA2) | (1ULL << PIN_MB2),
        .pull_up_en = 0
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BUTTON) | (1ULL << PIN_DIP_1) | (1ULL << PIN_DIP_2),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&in_cfg);

    // ADC
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc1);
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    adc_oneshot_config_channel(adc1, adc_ch, &ch_cfg);

    // PWM
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = PWM_RES_BITS
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t chA = {
        .gpio_num = PIN_MA1, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1; chB.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&chA);
    ledc_channel_config(&chB);

    // PID inicial
    PID_t pid;
    pid_init(&pid, KP_BASE, KI_BASE, KD_BASE);

    int base_speed = BASE_SPEED;
    int weights[8] = {-5, -3, -1, 0, 0, 1, 3, 5};
    float dt = DT_MS / 1000.0f;
    int tick = 0;
    int stop_count = 0;
    int last_dir = 1;

    while (1) {
        int blk[8] = {0};
        int black_count = 0;
        int sum = 0, act = 0;

        // Lectura de sensores
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
                /* Make sensor 0 much more aggressive: multiply its weight by 20 */
                int w = weights[i];
                if (i == 0) w *= 2; /* aggressive multiplier requested */
                sum += w;
                act++; black_count++;
            }
        }

        // Stop seguro
        if (black_count >= STOP_BLACK_MIN) {
            stop_count++;
        } else stop_count = 0;

        if (stop_count >= STOP_CONFIRM_COUNT) {
            motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
            ESP_LOGW(TAG, "STOP: black sensors=%d", black_count);
            vTaskDelay(pdMS_TO_TICKS(200));
            stop_count = 0;
            continue;
        }

        // Cálculo del error de línea
        float pos = (act > 0) ? ((float)sum / act) : 5 * last_dir;
        float error = -pos;
        last_dir = (error > 0) ? 1 : -1;

        // Ajuste dinámico de KD y KP
        float kd_dynamic = KD_BASE;
        float kp_dynamic = KP_BASE;

        if (fabs(error) > 2.5f) {
            kd_dynamic *= 1.5f;
            kp_dynamic *= 1.1f;
        }
        if (fabs(error) > 4.0f) {
            kd_dynamic *= 1.8f;
            kp_dynamic *= 1.3f;
        }

        // PID
        pid.integral += error * dt;
        float deriv = (error - pid.prev_error) / dt;
        pid.prev_error = error;

        float correction = kp_dynamic * error + KI_BASE * pid.integral + kd_dynamic * deriv;

        // Control de velocidad dinámica
        int L = clamp_int(base_speed - (int)correction, -PWM_MAX_DUTY, PWM_MAX_DUTY);
        int R = clamp_int(base_speed + (int)correction, -PWM_MAX_DUTY, PWM_MAX_DUTY);

        // Suaviza curvas y evita saturación
        if (black_count <= 2) {
            L *= 0.85f; R *= 0.85f;
        } else if (fabs(error) > 3.0f) {
            if (error > 0) R *= 1.1f;
            else L *= 1.1f;
        }

        motor_set(LEDC_CHANNEL_0, PIN_MA2, L);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, R);

        if (tick++ % 5 == 0)
            telemetry_send(tick, error, L, R, black_count);

        vTaskDelay(pdMS_TO_TICKS(DT_MS));
    }
}