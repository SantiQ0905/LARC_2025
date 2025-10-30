// src/main.c
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

static const char *TAG = "LFR_PID_TELEM";

/* ========== PINS ========== */
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

/* ========== PWM / TIMING ========== */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)
#define DT_MS           20         // loop period (ms)

/* ========== LINE SENSOR ========== */
#define LINE_THRESHOLD   1650
#define IGNORE_SENSOR_1  0
#define STOP_BLACK_MIN   7

/* ========== PID DEFAULTS (will be overridden by DIP mode) ========== */
#define MODE_SLOW_BASE_SPEED    600
#define MODE_SLOW_KP            90.0f
#define MODE_SLOW_KI            3.0f
#define MODE_SLOW_KD            45.0f

#define MODE_NORMAL_BASE_SPEED  850
#define MODE_NORMAL_KP          120.0f
#define MODE_NORMAL_KI          3.5f
#define MODE_NORMAL_KD          55.0f

#define MODE_FAST_BASE_SPEED    950
#define MODE_FAST_KP            140.0f
#define MODE_FAST_KI            3.8f
#define MODE_FAST_KD            60.0f

#define MODE_TURBO_BASE_SPEED   1200
#define MODE_TURBO_KP           160.0f
#define MODE_TURBO_KI           4.0f
#define MODE_TURBO_KD           65.0f

/* ========== Telemetry (UART1) ========== */
#define UART_PORT_NUM   UART_NUM_1
#define UART_TX_PIN     17
#define UART_RX_PIN     16
#define UART_BAUD       115200

static void telemetry_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static inline void telemetry_send_csv(int tick, int err, int pterm, int iterm, int dterm,
                                      int L, int R, int black_count, int stop_state, int gap_state)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                     tick, err, pterm, iterm, dterm, L, R, black_count, stop_state, gap_state);
    if (n > 0) {
        uart_write_bytes(UART_PORT_NUM, buf, n);
    }
}

/* ========== Helpers ========== */
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

static void motor_set(ledc_channel_t pwm_chan, gpio_num_t pin_dir, int speed) {
    if (speed >= 0) {
        gpio_set_level(pin_dir, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, speed);
    } else {
        gpio_set_level(pin_dir, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, -speed);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_chan);
}

/* ========== PID structure ========== */
typedef struct {
    float kP;
    float kI;
    float kD;
    float integral;
    float prev_error;
    float i_limit;
} PID_t;

static void pid_init(PID_t *p, float kp, float ki, float kd, float i_limit)
{
    p->kP = kp; p->kI = ki; p->kD = kd;
    p->integral = 0.0f; p->prev_error = 0.0f;
    p->i_limit = i_limit;
}

static float pid_compute(PID_t *p, float error, float dt)
{
    float P = p->kP * error;
    p->integral += error * dt;
    if (p->integral > p->i_limit) p->integral = p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;
    float I = p->kI * p->integral;
    float D = p->kD * ((error - p->prev_error) / dt);
    p->prev_error = error;
    return P + I + D;
}

/* ========== MAIN ========== */
void app_main(void)
{
    ESP_LOGI(TAG, "Boot LFR_PID_TELEM (PlatformIO / ESP-IDF)");

    /* ===== telemetry ===== */
    telemetry_init();

    /* ===== GPIO output (MUX + motor DIRs) ===== */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_MUX_S0) |
            (1ULL << PIN_MUX_S1) |
            (1ULL << PIN_MUX_S2) |
            (1ULL << PIN_MA2) |
            (1ULL << PIN_MB2),
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    /* ===== GPIO input (button + dips) ===== */
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BUTTON) | (1ULL << PIN_DIP_1) | (1ULL << PIN_DIP_2),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&in_cfg);

    /* ===== ADC one-shot (sensor MUX output) ===== */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, adc_ch, &ch_cfg));

    /* ===== LEDC PWM (motors) ===== */
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = PWM_RES_BITS
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t chA = {
        .gpio_num = PIN_MA1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1;
    chB.channel = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&chA));
    ESP_ERROR_CHECK(ledc_channel_config(&chB));

    /* ===== mode selection (DIP) ===== */
    int dip1 = gpio_get_level(PIN_DIP_1);
    int dip2 = gpio_get_level(PIN_DIP_2);

    int BASE_SPEED = MODE_NORMAL_BASE_SPEED;
    PID_t pid;
    if (dip1 == 1 && dip2 == 1) {
        BASE_SPEED = MODE_SLOW_BASE_SPEED;
        pid_init(&pid, MODE_SLOW_KP, MODE_SLOW_KI, MODE_SLOW_KD, 1000.0f);
    } else if (dip1 == 0 && dip2 == 1) {
        BASE_SPEED = MODE_NORMAL_BASE_SPEED;
        pid_init(&pid, MODE_NORMAL_KP, MODE_NORMAL_KI, MODE_NORMAL_KD, 2000.0f);
    } else if (dip1 == 1 && dip2 == 0) {
        BASE_SPEED = MODE_FAST_BASE_SPEED;
        pid_init(&pid, MODE_FAST_KP, MODE_FAST_KI, MODE_FAST_KD, 3000.0f);
    } else {
        BASE_SPEED = MODE_TURBO_BASE_SPEED;
        pid_init(&pid, MODE_TURBO_KP, MODE_TURBO_KI, MODE_TURBO_KD, 5000.0f);
    }

    ESP_LOGI(TAG, "Base=%d  kP=%.1f kI=%.2f kD=%.1f", BASE_SPEED, pid.kP, pid.kI, pid.kD);

    /* ===== main variables ===== */
    int weights[8] = { -5, -3, -1, 0, 0, 1, 3, 5 }; // center-dominant
    int last_turn_dir = 1;
    bool running = false;
    bool last_button_state = true;
    uint32_t tick = 0;
    const float dt = (float)DT_MS / 1000.0f;

    while (1) {
        /* ---------- button handling (toggle) ---------- */
        bool btn = gpio_get_level(PIN_BUTTON);
        if (last_button_state && !btn) { // pressed (pull-up -> goes low)
            vTaskDelay(pdMS_TO_TICKS(30)); // debounce
            if (gpio_get_level(PIN_BUTTON) == 0) {
                running = !running;
                if (!running) {
                    motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
                    motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
                    ESP_LOGI(TAG, "Stopped by button");
                } else {
                    pid.integral = 0.0f;
                    pid.prev_error = 0.0f;
                    ESP_LOGI(TAG, "Started by button");
                }
                while (gpio_get_level(PIN_BUTTON) == 0) vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        last_button_state = btn;

        if (!running) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* ---------- read sensors (via MUX) ---------- */
        int raw[8] = {0};
        int blk[8] = {0};
        int black_count = 0;
        for (int i = 0; i < 8; ++i) {
            mux_select((uint8_t)i);
            int v = 0, acc = 0;
            for (int k = 0; k < 3; ++k) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1, adc_ch, &v));
                acc += v;
            }
            v = acc / 3;
            raw[i] = v;
            blk[i] = (v < LINE_THRESHOLD) ? 1 : 0;
            if (IGNORE_SENSOR_1 && i == 1) blk[i] = 0;
            black_count += blk[i];
        }

        /* ---------- emergency stop (7 or 8 sensors) ---------- */
        if (black_count >= STOP_BLACK_MIN) {
            ESP_LOGW(TAG, "Emergency STOP: %d sensors black", black_count);
            motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
            running = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ---------- compute position error (center-dominant) ---------- */
        long sum = 0;
        int act = 0;
        for (int i = 0; i < 8; ++i) {
            if (blk[i]) { sum += weights[i]; act++; }
        }

        float position = 0.0f;
        if (act > 0) position = (float)sum / (float)act;
        else position = (float)5 * last_turn_dir; // if no sensor, bias to last dir

        /* We want the robot to be centered at position 0 */
        float error = 0.0f - position;

        /* ---------- PID compute (no leading) ---------- */
        float pid_out = pid_compute(&pid, error, dt);

        /* Compute motor commands */
        int left_cmd = clamp_int((int)(BASE_SPEED - pid_out), -PWM_MAX_DUTY, PWM_MAX_DUTY);
        int right_cmd = clamp_int((int)(BASE_SPEED + pid_out), -PWM_MAX_DUTY, PWM_MAX_DUTY);

        last_turn_dir = (error >= 0.0f) ? 1 : -1;

        /* ---------- apply motors ---------- */
        motor_set(LEDC_CHANNEL_0, PIN_MA2, left_cmd);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, right_cmd);

        /* ---------- telemetry send every tick (or throttle) ---------- */
        if ((tick++ % 5) == 0) { // reduce traffic if needed
            // send: tick,err,P,I,D,L,R,black_count,stop_state(0),gap_state(0)
            // P,I,D approximations (for telemetry) — compute separately for clarity
            float Pterm = pid.kP * error;
            float Iterm = pid.kI * pid.integral;
            float Dterm = pid.kD * ((error - pid.prev_error) / dt); // note prev_error changed by pid_compute; this approximates
            telemetry_send_csv((int)tick, (int)error, (int)Pterm, (int)Iterm, (int)Dterm,
                               left_cmd, right_cmd, black_count, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(DT_MS));
    }
}
