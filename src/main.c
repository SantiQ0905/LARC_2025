#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <stdbool.h>
#include <math.h>

static const char *TAG = "LFR-C6";

/* ========== Pinout ========== */
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

/* ========== PWM Configuration ========== */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)

/* ========== Control Modes ========== */
#define MODE_SLOW_BASE_SPEED    650
#define MODE_SLOW_KP            90
#define MODE_NORMAL_BASE_SPEED  850
#define MODE_NORMAL_KP          120
#define MODE_FAST_BASE_SPEED    950
#define MODE_FAST_KP            140
#define MODE_TURBO_BASE_SPEED   2000
#define MODE_TURBO_KP           160

/* ========== PID tuning ========== */
#define LINE_THRESHOLD   1500
#define KP_LEAD_FACTOR   1.15f   // anticipative gain
#define KD_GAIN          40.0f   // derivative for smoother turns
#define CURVE_BLEND      0.4f
#define MAX_TURN_SCALE   0.6f

/* ========== Stop logic ========== */
#define STOP_BLACK_MIN        7
#define STOP_CONSEC_IN_NORMAL 20
#define STOP_CONSEC_OUT        8
#define STOP_HOLD_MS           700
#define STOP_COOLDOWN_TICKS    160
#define STOP_CREEP_SPEED       350
#define SLOWDOWN_DUTY          400

typedef enum { ST_FOLLOW=0, ST_BRAKE, ST_HOLD, ST_CREEP, ST_COOLDOWN } stop_state_t;

/* ========== Helpers ========== */
static inline void mux_select(uint8_t ch) {
    gpio_set_level(PIN_MUX_S0, (ch >> 0) & 1);
    gpio_set_level(PIN_MUX_S1, (ch >> 1) & 1);
    gpio_set_level(PIN_MUX_S2, (ch >> 2) & 1);
    esp_rom_delay_us(5);
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

static inline void soft_brake(ledc_channel_t chL, gpio_num_t dirL,
                              ledc_channel_t chR, gpio_num_t dirR,
                              int *pL, int *pR) {
    for (int i = 0; i < 12; i++) {
        *pL = (*pL * 7) / 10;
        *pR = (*pR * 7) / 10;
        motor_set(chL, dirL, *pL);
        motor_set(chR, dirR, *pR);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    *pL = 0; *pR = 0;
    motor_set(chL, dirL, 0);
    motor_set(chR, dirR, 0);
}

/* ========== MAIN APP ========== */
void app_main(void) {
    ESP_LOGI(TAG, "Booting LFR (PID Lead + Smooth Turns + Stop States)");

    /* GPIO setup */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_MUX_S0) | (1ULL << PIN_MUX_S1) | (1ULL << PIN_MUX_S2) |
                        (1ULL << PIN_MA2) | (1ULL << PIN_MB2),
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = { .mode = GPIO_MODE_INPUT, .pin_bit_mask = (1ULL << PIN_BUTTON), .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&in_cfg);

    gpio_config_t dip_cfg = { .mode = GPIO_MODE_INPUT, .pin_bit_mask = (1ULL << PIN_DIP_1) | (1ULL << PIN_DIP_2) };
    gpio_config(&dip_cfg);

    /* ADC config */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, adc_ch, &ch_cfg));

    /* PWM setup */
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = PWM_RES_BITS
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t chA = { .gpio_num = PIN_MA1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0 };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1; chB.channel = LEDC_CHANNEL_1;
    ledc_channel_config(&chA); ledc_channel_config(&chB);

    /* Speed mode */
    int dip1 = gpio_get_level(PIN_DIP_1);
    int dip2 = gpio_get_level(PIN_DIP_2);
    int BASE_SPEED, KP;
    if (dip1 == 1 && dip2 == 1) { BASE_SPEED = MODE_SLOW_BASE_SPEED; KP = MODE_SLOW_KP; }
    else if (dip1 == 0 && dip2 == 1) { BASE_SPEED = MODE_NORMAL_BASE_SPEED; KP = MODE_NORMAL_KP; }
    else if (dip1 == 1 && dip2 == 0) { BASE_SPEED = MODE_FAST_BASE_SPEED; KP = MODE_FAST_KP; }
    else { BASE_SPEED = MODE_TURBO_BASE_SPEED; KP = MODE_TURBO_KP; }

    bool running = false, last_btn = true;
    stop_state_t st = ST_FOLLOW;
    int stop_in_ctr = 0, stop_out_ctr = 0, cooldown = 0;
    int prev_err = 0;

    /* ===== Main Loop ===== */
    while (1) {
        bool btn = gpio_get_level(PIN_BUTTON);
        if (last_btn && !btn) {
            vTaskDelay(pdMS_TO_TICKS(40));
            if (gpio_get_level(PIN_BUTTON) == 0) {
                running = !running;
                if (!running) {
                    motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
                    motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
                }
                while (gpio_get_level(PIN_BUTTON) == 0) vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        last_btn = btn;
        if (!running) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        int raw[8], blk[8];
        int black_count_raw = 0;
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int acc = 0, v = 0;
            for (int s = 0; s < 3; s++) { adc_oneshot_read(adc1, adc_ch, &v); acc += v; }
            raw[i] = acc / 3;
            blk[i] = (raw[i] < LINE_THRESHOLD);
            black_count_raw += blk[i];
        }

        /* Slowdown before stop */
        if (black_count_raw >= 8 && st == ST_FOLLOW) {
            motor_set(LEDC_CHANNEL_0, PIN_MA2, SLOWDOWN_DUTY);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, SLOWDOWN_DUTY);
            if (++stop_in_ctr > STOP_CONSEC_IN_NORMAL) st = ST_BRAKE;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Stop machine states */
        if (cooldown > 0) cooldown--;
        switch (st) {
            case ST_FOLLOW:
                if (black_count_raw >= STOP_BLACK_MIN && cooldown == 0) {
                    if (++stop_in_ctr > STOP_CONSEC_IN_NORMAL) st = ST_BRAKE;
                } else stop_in_ctr = 0;
                break;

            case ST_BRAKE: {
                int l = 0, r = 0;
                soft_brake(LEDC_CHANNEL_0, PIN_MA2, LEDC_CHANNEL_1, PIN_MB2, &l, &r);
                st = ST_HOLD;
                break;
            }

            case ST_HOLD:
                vTaskDelay(pdMS_TO_TICKS(STOP_HOLD_MS));
                st = ST_CREEP;
                break;

            case ST_CREEP:
                motor_set(LEDC_CHANNEL_0, PIN_MA2, STOP_CREEP_SPEED);
                motor_set(LEDC_CHANNEL_1, PIN_MB2, STOP_CREEP_SPEED);
                if (black_count_raw < 4) {
                    if (++stop_out_ctr >= STOP_CONSEC_OUT) {
                        st = ST_COOLDOWN;
                        cooldown = STOP_COOLDOWN_TICKS;
                    }
                } else stop_out_ctr = 0;
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;

            case ST_COOLDOWN: break;
        }

        /* PID lead correction */
    /* Double the effect of sensor 1 (blk[0]) as requested */
    int err = (blk[5] + 2*blk[6] + 3*blk[7]) - (2*blk[0] + 2*blk[1] + 3*blk[2]);
        int dErr = err - prev_err;
        prev_err = err;

        float pid_out = (KP * err * KP_LEAD_FACTOR) + (KD_GAIN * dErr);
        float turn = pid_out * MAX_TURN_SCALE / 10.0f;

        int L = BASE_SPEED - (int)(turn * (1.0f - CURVE_BLEND));
        int R = BASE_SPEED + (int)(turn * (1.0f - CURVE_BLEND));

        if (L > PWM_MAX_DUTY) L = PWM_MAX_DUTY;
        if (R > PWM_MAX_DUTY) R = PWM_MAX_DUTY;
        if (L < 0) L = 0;
        if (R < 0) R = 0;

        motor_set(LEDC_CHANNEL_0, PIN_MA2, L);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, R);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}







