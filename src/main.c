// fa
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <stdbool.h>
#include <string.h>

static const char *TAG = "LFR-C6";

/* ========== Pines ========== */
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

/* ========== PWM / CONTROL ========== */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)

/* Modos */
#define MODE_SLOW_BASE_SPEED    650
#define MODE_SLOW_KP            80
#define MODE_NORMAL_BASE_SPEED  850
#define MODE_NORMAL_KP          120
#define MODE_FAST_BASE_SPEED    995
#define MODE_FAST_KP            150
#define MODE_TURBO_BASE_SPEED   2000
#define MODE_TURBO_KP           160

/* Sensores */
#define LINE_THRESHOLD   1600
#define IGNORE_SENSOR_1  1
#define INVERT_BOTH_MOTORS  0

/* Curvas que sí te funcionaban */
#define CURVE_PUSH_SPEED   850   // rueda externa
#define CURVE_INNER_SPEED  300   // rueda interna

/* ===== STOP DETECTION ===== */
#define STOP_BLACK_MIN           7
#define STOP_CONSEC_IN_SLOW     36
#define STOP_CONSEC_IN_NORMAL   22
#define STOP_CONSEC_IN_FAST     15
#define STOP_CONSEC_IN_TURBO    10
#define STOP_CONSEC_OUT          8
#define STOP_HOLD_MS           700
#define STOP_CREEP_SPEED       350
#define STOP_COOLDOWN_TICKS    150

typedef enum { ST_FOLLOW=0, ST_BRAKE, ST_HOLD, ST_CREEP, ST_COOLDOWN } stop_state_t;

/* ===== GAP (líneas punteadas) ===== */
typedef enum { GAP_NONE=0, GAP_COAST, GAP_SWEEP } gap_state_t;
#define GAP_COAST_TICKS     30      // avanza recto sobre blanco
#define GAP_COAST_SPEED     520
#define GAP_SWEEP_SPEED     580
#define GAP_SWEEP_MAX_TICKS 140

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

/* frenado suave del STOP */
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

/* ========== MAIN ========== */
void app_main(void) {
    ESP_LOGI(TAG, "Boot LFR (curva buena + GAP + STOP)");

    /* GPIO salida */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_MUX_S0) |
            (1ULL << PIN_MUX_S1) |
            (1ULL << PIN_MUX_S2) |
            (1ULL << PIN_MA2)    |
            (1ULL << PIN_MB2),
    };
    gpio_config(&out_cfg);

    /* botón */
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&in_cfg);

    /* dips */
    gpio_config_t dip_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_DIP_1) | (1ULL << PIN_DIP_2),
    };
    gpio_config(&dip_cfg);

    /* ADC */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));
    adc_channel_t adc_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, adc_ch, &ch_cfg));

    /* PWM motores */
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .duty_resolution = PWM_RES_BITS
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

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
    ESP_ERROR_CHECK(ledc_channel_config(&chA));
    ESP_ERROR_CHECK(ledc_channel_config(&chB));

    /* leer modo */
    int dip1 = gpio_get_level(PIN_DIP_1);
    int dip2 = gpio_get_level(PIN_DIP_2);
    int BASE_SPEED, KP;
    int stop_consec_in;
    if (dip1 == 1 && dip2 == 1) {
        BASE_SPEED = MODE_SLOW_BASE_SPEED; KP = MODE_SLOW_KP;
        stop_consec_in = STOP_CONSEC_IN_SLOW;
    } else if (dip1 == 0 && dip2 == 1) {
        BASE_SPEED = MODE_NORMAL_BASE_SPEED; KP = MODE_NORMAL_KP;
        stop_consec_in = STOP_CONSEC_IN_NORMAL;
    } else if (dip1 == 1 && dip2 == 0) {
        BASE_SPEED = MODE_FAST_BASE_SPEED; KP = MODE_FAST_KP;
        stop_consec_in = STOP_CONSEC_IN_FAST;
    } else {
        BASE_SPEED = MODE_TURBO_BASE_SPEED; KP = MODE_TURBO_KP;
        stop_consec_in = STOP_CONSEC_IN_TURBO;
    }

    bool running = false;
    bool last_btn = true;
    int weights[8] = { -3, -2, -1, 0, 0, 1, 2, 3 };
    int last_turn_dir = 1;
    uint32_t tick = 0;

    /* STOP vars */
    stop_state_t st = ST_FOLLOW;
    int stop_in_ctr = 0, stop_out_ctr = 0, cooldown = 0;

    /* GAP vars */
    gap_state_t gap = GAP_NONE;
    int gap_ctr = 0, gap_sweep_ctr = 0;

    while (1) {

        /* toggle botón */
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

        if (!running) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }

        /* leer sensores */
        int raw[8] = {0}, blk[8] = {0};
        int black_count_raw = 0;
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int v = 0, acc = 0;
            for (int s = 0; s < 3; s++) { ESP_ERROR_CHECK(adc_oneshot_read(adc1, adc_ch, &v)); acc += v; }
            v = acc / 3;
            raw[i] = v;
            int is_black = (v < LINE_THRESHOLD);
            black_count_raw += is_black;
            blk[i] = is_black;
            if (IGNORE_SENSOR_1 && i == 1) blk[i] = 0;
        }

        /* ===== STOP ===== */
        if (cooldown > 0) cooldown--;
        switch (st) {
            case ST_FOLLOW:
                if (cooldown == 0) {
                    if (black_count_raw >= STOP_BLACK_MIN) {
                        if (++stop_in_ctr >= stop_consec_in) {
                            stop_in_ctr = 0;
                            st = ST_BRAKE;
                        }
                    } else stop_in_ctr = 0;
                }
                break;
            case ST_HOLD:
                vTaskDelay(pdMS_TO_TICKS(STOP_HOLD_MS));
                st = ST_CREEP;
                stop_out_ctr = 0;
                break;
            case ST_CREEP: {
                int sp = STOP_CREEP_SPEED;
                if (INVERT_BOTH_MOTORS) sp = -sp;
                motor_set(LEDC_CHANNEL_0, PIN_MA2, sp);
                motor_set(LEDC_CHANNEL_1, PIN_MB2, sp);
                if (black_count_raw < 4) {
                    if (++stop_out_ctr >= STOP_CONSEC_OUT) {
                        st = ST_COOLDOWN;
                        cooldown = STOP_COOLDOWN_TICKS;
                    }
                } else stop_out_ctr = 0;
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            case ST_COOLDOWN:
                break;
            case ST_BRAKE:  // se maneja después del control
                break;
        }

        /* ===== GAP (si no ve nada) ===== */
        int active = 0;
        for (int i = 0; i < 8; i++) if (blk[i]) active++;

        if (active == 0) {
            if (gap == GAP_NONE) {
                gap = GAP_COAST;
                gap_ctr = GAP_COAST_TICKS;
            }
        } else {
            gap = GAP_NONE;
            gap_ctr = 0;
            gap_sweep_ctr = 0;
        }

        if (gap == GAP_COAST) {
            int sp = GAP_COAST_SPEED;
            if (INVERT_BOTH_MOTORS) sp = -sp;
            motor_set(LEDC_CHANNEL_0, PIN_MA2, sp);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, sp);
            if (--gap_ctr <= 0) {
                gap = GAP_SWEEP;
                gap_sweep_ctr = GAP_SWEEP_MAX_TICKS;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        } else if (gap == GAP_SWEEP) {
            int Ls = GAP_SWEEP_SPEED * last_turn_dir;
            int Rs = -GAP_SWEEP_SPEED * last_turn_dir;
            if (INVERT_BOTH_MOTORS) { Ls = -Ls; Rs = -Rs; }
            motor_set(LEDC_CHANNEL_0, PIN_MA2, Ls);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, Rs);
            if (active > 0 || --gap_sweep_ctr <= 0) {
                gap = GAP_NONE;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* ===== CONTROL DE LÍNEA (tu lógica de curvas) ===== */
        bool cL = blk[3];
        bool cR = blk[4];
        int L = 0, R = 0;

        if (cL && cR) {
            // centro
            L = BASE_SPEED;
            R = BASE_SPEED;
        } else if (cL && !cR) {
            // poquito a la izq
            L = (BASE_SPEED * 85) / 100;
            R = BASE_SPEED;
            last_turn_dir = -1;
        } else if (!cL && cR) {
            // poquito a la der
            L = BASE_SPEED;
            R = (BASE_SPEED * 85) / 100;
            last_turn_dir = 1;
        } else {
            // centros no ven → revisar orillas y medios
            int sum_mid = 0, act_mid = 0;
            for (int i = 1; i <= 6; i++) {
                if (blk[i]) { sum_mid += weights[i]; act_mid++; }
            }
            bool left_ext  = blk[0] || blk[1];
            bool right_ext = blk[6] || blk[7];

            if (left_ext && !right_ext) {
                // CURVA IZQUIERDA fuerte
                // si además hay algún medio prendido, bajamos un poco la externa
                if (act_mid > 0) {
                    L = 220;            // más frenada
                    R = (CURVE_PUSH_SPEED * 90) / 100;  // 10% menos para que cierre
                } else {
                    L = CURVE_INNER_SPEED;
                    R = CURVE_PUSH_SPEED;
                }
                last_turn_dir = -1;
            } else if (right_ext && !left_ext) {
                // CURVA DERECHA fuerte
                if (act_mid > 0) {
                    L = (CURVE_PUSH_SPEED * 90) / 100;
                    R = 220;
                } else {
                    L = CURVE_PUSH_SPEED;
                    R = CURVE_INNER_SPEED;
                }
                last_turn_dir = 1;
            } else if (act_mid > 0) {
                // hay medios → corrige con KP
                int err  = sum_mid;
                int turn = KP * err;
                // para que no oscile tanto en recta, bajo un poquito la ganancia real
                L = BASE_SPEED - turn;
                R = BASE_SPEED + turn;
                last_turn_dir = (err >= 0) ? 1 : -1;
            } else {
                // seguridad
                L = 320 * last_turn_dir;
                R = -320 * last_turn_dir;
            }
        }

        /* clamp */
        if (L > PWM_MAX_DUTY) L = PWM_MAX_DUTY;
        if (L < -PWM_MAX_DUTY) L = -PWM_MAX_DUTY;
        if (R > PWM_MAX_DUTY) R = PWM_MAX_DUTY;
        if (R < -PWM_MAX_DUTY) R = -PWM_MAX_DUTY;

        /* STOP: si entró, frena aquí */
        if (st == ST_BRAKE) {
            int curL = L, curR = R;
            soft_brake(LEDC_CHANNEL_0, PIN_MA2, LEDC_CHANNEL_1, PIN_MB2, &curL, &curR);
            st = ST_HOLD;
        } else {
            if (INVERT_BOTH_MOTORS) {
                motor_set(LEDC_CHANNEL_0, PIN_MA2, -L);
                motor_set(LEDC_CHANNEL_1, PIN_MB2, -R);
            } else {
                motor_set(LEDC_CHANNEL_0, PIN_MA2, L);
                motor_set(LEDC_CHANNEL_1, PIN_MB2, R);
            }
        }

        if (++tick % 50 == 0) {
            ESP_LOGI(TAG, "blk=[%d %d %d %d %d %d %d %d] L=%d R=%d gap=%d st=%d",
                     blk[0],blk[1],blk[2],blk[3],blk[4],blk[5],blk[6],blk[7],
                     L,R,gap,st);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}