#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <inttypes.h>
#include <stdbool.h>

static const char *TAG = "LFR-C6";

/* -------- Pin map -------- */
#define PIN_MUX_S0  4
#define PIN_MUX_S1  5
#define PIN_MUX_S2  0
#define PIN_MUX_Y   1      // ADC input from MUX

#define PIN_MA1     18     // Motor A PWM
#define PIN_MA2     19     // Motor A DIR
#define PIN_MB1     20     // Motor B PWM
#define PIN_MB2     21     // Motor B DIR

static inline void mux_select(uint8_t i) {
    gpio_set_level(PIN_MUX_S0, i & 0x01);
    gpio_set_level(PIN_MUX_S1, (i >> 1) & 0x01);
    gpio_set_level(PIN_MUX_S2, (i >> 2) & 0x01);
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

void app_main(void) {
    ESP_LOGI(TAG, "Boot");

    /* GPIO outputs for MUX and motor dirs */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL<<PIN_MUX_S0) | (1ULL<<PIN_MUX_S1) | (1ULL<<PIN_MUX_S2) |
            (1ULL<<PIN_MA2)    | (1ULL<<PIN_MB2)
    };
    gpio_config(&out_cfg);

    /* --- ADC setup --- */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    adc_channel_t mux_y_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, mux_y_ch, &ch_cfg));

    /* --- PWM setup --- */
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 20000,
        .clk_cfg         = LEDC_AUTO_CLK,
        .duty_resolution = LEDC_TIMER_10_BIT
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t chA = {
        .gpio_num   = PIN_MA1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1;
    chB.channel  = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&chA));
    ESP_ERROR_CHECK(ledc_channel_config(&chB));

    /* --- Params --- */
    int base_speed = 600;
    int kP = 120;
    int threshold = 1500;

    // último giro (por si pierde la línea)
    int last_turn_dir = 1;

    while (1) {
        uint8_t bits[8] = {0};
        int raw[8] = {0};

        // leer 8 sensores
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int v = 0, acc = 0;
            for (int s = 0; s < 3; s++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1, mux_y_ch, &v));
                acc += v;
            }
            v = acc / 3;
            raw[i] = v;
            // OJO: en tu pista el negro da BAJO, el blanco ALTO → ajusta si es al revés
            bits[i] = (v < threshold) ? 1 : 0;  // 1 = negro
        }

        // pesos, pero vamos a ignorar 0 y 7 a menos que no haya nada más
        int weights[8] = {-3,-2,-1,0,0,1,2,3};

        bool cL = bits[3];
        bool cR = bits[4];

        int left = 0, right = 0;

        if (cL && cR) {
            // línea justo en el centro → ir derecho
            left = base_speed;
            right = base_speed;
        } else if (cL && !cR) {
            // un poco cargado a la izquierda → corregir derecha suave
            left = base_speed / 2;
            right = base_speed;
            last_turn_dir = -1;
        } else if (!cL && cR) {
            // un poco cargado a la derecha → corregir izquierda suave
            left = base_speed;
            right = base_speed / 2;
            last_turn_dir = 1;
        } else {
            // centros no ven nada
            // ¿hay algo en 1..6?
            int sum_mid = 0, act_mid = 0;
            for (int i = 1; i <= 6; i++) {   // <--- ignoramos extremos aquí
                if (bits[i]) {
                    sum_mid += weights[i];
                    act_mid++;
                }
            }

            if (act_mid > 0) {
                // usar el proporcional pero SOLO con los sensores 1..6
                int err = sum_mid;       // ya está ponderado
                int turn = kP * err;
                left  = base_speed - turn;
                right = base_speed + turn;
                last_turn_dir = (err >= 0) ? 1 : -1;
            } else {
                // solo hay extremos (0 o 7) o nada → NO los persigas duro
                // mejor gira despacito hacia la última dirección buena
                left  = 300 * last_turn_dir;
                right = -300 * last_turn_dir;
            }
        }

        // clamp
        if (left > 1023) left = 1023;
        if (left < -1023) left = -1023;
        if (right > 1023) right = 1023;
        if (right < -1023) right = -1023;

        // IMPORTANTE: aquí ya NO invertimos el derecho
        motor_set(LEDC_CHANNEL_0, PIN_MA2, left);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, right);

        static uint32_t tick = 0;
        if (++tick % 50 == 0) {
            ESP_LOGI(TAG, "bits=%d%d%d%d%d%d%d%d  L=%d R=%d  raw=%d,%d,%d,%d,%d,%d,%d,%d",
                     bits[0],bits[1],bits[2],bits[3],
                     bits[4],bits[5],bits[6],bits[7],
                     left, right,
                     raw[0],raw[1],raw[2],raw[3],
                     raw[4],raw[5],raw[6],raw[7]);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}