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

/* ========== Pines de tu PCB ========== */
#define PIN_MUX_S0  4
#define PIN_MUX_S1  5
#define PIN_MUX_S2  0
#define PIN_MUX_Y   1   // ADC1_CH1

#define PIN_MA1     18  // PWM motor izq
#define PIN_MA2     19  // DIR motor izq
#define PIN_MB1     20  // PWM motor der
#define PIN_MB2     21  // DIR motor der

#define PIN_BUTTON  14  // Botón de inicio/parada

// DIP switch pins (SW3-1 y SW3-3)
#define PIN_DIP_1   6   // GPIO6 (IO6)
#define PIN_DIP_2   7   // GPIO7 (IO7)

/* ========== Config general ========== */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)

// Modos de velocidad basados en DIP switches
// DIP1=OFF, DIP2=OFF -> Modo LENTO (Slow)
#define MODE_SLOW_BASE_SPEED    450
#define MODE_SLOW_KP            80

// DIP1=ON, DIP2=OFF -> Modo NORMAL
#define MODE_NORMAL_BASE_SPEED  650
#define MODE_NORMAL_KP          120

// DIP1=OFF, DIP2=ON -> Modo RAPIDO (Fast)
#define MODE_FAST_BASE_SPEED    800
#define MODE_FAST_KP            140

// DIP1=ON, DIP2=ON -> Modo TURBO
#define MODE_TURBO_BASE_SPEED   950
#define MODE_TURBO_KP           160

#define LINE_THRESHOLD  1500

/* si un sensor no sirve: */
#define IGNORE_SENSOR_1   1

/* --- motor flip global --- 
   1 = invierte los DOS motores
   (porque tu compa los conectó al revés)
*/
#define INVERT_BOTH_MOTORS  0

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

/* ========== MAIN ========== */
void app_main(void) {
    ESP_LOGI(TAG, "Boot LFR");

    /* GPIO salidas */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_MUX_S0) |
            (1ULL << PIN_MUX_S1) |
            (1ULL << PIN_MUX_S2) |
            (1ULL << PIN_MA2)    |
            (1ULL << PIN_MB2),
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    /* GPIO entrada para botón con pull-up */
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_cfg);

    /* GPIO entrada para DIP switches (pull-ups ya en la placa) */
    gpio_config_t dip_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_DIP_1) | (1ULL << PIN_DIP_2),
        .pull_up_en = GPIO_PULLUP_DISABLE,   // pull-ups ya están en hardware
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dip_cfg);

    /* ADC one-shot */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    adc_channel_t adc_ch = ADC_CHANNEL_1;   // GPIO1
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
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1;
    chB.channel  = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&chA));
    ESP_ERROR_CHECK(ledc_channel_config(&chB));

    int weights[8] = { -3, -2, -1, 0, 0, 1, 2, 3 };
    int last_turn_dir = 1;
    uint32_t tick = 0;

    /* Leer DIP switches para determinar el modo */
    // DIP ON = LOW (0), DIP OFF = HIGH (1) debido a pull-ups
    int dip1 = gpio_get_level(PIN_DIP_1);  // 0=ON, 1=OFF
    int dip2 = gpio_get_level(PIN_DIP_2);  // 0=ON, 1=OFF
    
    int BASE_SPEED, KP;
    const char *mode_name;
    
    if (dip1 == 1 && dip2 == 1) {
        // Ambos OFF -> LENTO
        BASE_SPEED = MODE_SLOW_BASE_SPEED;
        KP = MODE_SLOW_KP;
        mode_name = "LENTO (Slow)";
    } else if (dip1 == 0 && dip2 == 1) {
        // DIP1=ON, DIP2=OFF -> NORMAL
        BASE_SPEED = MODE_NORMAL_BASE_SPEED;
        KP = MODE_NORMAL_KP;
        mode_name = "NORMAL";
    } else if (dip1 == 1 && dip2 == 0) {
        // DIP1=OFF, DIP2=ON -> RAPIDO
        BASE_SPEED = MODE_FAST_BASE_SPEED;
        KP = MODE_FAST_KP;
        mode_name = "RAPIDO (Fast)";
    } else {
        // Ambos ON -> TURBO
        BASE_SPEED = MODE_TURBO_BASE_SPEED;
        KP = MODE_TURBO_KP;
        mode_name = "TURBO";
    }
    
    ESP_LOGI(TAG, "Modo seleccionado: %s (BASE=%d, KP=%d)", mode_name, BASE_SPEED, KP);

    /* Esperar presión de botón para iniciar */
    ESP_LOGI(TAG, "Esperando botón para iniciar...");
    bool running = false;

    while (1) {
        /* Chequeo de botón con debounce */
        static bool last_button_state = true;  // true = no presionado (pull-up)
        bool current_button_state = gpio_get_level(PIN_BUTTON);
        
        /* Detectar presión de botón (transición de alto a bajo) */
        if (last_button_state == true && current_button_state == false) {
            vTaskDelay(pdMS_TO_TICKS(50));  // debounce
            if (gpio_get_level(PIN_BUTTON) == 0) {  // confirmar que sigue presionado
                running = !running;  // alternar estado
                if (running) {
                    ESP_LOGI(TAG, "¡Iniciado!");
                } else {
                    ESP_LOGI(TAG, "¡Detenido!");
                    motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
                    motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);
                }
                /* Esperar a que suelte el botón */
                while (gpio_get_level(PIN_BUTTON) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        last_button_state = current_button_state;
        
        /* Si no está corriendo, esperar y continuar */
        if (!running) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int raw[8]  = {0};
        int blk[8]  = {0};

        /* 1) leer los 8 sensores */
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int v = 0, acc = 0;
            for (int s = 0; s < 3; s++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1, adc_ch, &v));
                acc += v;
            }
            v = acc / 3;
            raw[i] = v;

            int is_black = (v < LINE_THRESHOLD) ? 1 : 0;
            if (IGNORE_SENSOR_1 && i == 1) {
                is_black = 0;
            }
            blk[i] = is_black;
        }

        /* 2) lógica de seguimiento */
        bool centerL = blk[3];
        bool centerR = blk[4];

        int left_cmd = 0;
        int right_cmd = 0;

        if (centerL && centerR) {
            // bien en el centro
            left_cmd  = BASE_SPEED;
            right_cmd = BASE_SPEED;
        } else if (centerL && !centerR) {
            // un poco a la izq → corrige derecha
            left_cmd  = (BASE_SPEED * 3) / 4;   // no lo frenes tanto
            right_cmd = BASE_SPEED;
            last_turn_dir = -1;
        } else if (!centerL && centerR) {
            // un poco a la der → corrige izq
            left_cmd  = BASE_SPEED;
            right_cmd = (BASE_SPEED * 3) / 4;
            last_turn_dir = 1;
        } else {
            // centros no ven
            int sum_mid = 0, act_mid = 0;
            for (int i = 1; i <= 6; i++) {
                if (blk[i]) {
                    sum_mid += weights[i];
                    act_mid++;
                }
            }

            if (act_mid > 0) {
                int err  = sum_mid;
                int turn = KP * err;
                left_cmd  = BASE_SPEED - turn;
                right_cmd = BASE_SPEED + turn;
                last_turn_dir = (err >= 0) ? 1 : -1;
            } else {
                // solo extremos o nada → buscar
                left_cmd  = 300 * last_turn_dir;
                right_cmd = -300 * last_turn_dir;
            }
        }

        /* 3) clamp */
        if (left_cmd > PWM_MAX_DUTY)  left_cmd = PWM_MAX_DUTY;
        if (left_cmd < -PWM_MAX_DUTY) left_cmd = -PWM_MAX_DUTY;
        if (right_cmd > PWM_MAX_DUTY) right_cmd = PWM_MAX_DUTY;
        if (right_cmd < -PWM_MAX_DUTY)right_cmd = -PWM_MAX_DUTY;

        /* 4) aplicar motores
              aquí invertimos los DOS porque ya nos dijiste que tu amigo los volteó
        */
        if (INVERT_BOTH_MOTORS) {
            motor_set(LEDC_CHANNEL_0, PIN_MA2, -left_cmd);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, -right_cmd);
        } else {
            motor_set(LEDC_CHANNEL_0, PIN_MA2, left_cmd);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, right_cmd);
        }

        /* 5) debug */
        if (++tick % 50 == 0) {
            ESP_LOGI(TAG,
                     "blk=[%d %d %d %d %d %d %d %d] raw=[%4d %4d %4d %4d %4d %4d %4d %4d] L=%d R=%d",
                     blk[0],blk[1],blk[2],blk[3],blk[4],blk[5],blk[6],blk[7],
                     raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],
                     left_cmd, right_cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}