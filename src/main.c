#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "LFR-C6";

/* ---------- Pines (tu PCB) ---------- */
#define PIN_MUX_S0  4
#define PIN_MUX_S1  5
#define PIN_MUX_S2  0
#define PIN_MUX_Y   1      // GPIO1 -> ADC_CHANNEL_1

#define PIN_MA1     18     // Motor A PWM (izq)
#define PIN_MA2     19     // Motor A DIR  (izq)
#define PIN_MB1     20     // Motor B PWM (der)
#define PIN_MB2     21     // Motor B DIR  (der)

/* ---------- Config de control ---------- */
#define PWM_FREQ_HZ         20000
#define PWM_RES_BITS        10                // 0..1023
#define PWM_MAX_DUTY        ((1<<PWM_RES_BITS)-1)
#define BASE_SPEED          500               // velocidad base (0..1023)
#define KP                  90                // ganancia proporcional
#define LOOP_MS             5                 // periodo del control

/* Lógica de negro/blanco:
   raw bajo = negro (TCRT5000 sobre línea) => blk=1 por defecto.
   Si tu hardware fuese al revés, pon INVERT_BLACK_LOGIC = 1. */
#define INVERT_BLACK_LOGIC  0  // 0: negro = raw BAJO; 1: negro = raw ALTO

/* Política de avance:
   - MIN_BLACK_SENSORS: cantidad mínima de sensores en negro para avanzar
   - STOP_IF_ANY_WHITE: si 1, se detiene si ALGÚN sensor ve blanco (muy estricto)
                        si 0, solo exige el mínimo de negros (recomendado)       */
#define MIN_BLACK_SENSORS   2
#define STOP_IF_ANY_WHITE   0

/* ---------- Helpers ---------- */
static inline void mux_select(uint8_t i) {
    gpio_set_level(PIN_MUX_S0, i & 0x01);
    gpio_set_level(PIN_MUX_S1, (i >> 1) & 0x01);
    gpio_set_level(PIN_MUX_S2, (i >> 2) & 0x01);
    esp_rom_delay_us(5);   // settle
}

static void motor_set(ledc_channel_t pwm_chan, gpio_num_t pin_dir, int speed) {
    if (speed >= 0) {
        gpio_set_level(pin_dir, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, speed);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_chan);
    } else {
        gpio_set_level(pin_dir, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_chan, -speed);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_chan);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Boot");

    /* GPIO salidas: MUX y DIR motores */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL<<PIN_MUX_S0) | (1ULL<<PIN_MUX_S1) | (1ULL<<PIN_MUX_S2) |
            (1ULL<<PIN_MA2)    | (1ULL<<PIN_MB2),
        .pull_down_en = 0,
        .pull_up_en   = 0,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    /* ADC one-shot (GPIO1 -> ADC_CHANNEL_1) */
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    adc_channel_t mux_y_ch = ADC_CHANNEL_1;
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12  // C6: usar DB_12 (DB_11 deprecado)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, mux_y_ch, &ch_cfg));

    /* LEDC PWM @20kHz, 10-bit */
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

    /* Estado sensores */
    int raw[8]     = {0};
    int minv[8];   for (int i=0;i<8;i++) minv[i] = 4095;
    int maxv[8];   for (int i=0;i<8;i++) maxv[i] = 0;
    int thr[8];    memset(thr, 0, sizeof(thr));
    uint8_t blk[8] = {0};  // 1 = NEGRO (LED apagado, según tu regla)

    /* Pesos para el error (centro entre 3 y 4) */
    int w[8] = {-3,-2,-1,0,0,1,2,3};

    ESP_LOGI(TAG, "Starting loop… INVERT_BLACK_LOGIC=%d", INVERT_BLACK_LOGIC);

    uint32_t tick = 0;
    while (1) {
        /* Leer 8 sensores y actualizar min/max/umbral */
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int v = 0, acc = 0;
            for (int s=0; s<3; s++) {  // promedio simple
                ESP_ERROR_CHECK(adc_oneshot_read(adc1, mux_y_ch, &v));
                acc += v;
            }
            v = acc / 3;
            raw[i] = v;

            if (v < minv[i]) minv[i] = v;
            if (v > maxv[i]) maxv[i] = v;

            int range = maxv[i] - minv[i];
            int t = (range < 50) ? (minv[i] + range/2) : ((minv[i] + maxv[i]) / 2);
            thr[i] = t;

            uint8_t is_black;
            if (!INVERT_BLACK_LOGIC) {      // negro = raw BAJO
                is_black = (raw[i] <= thr[i]) ? 1 : 0;
            } else {                         // negro = raw ALTO
                is_black = (raw[i] >= thr[i]) ? 1 : 0;
            }
            blk[i] = is_black;
        }

        /* Política de avance según tu regla */
        int count_black = 0;
        for (int i=0;i<8;i++) count_black += blk[i];
        int count_white = 8 - count_black;

        int left=0, right=0;
        bool can_move;

        if (STOP_IF_ANY_WHITE) {
            // MUY estricto: solo avanza si TODOS ven negro y hay suficiente negros
            can_move = (count_white == 0) && (count_black >= MIN_BLACK_SENSORS);
        } else {
            // Recomendado: avanza si hay suficientes en negro
            can_move = (count_black >= MIN_BLACK_SENSORS);
        }

        if (!can_move) {
            left = 0;
            right = 0;
        } else {
            // Control proporcional sobre los sensores que sí ven negro
            int sum_w = 0, c = 0;
            for (int i=0;i<8;i++) if (blk[i]) { sum_w += w[i]; c++; }
            int err = (c > 0) ? sum_w : 0;

            int turn = KP * err;
            left  = BASE_SPEED - turn;
            right = BASE_SPEED + turn;

            if (left  >  PWM_MAX_DUTY) left  =  PWM_MAX_DUTY;
            if (left  < -PWM_MAX_DUTY) left  = -PWM_MAX_DUTY;
            if (right >  PWM_MAX_DUTY) right =  PWM_MAX_DUTY;
            if (right < -PWM_MAX_DUTY) right = -PWM_MAX_DUTY;
        }

        // Aplica motores (invierte uno si lo requiere tu mecánica)
        motor_set(LEDC_CHANNEL_0, PIN_MA2, left);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, right);

        // Debug cada ~50 ciclos
        if ((++tick % 50) == 0) {
            printf("blk(off=negro)=[%d %d %d %d %d %d %d %d]  "
                   "thr=[%4d %4d %4d %4d %4d %4d %4d %4d]  "
                   "raw=[%4d %4d %4d %4d %4d %4d %4d %4d]  "
                   "move=%d (black=%d white=%d)\n",
                   blk[0],blk[1],blk[2],blk[3],blk[4],blk[5],blk[6],blk[7],
                   thr[0],thr[1],thr[2],thr[3],thr[4],thr[5],thr[6],thr[7],
                   raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],
                   can_move, count_black, count_white);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}