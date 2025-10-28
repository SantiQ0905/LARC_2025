#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "LFR-C6";

/* ---------- Pins (C6) ---------- */
#define PIN_MUX_S0  4
#define PIN_MUX_S1  5
#define PIN_MUX_S2  0
#define PIN_MUX_Y   1   // ADC-capable per your board pinout

#define PIN_ENC_A1  2
#define PIN_ENC_A2  3
#define PIN_ENC_B1  23
#define PIN_ENC_B2  22

#define PIN_MA1     18  // PWM
#define PIN_MA2     19  // DIR
#define PIN_MB1     20  // PWM
#define PIN_MB2     21  // DIR

/* ---------- Globals ---------- */
volatile int32_t encA = 0;
volatile int32_t encB = 0;

static inline void mux_select(uint8_t i) {
    gpio_set_level(PIN_MUX_S0, i & 0x01);
    gpio_set_level(PIN_MUX_S1, (i >> 1) & 0x01);
    gpio_set_level(PIN_MUX_S2, (i >> 2) & 0x01);
    ets_delay_us(5);
}

/* ---------- Encoders (quadrature) ---------- */
static void IRAM_ATTR isr_enc_a1(void *arg) {
    int a = gpio_get_level(PIN_ENC_A1);
    int b = gpio_get_level(PIN_ENC_A2);
    encA += (a == b) ? +1 : -1;
}
static void IRAM_ATTR isr_enc_a2(void *arg) {
    int a = gpio_get_level(PIN_ENC_A1);
    int b = gpio_get_level(PIN_ENC_A2);
    encA += (a != b) ? +1 : -1;
}
static void IRAM_ATTR isr_enc_b1(void *arg) {
    int a = gpio_get_level(PIN_ENC_B1);
    int b = gpio_get_level(PIN_ENC_B2);
    encB += (a == b) ? +1 : -1;
}
static void IRAM_ATTR isr_enc_b2(void *arg) {
    int a = gpio_get_level(PIN_ENC_B1);
    int b = gpio_get_level(PIN_ENC_B2);
    encB += (a != b) ? +1 : -1;
}

/* ---------- LEDC (PWM) ---------- */
static void motor_set(int pwm_chan, gpio_num_t pin_dir, int speed) {
    // speed: -1023..1023
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

/* ---------- App ---------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Boot");

    /* GPIO directions */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL<<PIN_MUX_S0) | (1ULL<<PIN_MUX_S1) | (1ULL<<PIN_MUX_S2) |
            (1ULL<<PIN_MA2)    | (1ULL<<PIN_MB2)
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask =
            (1ULL<<PIN_ENC_A1) | (1ULL<<PIN_ENC_A2) |
            (1ULL<<PIN_ENC_B1) | (1ULL<<PIN_ENC_B2)
    };
    gpio_config(&in_cfg);

    // ADC one-shot on MUX_Y (adjust channel if needed for your board)
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    // NOTE: GPIO->ADC channel mapping differs by chip/board.
    // On many ESP32-C6 devkits GPIO1 is ADC1_CH1. If your readings are -1,
    // change to ADC_CHANNEL_0 or the proper channel for GPIO1.
    adc_channel_t mux_y_ch = ADC_CHANNEL_1;

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, mux_y_ch, &ch_cfg));

    // Encoders: ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_A1, isr_enc_a1, NULL);
    gpio_isr_handler_add(PIN_ENC_A2, isr_enc_a2, NULL);
    gpio_isr_handler_add(PIN_ENC_B1, isr_enc_b1, NULL);
    gpio_isr_handler_add(PIN_ENC_B2, isr_enc_b2, NULL);

    // LEDC timer
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 20000,         // 20kHz – quiet
        .clk_cfg          = LEDC_AUTO_CLK,
        .duty_resolution  = LEDC_TIMER_10_BIT  // 0..1023
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // LEDC channels (MA1 on CH0, MB1 on CH1)
    ledc_channel_config_t c0 = {
        .gpio_num   = PIN_MA1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0
    };
    ledc_channel_config_t c1 = c0;
    c1.gpio_num = PIN_MB1;
    c1.channel  = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&c0));
    ESP_ERROR_CHECK(ledc_channel_config(&c1));

    // Safe stop
    motor_set(LEDC_CHANNEL_0, PIN_MA2, 0);
    motor_set(LEDC_CHANNEL_1, PIN_MB2, 0);

    int base = 700;       // 0..1023
    int kP   = 120;       // steering gain
    int threshold = 1800; // tune for your floor/line

    while (1) {
        // Read 8 sensors via mux
        uint8_t bits[8] = {0};
        int raw[8] = {0};
        for (int i = 0; i < 8; i++) {
            mux_select(i);
            int v;
            // average 3 samples
            int acc = 0;
            for (int s=0; s<3; s++) {
                ESP_ERROR_CHECK(adc_oneshot_read(adc1, mux_y_ch, &v));
                acc += v;
            }
            v = acc/3;
            raw[i] = v;
            bits[i] = (v < threshold) ? 1 : 0;   // invert if needed
        }

        // Simple proportional line-follow
        int weights[8] = {-3,-2,-1,0,0,1,2,3};
        int sum=0, act=0;
        for (int i=0;i<8;i++){ if(bits[i]){ sum+=weights[i]; act++; } }
        int err = act ? sum : 0;
        int turn = kP * err;

        motor_set(LEDC_CHANNEL_0, PIN_MA2, base - turn);
        motor_set(LEDC_CHANNEL_1, PIN_MB2, base + turn);

        // Telemetry (200 ms)
        static uint32_t tick = 0;
        if (++tick % 40 == 0) {
            int32_t a = encA, b = encB;
            ESP_LOGI(TAG, "bits=%d%d%d%d%d%d%d%d  raw=%d %d %d %d %d %d %d %d  encA=%" PRId32 " encB=%" PRId32,
                     bits[0],bits[1],bits[2],bits[3],bits[4],bits[5],bits[6],bits[7],
                     raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7], a, b);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
