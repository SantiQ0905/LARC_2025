#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include <stdio.h>

static const char *TAG = "ONE_SENSOR_AUTO";

/* MUX pins (tu PCB) */
#define PIN_MUX_S0   4
#define PIN_MUX_S1   5
#define PIN_MUX_S2   0
#define PIN_MUX_Y    1   // ADC input (ESP32-C6)

#define SENSOR_CH        0   // canal del mux a leer (0..7)
#define MUX_SETTLE_MS    3

static adc_oneshot_unit_handle_t adc1;
static adc_channel_t adc_chan_muxY = ADC_CHANNEL_1;

/* Seleccionar canal del mux */
static inline void mux_select(uint8_t ch) {
    gpio_set_level(PIN_MUX_S0, (ch >> 0) & 1);
    gpio_set_level(PIN_MUX_S1, (ch >> 1) & 1);
    gpio_set_level(PIN_MUX_S2, (ch >> 2) & 1);
}

/* Leer canal actual */
static int read_one_channel(uint8_t ch) {
    mux_select(ch);
    vTaskDelay(pdMS_TO_TICKS(MUX_SETTLE_MS));

    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1, adc_chan_muxY, &raw);
    if (err != ESP_OK) raw = -1;
    return raw;
}

static void task_single_sensor(void *_) {
    ESP_LOGI(TAG, "Single sensor auto-cal started (canal %d)", SENSOR_CH);

    // vamos a trackear el rango que vemos
    int seen_min = 4095;
    int seen_max = 0;

    while (1) {
        int raw = read_one_channel(SENSOR_CH);

        if (raw >= 0) {
            // actualizamos min/max vistos
            if (raw < seen_min) seen_min = raw;
            if (raw > seen_max) seen_max = raw;
        }

        // calculamos threshold en medio del rango actual
        int threshold_dynamic = (seen_min + seen_max) / 2;

        const char *estado;
        if (raw < 0) {
            estado = "LECTURA_ERROR";
        } else {
            // negro debería ser más oscuro -> raw más BAJO
            if (raw <= threshold_dynamic) {
                estado = "NEGRO";
            } else {
                estado = "BLANCO";
            }
        }

        printf("CH%d RAW=%d  min=%d max=%d thr=%d  -> %s\n",
               SENSOR_CH,
               raw,
               seen_min,
               seen_max,
               threshold_dynamic,
               estado);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== ONE SENSOR AUTO THRESH ===");

    // config pines mux como salida
    gpio_config_t mux_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL<<PIN_MUX_S0) |
            (1ULL<<PIN_MUX_S1) |
            (1ULL<<PIN_MUX_S2),
        .pull_down_en = 0,
        .pull_up_en   = 0,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&mux_cfg));
    mux_select(SENSOR_CH);

    // init ADC en IO1
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // 12-bit
        .atten    = ADC_ATTEN_DB_12        // ~0-3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, adc_chan_muxY, &ch_cfg));

    xTaskCreate(task_single_sensor, "single_sensor", 4096, NULL, 5, NULL);
}