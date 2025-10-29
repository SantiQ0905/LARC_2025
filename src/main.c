#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_err.h"

// --- NEW ADC one-shot driver (ESP-IDF v5+) ---
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "Sensor-Test";

/* ---------- Pin map (from your table) ---------- */
#define PIN_MUX_S0   4
#define PIN_MUX_S1   5
#define PIN_MUX_S2   0
#define PIN_MUX_Y    1   // ADC input

#define PIN_ENCA1    2   // Left encoder A
#define PIN_ENCA2    3   // Left encoder B
#define PIN_ENCB1    23  // Right encoder A
#define PIN_ENCB2    22  // Right encoder B

/* ---------- Line sensor config ---------- */
#define NUM_LINE_CH   8
#define LINE_SETTLE_MS 2           // allow MUX/ADC to settle
#define LINE_THRESH   1800         // tune: ~12-bit ADC raw midpoint (0..~4095)

/* ---------- Encoder sampling ---------- */
typedef struct {
    gpio_num_t pinA;
    gpio_num_t pinB;
    int32_t count;
    int lastA;
} quad_t;

static quad_t enc_left  = { .pinA = PIN_ENCA1, .pinB = PIN_ENCA2, .count = 0, .lastA = 0 };
static quad_t enc_right = { .pinA = PIN_ENCB1, .pinB = PIN_ENCB2, .count = 0, .lastA = 0 };

/* ---------- Globals ---------- */
static adc_oneshot_unit_handle_t adc1;
static adc_channel_t adc_chan_muxY = ADC_CHANNEL_1; // GPIO1 on ESP32-C6

/* ---------- Helpers ---------- */
static inline void mux_select(uint8_t ch) {
    gpio_set_level(PIN_MUX_S0, (ch >> 0) & 1);
    gpio_set_level(PIN_MUX_S1, (ch >> 1) & 1);
    gpio_set_level(PIN_MUX_S2, (ch >> 2) & 1);
}

static int read_mux_channel(uint8_t ch) {
    mux_select(ch);
    vTaskDelay(pdMS_TO_TICKS(LINE_SETTLE_MS));
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1, adc_chan_muxY, &raw);
    if (err != ESP_OK) raw = -1;
    return raw;
}

static void encoder_sample(quad_t *e) {
    int a = gpio_get_level(e->pinA);
    int b = gpio_get_level(e->pinB);
    // Count on rising edge of A; direction decided by B
    if (e->lastA == 0 && a == 1) {
        if (b == 0) e->count++;    // one direction
        else        e->count--;    // opposite direction
    }
    e->lastA = a;
}

/* ---------- Tasks ---------- */
static void task_line_sensors(void *_) {
    ESP_LOGI(TAG, "Line sensor scan started");
    int raw[NUM_LINE_CH];
    int bin[NUM_LINE_CH];

    while (1) {
        for (int ch = 0; ch < NUM_LINE_CH; ch++) {
            raw[ch] = read_mux_channel(ch);
            // NOTE: depending on your array, black may be LOW or HIGH.
            // If inverted, flip the comparison.
            bin[ch] = (raw[ch] < LINE_THRESH) ? 1 : 0;
        }

        // Pretty print
        printf("RAW: ");
        for (int i = 0; i < NUM_LINE_CH; i++) {
            printf("%4d%s", raw[i], (i == NUM_LINE_CH-1) ? "  " : ",");
        }
        printf(" BIN: [");
        for (int i = 0; i < NUM_LINE_CH; i++) {
            printf("%d%s", bin[i], (i == NUM_LINE_CH-1) ? "" : " ");
        }
        printf("]\n");

        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 Hz
    }
}

static void task_encoders(void *_) {
    ESP_LOGI(TAG, "Encoder sampling started");
    // Init lastA for edge detection
    enc_left.lastA  = gpio_get_level(enc_left.pinA);
    enc_right.lastA = gpio_get_level(enc_right.pinA);

    while (1) {
        encoder_sample(&enc_left);
        encoder_sample(&enc_right);

        static TickType_t lastPrint = 0;
        TickType_t now = xTaskGetTickCount();
        if (now - lastPrint > pdMS_TO_TICKS(250)) {
            lastPrint = now;
            printf("ENC L: %ld   ENC R: %ld\n", (long)enc_left.count, (long)enc_right.count);
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // ~500 Hz sampling
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Sensors Test - starting…");

    /* --- GPIO: MUX select lines as outputs --- */
    gpio_config_t mux_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<PIN_MUX_S0) | (1ULL<<PIN_MUX_S1) | (1ULL<<PIN_MUX_S2),
        .pull_down_en = 0, .pull_up_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&mux_cfg));
    mux_select(0);

    /* --- GPIO: Encoders as inputs with pull-ups --- */
    gpio_config_t enc_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<PIN_ENCA1) | (1ULL<<PIN_ENCA2) | (1ULL<<PIN_ENCB1) | (1ULL<<PIN_ENCB2),
        .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&enc_cfg));

    /* --- ADC one-shot on GPIO1 (MUX_Y) --- */
    adc_oneshot_unit_init_cfg_t init = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init, &adc1));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // use default for chip
        .atten    = ADC_ATTEN_DB_11        // ~0-3.3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, adc_chan_muxY, &ch_cfg));

    /* --- Launch tasks --- */
    xTaskCreate(task_line_sensors, "line_sensors", 4096, NULL, 5, NULL);
    xTaskCreate(task_encoders,    "encoders",     4096, NULL, 5, NULL);
}
