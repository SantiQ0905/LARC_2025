#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* -------- Tag -------- */
static const char *TAG = "Motor-Test";

/* -------- Pin map -------- */
#define PIN_MA1     18     // Motor A PWM
#define PIN_MA2     19     // Motor A DIR
#define PIN_MB1     20     // Motor B PWM
#define PIN_MB2     21     // Motor B DIR

/* LEDC motor control: speed [-1023..1023] */
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
    ESP_LOGI(TAG, "Motor Test Starting");

    /* GPIO outputs for motor direction pins */
    gpio_config_t out_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<PIN_MA2) | (1ULL<<PIN_MB2)
    };
    gpio_config(&out_cfg);

    /* LEDC PWM: 20 kHz, 10-bit */
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

    /* Simple forward/backward test */
    while (1) {
        // Forward for 5 seconds
        ESP_LOGI(TAG, "FORWARD");
        motor_set(LEDC_CHANNEL_0, PIN_MA2, -700);  // Left motor inverted
        motor_set(LEDC_CHANNEL_1, PIN_MB2, 700);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Backward for 5 seconds
        ESP_LOGI(TAG, "BACKWARD");
        motor_set(LEDC_CHANNEL_0, PIN_MA2, 700);   // Left motor inverted
        motor_set(LEDC_CHANNEL_1, PIN_MB2, -700);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
