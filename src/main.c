#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <stdbool.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

static const char *TAG = "LFR-C6";

/* ========== WIFI CONFIG ========== */
#define WIFI_SSID       "SpeedBird_LFR"
#define WIFI_PASS       "speedbird123"
#define WIFI_CHANNEL    1
#define MAX_STA_CONN    4

/* Global telemetry buffer for WiFi transmission */
static char telemetry_buffer[256] = {0};
static bool telemetry_updated = false;
static httpd_handle_t server = NULL;

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

/* ========== PWM / CONTROL ========== */
#define PWM_FREQ_HZ     20000
#define PWM_RES_BITS    10
#define PWM_MAX_DUTY    ((1 << PWM_RES_BITS) - 1)

#define MODE_SLOW_BASE_SPEED    650
#define MODE_SLOW_KP            80
#define MODE_NORMAL_BASE_SPEED  850
#define MODE_NORMAL_KP          120
#define MODE_FAST_BASE_SPEED    995
#define MODE_FAST_KP            150
#define MODE_TURBO_BASE_SPEED   2000
#define MODE_TURBO_KP           160

/* SENSORS */
#define LINE_THRESHOLD   1650
#define IGNORE_SENSOR_1  0
#define INVERT_BOTH_MOTORS  0

/* CLEAR CURVES */
#define CURVE_PUSH_SPEED   850
#define CURVE_INNER_SPEED  300

/* ===== STOP DETECTION PARAMS ===== */
#define STOP_BLACK_MIN           7
#define STOP_CONSEC_IN_SLOW     40   // ~180 ms (@ 5 ms loop)
#define STOP_CONSEC_IN_NORMAL   22   // ~95 ms
#define STOP_CONSEC_IN_FAST     15   // ~65 ms
#define STOP_CONSEC_IN_TURBO    10   // ~40 ms
#define STOP_CONSEC_OUT          8
#define STOP_HOLD_MS           700
#define STOP_CREEP_SPEED       350
#define STOP_COOLDOWN_TICKS    160   // ~800 ms

typedef enum { ST_FOLLOW=0, ST_BRAKE, ST_HOLD, ST_CREEP, ST_COOLDOWN } stop_state_t;

/* ===== GAP RECOVERY (DOTTED LINE) ===== */
typedef enum { GAP_NONE=0, GAP_COAST, GAP_SWEEP } gap_state_t;
#define GAP_COAST_TICKS     24     // was 14 → ~120 ms coast
#define GAP_COAST_SPEED     450    // was 320
#define GAP_SWEEP_SPEED     550    // was 420
#define GAP_SWEEP_MAX_TICKS 200    // was 120

/* ===== PD CONTROL KNOBS ===== */
// PD strength
#define KD_DIVISOR          2      // was 4 → stronger D
#define KP_BOOST_NUM        160    // was 135 → 1.60x KP on sharp turns
#define KP_BOOST_DEN        100
#define KD_BOOST_NUM        200    // was 150 → 2.00x KD on sharp turns
#define KD_BOOST_DEN        100
#define SPEED_DROP_NUM       70    // was 85 → drop speed to 70% on sharp turns
#define SPEED_DROP_DEN      100

/* ========== HELPERS ========== */
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

/* SOFT BRAKE */
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

/* ========== WIFI & HTTP SERVER ========== */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station joined, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station left, AID=%d", event->aid);
    }
}

static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s password:%s channel:%d",
             WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}

/* HTTP handlers */
static esp_err_t root_get_handler(httpd_req_t *req) {
    const char* html = 
        "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>ESP32 LFR Telemetry</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:20px;background:#1a1a1a;color:#fff;}"
        "h1{color:#4CAF50;}#data{font-size:18px;margin:20px 0;}"
        ".sensor{display:inline-block;width:40px;height:40px;margin:5px;"
        "border:2px solid #333;text-align:center;line-height:40px;border-radius:5px;}"
        ".white{background:#fff;color:#000;}.black{background:#000;color:#fff;}"
        "#status{margin:10px 0;padding:10px;background:#333;border-radius:5px;}"
        "</style></head><body>"
        "<h1>ESP32 Line Follower Robot</h1>"
        "<div id='status'>Connecting...</div>"
        "<h2>Sensor Array (0=Black, 1=White)</h2>"
        "<div id='sensors'></div>"
        "<div id='data'>Waiting for data...</div>"
        "<script>"
        "function update(){"
        "fetch('/telemetry').then(r=>r.text()).then(d=>{"
        "document.getElementById('data').innerHTML='<pre>'+d+'</pre>';"
        "var vals=d.match(/\\d/g);"
        "if(vals&&vals.length>=8){"
        "var html='';"
        "for(var i=0;i<8;i++){"
        "var cls=vals[i]=='1'?'white':'black';"
        "html+='<div class=\"sensor '+cls+'\">'+vals[i]+'</div>';"
        "}"
        "document.getElementById('sensors').innerHTML=html;"
        "document.getElementById('status').innerHTML='Connected - Live Data';"
        "}"
        "}).catch(e=>{"
        "document.getElementById('status').innerHTML='Connection Error';"
        "});}"
        "setInterval(update,100);"
        "</script></body></html>";
    
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t telemetry_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, telemetry_buffer, strlen(telemetry_buffer));
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    
    httpd_handle_t srv = NULL;
    
    if (httpd_start(&srv, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(srv, &root);
        
        httpd_uri_t telemetry = {
            .uri       = "/telemetry",
            .method    = HTTP_GET,
            .handler   = telemetry_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(srv, &telemetry);
        
        ESP_LOGI(TAG, "Web server started. Access at http://192.168.4.1");
        return srv;
    }
    
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

/* Update telemetry for both serial and WiFi */
static inline void update_telemetry(int s0, int s1, int s2, int s3, int s4, int s5, int s6, int s7) {
    snprintf(telemetry_buffer, sizeof(telemetry_buffer), 
             "%d %d %d %d %d %d %d %d", s0, s1, s2, s3, s4, s5, s6, s7);
    ESP_LOGI(TAG, "%s", telemetry_buffer);
    telemetry_updated = true;
}

/* ========== MAIN ========== */
void app_main(void) {
    ESP_LOGI(TAG, "Boot LFR (Adaptive PD + STOP + GAP + WiFi)");

    /* Initialize NVS for WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize WiFi AP */
    wifi_init_softap();
    
    /* Start HTTP Server */
    server = start_webserver();

    /* GPIO OUTPUT */
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

    /* BUTTON */
    gpio_config_t in_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&in_cfg);

    /* DIPS */
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

    /* MOTOR PWM */
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
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config_t chB = chA;
    chB.gpio_num = PIN_MB1;
    chB.channel  = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&chA));
    ESP_ERROR_CHECK(ledc_channel_config(&chB));

    int dip1 = gpio_get_level(PIN_DIP_1);
    int dip2 = gpio_get_level(PIN_DIP_2);
    int BASE_SPEED, KP;
    int stop_consec_in;

    if (dip1 == 1 && dip2 == 1) {
        BASE_SPEED = MODE_SLOW_BASE_SPEED;   KP = MODE_SLOW_KP;
        stop_consec_in = STOP_CONSEC_IN_SLOW;
    } else if (dip1 == 0 && dip2 == 1) {
        BASE_SPEED = MODE_NORMAL_BASE_SPEED; KP = MODE_NORMAL_KP;
        stop_consec_in = STOP_CONSEC_IN_NORMAL;
    } else if (dip1 == 1 && dip2 == 0) {
        BASE_SPEED = MODE_FAST_BASE_SPEED;   KP = MODE_FAST_KP;
        stop_consec_in = STOP_CONSEC_IN_FAST;
    } else {
        BASE_SPEED = MODE_TURBO_BASE_SPEED;  KP = MODE_TURBO_KP;
        stop_consec_in = STOP_CONSEC_IN_TURBO;
    }

    bool running = false;
    bool last_btn = true;
    int weights[8] = { -5, -3, -1, 0, 0, 1, 3, 5 };
    int last_turn_dir = 1;
    int last_err = 0;

    /* STOP STATE */
    stop_state_t st = ST_FOLLOW;
    int stop_in_ctr = 0, stop_out_ctr = 0, cooldown = 0;

    /* GAP RECOVERY STATE */
    gap_state_t gap = GAP_NONE;
    int gap_ctr = 0, gap_sweep_ctr = 0;

    uint32_t tick = 0;

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

        if (!running) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }

        /* SENSOR READOUT */
        int raw[8] = {0}, blk[8] = {0};
        int telem_white[8] = {0};     
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

            telem_white[i] = is_black ? 0 : 1;         
        }

        /* STOP STATE MACHINE */
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
                int Lc = STOP_CREEP_SPEED, Rc = STOP_CREEP_SPEED;
                if (INVERT_BOTH_MOTORS) { Lc = -Lc; Rc = -Rc; }
                motor_set(LEDC_CHANNEL_0, PIN_MA2, Lc);
                motor_set(LEDC_CHANNEL_1, PIN_MB2, Rc);

                if (black_count_raw < 4) {
                    if (++stop_out_ctr >= STOP_CONSEC_OUT) {
                        st = ST_COOLDOWN;
                        cooldown = STOP_COOLDOWN_TICKS;
                    }
                } else stop_out_ctr = 0;

                vTaskDelay(pdMS_TO_TICKS(5));
                if (++tick % 10 == 0) {
                    update_telemetry(telem_white[0], telem_white[1], telem_white[2], telem_white[3],
                                     telem_white[4], telem_white[5], telem_white[6], telem_white[7]);
                }
                continue; 
            }

            case ST_COOLDOWN:
                break;

            case ST_BRAKE:
                break;
        }

        /* NORMAL CONTROL (PD + GAP) */
        int L = 0, R = 0;

        /* GAP HANDLING */
        int act = 0, min_i = 8, max_i = -1;
        for (int i = 0; i < 8; i++) {
            if (blk[i]) { act++; if (i < min_i) min_i = i; if (i > max_i) max_i = i; }
        }
        int spread = (act > 0) ? (max_i - min_i) : 0;

        if (act == 0) {
            if (gap == GAP_NONE) { gap = GAP_COAST; gap_ctr = GAP_COAST_TICKS; }
        } else {
            gap = GAP_NONE; gap_ctr = 0; gap_sweep_ctr = 0;
        }

        if (gap == GAP_COAST) {
            int fwd = GAP_COAST_SPEED;
            if (INVERT_BOTH_MOTORS) fwd = -fwd;
            motor_set(LEDC_CHANNEL_0, PIN_MA2, fwd);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, fwd);

            if (--gap_ctr <= 0) { gap = GAP_SWEEP; gap_sweep_ctr = GAP_SWEEP_MAX_TICKS; }
            vTaskDelay(pdMS_TO_TICKS(5));
            if (++tick % 10 == 0) {
                update_telemetry(telem_white[0], telem_white[1], telem_white[2], telem_white[3],
                                 telem_white[4], telem_white[5], telem_white[6], telem_white[7]);
            }
            continue;
        } else if (gap == GAP_SWEEP) {
            int s = GAP_SWEEP_SPEED;
            int Ls =  s * last_turn_dir;
            int Rs = -s * last_turn_dir;
            if (INVERT_BOTH_MOTORS) { Ls = -Ls; Rs = -Rs; }
            motor_set(LEDC_CHANNEL_0, PIN_MA2, Ls);
            motor_set(LEDC_CHANNEL_1, PIN_MB2, Rs);

            if (act > 0 || --gap_sweep_ctr <= 0) { gap = GAP_NONE; }
            vTaskDelay(pdMS_TO_TICKS(5));
            if (++tick % 10 == 0) {
                update_telemetry(telem_white[0], telem_white[1], telem_white[2], telem_white[3],
                                 telem_white[4], telem_white[5], telem_white[6], telem_white[7]);
            }
            continue;
        }

        /* SENSOR PD CONTROLLER */
        long err_sum = 0;
        for (int i = 0; i < 8; i++) if (blk[i]) err_sum += weights[i];
        int err = (int)err_sum;                 
        int kd_base = KP / KD_DIVISOR;

        bool sharp = (blk[0] || blk[7] || spread >= 3); 
        int kp_use = KP;
        int kd_use = kd_base;
        int base_use = BASE_SPEED;

        if (sharp) {
            kp_use = (KP * KP_BOOST_NUM) / KP_BOOST_DEN;
            kd_use = (kd_base * KD_BOOST_NUM) / KD_BOOST_DEN;
            base_use = (BASE_SPEED * SPEED_DROP_NUM) / SPEED_DROP_DEN; 
        }

        // If only extreme sensors triggered, use push/inner clamp to snap in
        if ((blk[0] || blk[1]) && !(blk[6] || blk[7])) {
            L = CURVE_INNER_SPEED; R = CURVE_PUSH_SPEED; last_turn_dir = -1;
        } else if ((blk[6] || blk[7]) && !(blk[0] || blk[1])) {
            L = CURVE_PUSH_SPEED; R = CURVE_INNER_SPEED; last_turn_dir = 1;
        } else if (act > 0) {
            int d = err - last_err;
            long turn = (long)kp_use * err + (long)kd_use * d;
            L = base_use - (int)turn;
            R = base_use + (int)turn;
            last_turn_dir = (err >= 0) ? 1 : -1;
        } else {
            // Should be handled by GAP, but keep a tiny spin as a safety
            L = 320 * last_turn_dir;
            R = -320 * last_turn_dir;
        }
        last_err = err;

        /* CLAMP */
        if (L > PWM_MAX_DUTY) L = PWM_MAX_DUTY;
        if (L < -PWM_MAX_DUTY) L = -PWM_MAX_DUTY;
        if (R > PWM_MAX_DUTY) R = PWM_MAX_DUTY;
        if (R < -PWM_MAX_DUTY) R = -PWM_MAX_DUTY;

        /* STOP  */
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

        /* TELEMETRY */
        if (++tick % 10 == 0) {
            update_telemetry(telem_white[0], telem_white[1], telem_white[2], telem_white[3],
                             telem_white[4], telem_white[5], telem_white[6], telem_white[7]);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
