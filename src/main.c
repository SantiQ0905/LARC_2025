#include <stdio.h>
#include <string.h>
#include <math.h>
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"

#define NUM_SENSORS     16
#define ADC_MAX         4095
#define BASE_SPEED      650
#define TURN_GAIN       4.5f
#define KD_BASE         35.0f
#define FILTER_ALPHA    0.7f

#define STOP_THRESHOLD  7  // detener si detecta 7-8 sensores activos

// Pines de motores
#define MOTOR_LEFT_PWM   17
#define MOTOR_RIGHT_PWM  18
#define MOTOR_LEFT_DIR   19
#define MOTOR_RIGHT_DIR  20

// Configuración PID
typedef struct {
    float kp, kd;
    float prev_error;
    float filtered_error;
} PID_t;

static PID_t pid = {TURN_GAIN, KD_BASE, 0, 0};
static int sensor_values[NUM_SENSORS];
static int sensor_positions[NUM_SENSORS];
static int last_turn_dir = 1;  // 1 = derecha, -1 = izquierda

// --- Funciones simuladas o dependientes del hardware ---

void read_sensors(int *values, int *act_count, int *sum_mid)
{
    *act_count = 0;
    *sum_mid = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        values[i] = adc1_get_raw(i % 8);  // ejemplo
        if (values[i] > 1500) {
            (*act_count)++;
            *sum_mid += (i - (NUM_SENSORS / 2));
        }
    }
}

void set_motor_speed(int left, int right)
{
    // Saturación de PWM
    if (left > 1000) left = 1000;
    if (right > 1000) right = 1000;
    if (left < -1000) left = -1000;
    if (right < -1000) right = -1000;

    // Dirección
    int left_dir = (left >= 0);
    int right_dir = (right >= 0);

    // PWM absoluto
    left = abs(left);
    right = abs(right);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, left / 10.0f);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, right / 10.0f);

    gpio_set_level(MOTOR_LEFT_DIR, left_dir);
    gpio_set_level(MOTOR_RIGHT_DIR, right_dir);
}

// --- Control principal ---
void app_main(void)
{
    printf("Iniciando seguidor de línea optimizado...\n");

    // Inicialización de hardware
    for (int i = 0; i < NUM_SENSORS; i++)
        sensor_positions[i] = i - NUM_SENSORS / 2;

    while (1) {
        int act_mid = 0, sum_mid = 0;
        read_sensors(sensor_values, &act_mid, &sum_mid);

        int left_cmd = 0, right_cmd = 0;

        // Si detecta 7 u 8 sensores activos => STOP de seguridad
        if (act_mid >= STOP_THRESHOLD) {
            set_motor_speed(0, 0);
            esp_rom_delay_us(200000);  // pequeña pausa
            continue;
        }

        // Si no detecta línea, buscarla girando
        if (act_mid == 0) {
            left_cmd  = 350 * last_turn_dir;
            right_cmd = -350 * last_turn_dir;
            set_motor_speed(left_cmd, right_cmd);
            continue;
        }

        // --- Control PD con suavizado ---
        float error = (float)sum_mid / act_mid;

        pid.filtered_error = FILTER_ALPHA * pid.filtered_error + (1 - FILTER_ALPHA) * error;
        float derr = pid.filtered_error - pid.prev_error;
        pid.prev_error = pid.filtered_error;

        // Ajuste dinámico de KD (más fuerte en curvas)
        float kd_dynamic = pid.kd;
        if (fabs(error) > 2.5f) kd_dynamic *= 1.4f;
        else if (fabs(error) > 4.0f) kd_dynamic *= 1.8f;

        float turn = pid.kp * pid.filtered_error + kd_dynamic * derr;

        left_cmd  = BASE_SPEED - (int)turn;
        right_cmd = BASE_SPEED + (int)turn;

        // Asegura que nunca deje de buscar el centro
        last_turn_dir = (pid.filtered_error >= 0) ? 1 : -1;

        set_motor_speed(left_cmd, right_cmd);
        esp_rom_delay_us(8000);  // velocidad de control
    }
}
