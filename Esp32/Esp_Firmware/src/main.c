#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "initializers.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include "esp_err.h"

// Declaração dos semáforos
SemaphoreHandle_t xSemaphore_getSpeed;
SemaphoreHandle_t xSemaphore_getRosSpeed;
SemaphoreHandle_t xSemaphore_getLed;
EventGroupHandle_t initialization_groupEvent;

const int task0_init_done = 0b01;
const int task1_init_done = 0b10;

// Variáveis globais
float global_ros_angular_speed_left = 0;
float global_ros_angular_speed_right = 0;

float global_motor_angular_speed_left = 0;
float global_motor_angular_speed_right = 0;

bool global_led_state = 0;

double global_total_x = 0;
double global_total_y = 0;
double global_total_theta = 0;

float global_servo_angle = (float)SERVO_INITIAL_ANGLE;
float global_ros_servo_angle = (float)SERVO_INITIAL_ANGLE;
uint8_t led_byte = 0;
uint32_t global_ros_last_rx_ms = 0;

uint32_t global_timer_miliseconds = 0;
uint32_t global_time_stamp_miliseconds = 0;

void app_main(void) {
    gpio_set_direction(STAND_BY, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(STAND_BY, 1);

    // 1. Configura a GPIO 32 do LED como saída
    led_init();

    // 2. Inicializa o PWM (motores + iot_servo)
    pwm_motors_init();

    // Manda o servo para 30 graus no início
    while (1) {
        
        pwm_servo_actuate(34.0f); //offset de 34 graus para compensar a barra comum
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        pwm_actuate(ESQ, 8000);
        pwm_actuate(DIR, 8000);
        vTaskDelay(pdMS_TO_TICKS(5000));

        pwm_actuate(ESQ, 0);
        pwm_actuate(DIR, 0);

        // Liga o LED após os motores pararem
        gpio_set_level(GPIO_NUM_32, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Desliga o LED
        gpio_set_level(GPIO_NUM_32, 0);
        vTaskDelay(pdMS_TO_TICKS(200000000000));
    }
}