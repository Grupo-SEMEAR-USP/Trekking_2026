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

void app_main(void)
{
    xSemaphore_getSpeed = xSemaphoreCreateMutex();
    xSemaphore_getRosSpeed = xSemaphoreCreateMutex();
    initialization_groupEvent = xEventGroupCreate();

    if (xSemaphore_getSpeed == NULL || xSemaphore_getRosSpeed == NULL ||
        initialization_groupEvent == NULL) {
        ESP_LOGE("MAIN", "Falha ao criar sincronizacao");
        abort();
    }

    if (xTaskCreatePinnedToCore(core0fuctions, "motor_control", 8192, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE("MAIN", "Falha ao criar tarefa dos motores");
        abort();
    }

    if (xTaskCreatePinnedToCore(core1functions, "uart", 4096, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Falha ao criar tarefa UART");
        abort();
    }
}
