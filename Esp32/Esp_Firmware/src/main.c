#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "hardware_config.h"
#include "esp_log.h"

SemaphoreHandle_t xSemaphore_getSpeed;
SemaphoreHandle_t xSemaphore_getRosSpeed;
EventGroupHandle_t initialization_groupEvent;
const int task0_init_done = 1;
const int task1_init_done = 2;
float global_ros_angular_speed_left = 0, global_ros_angular_speed_right = 0;
float global_motor_angular_speed_left = 0, global_motor_angular_speed_right = 0;
float global_ros_servo_angle = 0;
double global_total_x = 0, global_total_y = 0, global_total_theta = 0;
uint32_t global_timer_miliseconds = 0, global_time_stamp_miliseconds = 0;
int64_t global_last_command_us = 0;
uint8_t led_byte = 0;

void app_main(void) {
    /* UART0 é protocolo binário; não misturar logs de aplicação aos frames. */
    esp_log_level_set("*", ESP_LOG_NONE);
    xSemaphore_getSpeed = xSemaphoreCreateMutex();
    xSemaphore_getRosSpeed = xSemaphoreCreateMutex();
    initialization_groupEvent = xEventGroupCreate();
    configASSERT(xSemaphore_getSpeed && xSemaphore_getRosSpeed && initialization_groupEvent);
    BaseType_t ok = xTaskCreatePinnedToCore(core0fuctions, "control", 4096, NULL, 5, NULL, 0);
    configASSERT(ok == pdPASS);
    ok = xTaskCreatePinnedToCore(core1functions, "uart", 4096, NULL, 4, NULL, 1);
    configASSERT(ok == pdPASS);
}
