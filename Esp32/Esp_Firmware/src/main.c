#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "initializers.h"
    
    int64_t global_last_command_us = 0;

    //declaring locks
    SemaphoreHandle_t xSemaphore_getSpeed;
    SemaphoreHandle_t xSemaphore_getRosSpeed;
    EventGroupHandle_t initialization_groupEvent;

    const int task0_init_done = 0b01;
    const int task1_init_done = 0b10;

    //declaring and initializing global variables
    float global_ros_angular_speed_left = 0;
    float global_ros_angular_speed_right = 0;

    float global_motor_angular_speed_left = 0 ;
    float global_motor_angular_speed_right = 0;

    double global_total_x = 0;
    double global_total_y = 0;
    double global_total_theta = 0;//PI/2;

    float global_servo_angle = (float) SERVO_INITIAL_ANGLE;
    float global_ros_servo_angle = (float) SERVO_INITIAL_ANGLE;

    uint32_t global_timer_miliseconds = 0;
    uint32_t global_time_stamp_miliseconds = 0;

    // tick (FreeRTOS) do ultimo frame valido recebido do ROS (usado no failsafe)
    volatile TickType_t global_last_ros_rx_tick = 0;


void app_main(void)
{
    xSemaphore_getSpeed = xSemaphoreCreateMutex();
    xSemaphore_getRosSpeed = xSemaphoreCreateMutex();
    initialization_groupEvent = xEventGroupCreate();

    configASSERT(xSemaphore_getSpeed != NULL);
    configASSERT(xSemaphore_getRosSpeed != NULL);
    configASSERT(initialization_groupEvent != NULL);

    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        core0fuctions,
        "motor_control",
        4096,
        NULL,
        1,
        NULL,
        0
    );
    configASSERT(result == pdPASS);

    result = xTaskCreatePinnedToCore(
        core1functions,
        "uart_comm",
        4096,
        NULL,
        1,
        NULL,
        1
    );
    configASSERT(result == pdPASS);
}

