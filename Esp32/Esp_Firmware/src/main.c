#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "initializers.h"

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


void app_main() {

    // Driver de motor (TB6612): STBY precisa ficar em nivel ALTO, senao os motores NAO giram
    gpio_reset_pin(STAND_BY);
    gpio_set_direction(STAND_BY, GPIO_MODE_OUTPUT);
    gpio_set_level(STAND_BY, 1);

    //initializing locks
    xSemaphore_getSpeed = xSemaphoreCreateMutex(); 
    xSemaphore_getRosSpeed = xSemaphoreCreateMutex();

    initialization_groupEvent = xEventGroupCreate();

    //Inicializar as tasks
    // core 0: pwm, encoders, pid e servo | core 1: comunicacao UART com o ROS
    xTaskCreatePinnedToCore(&core0fuctions, "task que inicializa pwm,encoders e pid no core 0", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(&core1functions, "task que inicializa a uart no core 1", 8192, NULL, 1, NULL, 1);

    /*
     * Firmware de TESTE de bancada (motores girando direto, sem ROS) que estava aqui antes:
     *
     *   pwm_motors_init(); ... encoders ...
     *   while(1){ vTaskDelay(pdMS_TO_TICKS(100)); pwm_actuate(ESQ,1023); pwm_actuate(DIR,1023); }
     *
     * Ele ignorava completamente os comandos vindos do ROS pela UART.
     */
}

