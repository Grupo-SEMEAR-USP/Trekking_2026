```c
#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "initializers.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

uint32_t global_timer_miliseconds = 0;
uint32_t global_time_stamp_miliseconds = 0;

void app_main(void)
{
    // Inicializa PWM dos motores e do servo
    pwm_motors_init();

    // Inicialização do encoder esquerdo
    rotary_encoder_t *encoder_left = NULL;

    rotary_encoder_config_t config_encoder_left =
        ROTARY_ENCODER_DEFAULT_CONFIG(
            (rotary_encoder_dev_t)0,
            PCNT_CHA_LEFT,
            PCNT_CHB_LEFT
        );

    ESP_ERROR_CHECK(
        rotary_encoder_new_ec11(
            &config_encoder_left,
            &encoder_left
        )
    );

    // Inicialização do encoder direito
    rotary_encoder_t *encoder_right = NULL;

    rotary_encoder_config_t config_encoder_right =
        ROTARY_ENCODER_DEFAULT_CONFIG(
            (rotary_encoder_dev_t)1,
            PCNT_CHA_RIGHT,
            PCNT_CHB_RIGHT
        );

    ESP_ERROR_CHECK(
        rotary_encoder_new_ec11(
            &config_encoder_right,
            &encoder_right
        )
    );

    // Inicia os encoders
    ESP_ERROR_CHECK(encoder_left->start(encoder_left));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    // Mantém os motores inicialmente parados
    pwm_actuate(ESQ, 0);
    pwm_actuate(DIR, 0);

    while (1)
    {
        // Servo na posição de 45 graus
        iot_servo_write_angle(
            LEDC_LOW_SPEED_MODE,
            SERVO_PWM_CHANNEL,
            -30.0f + SERVO_OFFSET
        );

        // Aciona os dois motores
        pwm_actuate(ESQ, 512);
        pwm_actuate(DIR, 512);

        vTaskDelay(pdMS_TO_TICKS(3000));

        // Para os motores
        pwm_actuate(ESQ, 0);
        pwm_actuate(DIR, 0);

        // Servo na posição de 135 graus
        iot_servo_write_angle(
            LEDC_LOW_SPEED_MODE,
            SERVO_PWM_CHANNEL,
            30.0f + SERVO_OFFSET
        );

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```
