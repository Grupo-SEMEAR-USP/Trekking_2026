#include "types.h"
#include "task_core0.h"
#include "initializers.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "pid.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/semphr.h"

#define COMMAND_TIMEOUT_US 500000LL

rotary_encoder_t *encoder_left = NULL;
rotary_encoder_t *encoder_right = NULL;

pid_handle_v2 pid_handle_left;
pid_handle_v2 pid_handle_right;

static float pid_result_duty_left = 0.0f;
static float pid_result_duty_right = 0.0f;

static float local_ros_angular_speed_left = 0.0f;
static float local_ros_angular_speed_right = 0.0f;
static float local_servo_angle = SERVO_INITIAL_ANGLE;

static int64_t local_last_command_us = 0;
static int64_t last_encoder_time_us = 0;

static int count_get_real = 0;

void monitor_encoder_pid_calc(void);

static void reset_pid_state(pid_handle_v2 *handle)
{
    handle->pid_handle->integral_err = 0.0f;
    handle->pid_handle->previous_err1 = 0.0f;
    handle->pid_handle->previous_err2 = 0.0f;
    handle->pid_handle->last_output = 0.0f;
}

void core0fuctions(void *params)
{
    (void)params;

    pwm_motors_init();

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

    ESP_ERROR_CHECK(
        encoder_left->set_glitch_filter(encoder_left, 10)
    );
    ESP_ERROR_CHECK(
        encoder_right->set_glitch_filter(encoder_right, 10)
    );

    ESP_ERROR_CHECK(encoder_left->start(encoder_left));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    pid_init(&pid_handle_left, &pid_handle_right);

    configASSERT(pid_handle_left.pid_handle != NULL);
    configASSERT(pid_handle_right.pid_handle != NULL);

    pwm_actuate(ESQ, 0);
    pwm_actuate(DIR, 0);

    xEventGroupSetBits(
        initialization_groupEvent,
        task0_init_done
    );

    xEventGroupWaitBits(
        initialization_groupEvent,
        task1_init_done,
        pdTRUE,
        pdTRUE,
        portMAX_DELAY
    );

    encoder_left->reset_counter_value(encoder_left);
    encoder_right->reset_counter_value(encoder_right);

    last_encoder_time_us = esp_timer_get_time();

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        monitor_encoder_pid_calc();

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(PID_DELAY)
        );
    }
}

void monitor_encoder_pid_calc(void)
{
    // Copia os comandos e o horário de recepção sob o mesmo mutex.
    if (xSemaphoreTake(xSemaphore_getRosSpeed, 0) == pdTRUE) {
        local_ros_angular_speed_left =
            global_ros_angular_speed_left;

        local_ros_angular_speed_right =
            global_ros_angular_speed_right;

        local_servo_angle = global_ros_servo_angle;
        local_last_command_us = global_last_command_us;

        xSemaphoreGive(xSemaphore_getRosSpeed);
    }

    int64_t now_us = esp_timer_get_time();

    bool command_expired =
        local_last_command_us == 0 ||
        now_us - local_last_command_us >= COMMAND_TIMEOUT_US;

    bool command_invalid =
        !isfinite(local_ros_angular_speed_left) ||
        !isfinite(local_ros_angular_speed_right) ||
        !isfinite(local_servo_angle);

    bool stop_motors = command_expired || command_invalid;

    // Para imediatamente, mesmo fora do ciclo de cálculo do PID.
    if (stop_motors) {
        local_ros_angular_speed_left = 0.0f;
        local_ros_angular_speed_right = 0.0f;

        pid_result_duty_left = 0.0f;
        pid_result_duty_right = 0.0f;

        reset_pid_state(&pid_handle_left);
        reset_pid_state(&pid_handle_right);

        pwm_actuate(ESQ, 0);
        pwm_actuate(DIR, 0);
    }

    // Mantém a última posição do servo quando o comando expira.
    if (!stop_motors) {
        ESP_ERROR_CHECK(
            iot_servo_write_angle(
                LEDC_LOW_SPEED_MODE,
                SERVO_PWM_CHANNEL,
                local_servo_angle
            )
        );
    }

    count_get_real++;
    global_timer_miliseconds += PID_DELAY;

    if (count_get_real < ENCODER_COUNTER_WAIT_PID_OP) {
        return;
    }

    if (xSemaphoreTake(xSemaphore_getSpeed, 0) != pdTRUE) {
        return;
    }

    count_get_real = 0;

    int ticks_left =
        encoder_left->get_counter_value(encoder_left);

    int ticks_right =
        encoder_right->get_counter_value(encoder_right);

    encoder_left->reset_counter_value(encoder_left);
    encoder_right->reset_counter_value(encoder_right);

    int64_t encoder_time_us = esp_timer_get_time();
    int64_t delta_time_us =
        encoder_time_us - last_encoder_time_us;

    last_encoder_time_us = encoder_time_us;

    float speed_left = 0.0f;
    float speed_right = 0.0f;

    if (delta_time_us > 0) {
        float delta_time_s = (float)delta_time_us / 1000000.0f;

        speed_left =
            ((float)ticks_left * 2.0f * PI) /
            ((float)ENCODER_RESOLUTION_TICKS * delta_time_s);

        speed_right =
            ((float)ticks_right * 2.0f * PI) /
            ((float)ENCODER_RESOLUTION_TICKS * delta_time_s);
    }

    global_motor_angular_speed_left = speed_left;
    global_motor_angular_speed_right = speed_right;

    // Mantém as equações de odometria do projeto original.
    global_total_x +=
        ((double)ticks_right + (double)ticks_left) *
        ENCODER_DISPLACEMENT * 0.5 *
        cos(global_total_theta);

    global_total_y +=
        ((double)ticks_right + (double)ticks_left) *
        ENCODER_DISPLACEMENT * 0.5 *
        sin(global_total_theta);

    global_total_theta +=
        (((double)ticks_right - (double)ticks_left) *
         0.5 * ENCODER_DISPLACEMENT) /
        WHELL_REAR_SEPARATION;

    global_time_stamp_miliseconds =
        global_timer_miliseconds;

    if (!stop_motors) {
        if (fabsf(local_ros_angular_speed_left) < 1e-3f) {
            pid_handle_left.pid_handle->integral_err = 0.0f;
        }

        if (fabsf(local_ros_angular_speed_right) < 1e-3f) {
            pid_handle_right.pid_handle->integral_err = 0.0f;
        }

        ESP_ERROR_CHECK(
            pid_handle_left.pid_calculate(
                &pid_handle_left,
                speed_left,
                local_ros_angular_speed_left,
                &pid_result_duty_left
            )
        );

        ESP_ERROR_CHECK(
            pid_handle_right.pid_calculate(
                &pid_handle_right,
                speed_right,
                local_ros_angular_speed_right,
                &pid_result_duty_right
            )
        );

        pwm_actuate(ESQ, pid_result_duty_left);
        pwm_actuate(DIR, pid_result_duty_right);
    }

    xSemaphoreGive(xSemaphore_getSpeed);
}
