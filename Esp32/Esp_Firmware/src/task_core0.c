#include "task_core0.h"
#include "types.h"
#include "initializers.h"
#include "sycronization.h"
#include "global_variables.h"
#include "hardware_config.h"
#include "pwm.h"
#include "pid.h"
#include "esp_timer.h"

static void reset_pid(pid_handle_v2 *handle) {
    pid_ctrl_block_handle_t p = handle->pid_handle;
    p->integral_err = p->previous_err1 = p->previous_err2 = p->last_output = 0;
    p->prev_ts = esp_timer_get_time();
}

void core0fuctions(void *params) {
    (void)params;
    pwm_motors_init();
    rotary_encoder_t *left = NULL, *right = NULL;
    rotary_encoder_config_t lc = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)0, PCNT_CHA_LEFT, PCNT_CHB_LEFT);
    rotary_encoder_config_t rc = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)1, PCNT_CHA_RIGHT, PCNT_CHB_RIGHT);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&lc, &left));
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&rc, &right));
    ESP_ERROR_CHECK(left->set_glitch_filter(left, ENCODER_FILTER_US));
    ESP_ERROR_CHECK(right->set_glitch_filter(right, ENCODER_FILTER_US));
    ESP_ERROR_CHECK(left->start(left));
    ESP_ERROR_CHECK(right->start(right));
    pid_handle_v2 lp = {0}, rp = {0};
    pid_init(&lp, &rp);
    xEventGroupSetBits(initialization_groupEvent, task0_init_done);
    /* Não espera a UART para executar a parada: controle é independente. */
    int last_l = left->get_counter_value(left), last_r = right->get_counter_value(right);
    int64_t last_sample = esp_timer_get_time(), last_command = 0;
    float target_l = 0, target_r = 0, steering = 0;
    uint8_t led = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        int64_t now = esp_timer_get_time();
        if (xSemaphoreTake(xSemaphore_getRosSpeed, 0) == pdTRUE) {
            target_l = global_ros_angular_speed_left;
            target_r = global_ros_angular_speed_right;
            steering = global_ros_servo_angle;
            last_command = global_last_command_us;
            led = led_byte;
            xSemaphoreGive(xSemaphore_getRosSpeed);
        }
        bool fresh = last_command > 0 && now - last_command <= COMMAND_TIMEOUT_US;
        bool moving = fresh && (fabsf(target_l) > 0.001f || fabsf(target_r) > 0.001f);
        if (!fresh) { target_l = target_r = steering = 0; led = 0; }
        gpio_set_level(LED_GPIO, led ? LED_ACTIVE_LEVEL : !LED_ACTIVE_LEVEL);
        gpio_set_level(STAND_BY, moving ? 1 : 0);
        if (!moving) {
            pwm_actuate(ESQ, 0); pwm_actuate(DIR, 0);
            reset_pid(&lp); reset_pid(&rp);
        }
        float servo_deg = SERVO_CENTER_DEG + SERVO_DIRECTION_SIGN * SERVO_DEG_PER_WHEEL_RAD * steering;
        servo_deg = fmaxf(SERVO_MIN_DEG, fminf(SERVO_MAX_DEG, servo_deg));
        ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_PWM_CHANNEL, servo_deg));

        if (now - last_sample >= 40000) {
            int curr_l = left->get_counter_value(left), curr_r = right->get_counter_value(right);
            int32_t ticks_l = (int32_t)((uint32_t)curr_l - (uint32_t)last_l) * ENCODER_LEFT_SIGN;
            int32_t ticks_r = (int32_t)((uint32_t)curr_r - (uint32_t)last_r) * ENCODER_RIGHT_SIGN;
            float dt = (now - last_sample) / 1000000.0f;
            last_sample = now; last_l = curr_l; last_r = curr_r;
            float speed_l = ticks_l * 2.0f * PI / ENCODER_RESOLUTION_TICKS / dt;
            float speed_r = ticks_r * 2.0f * PI / ENCODER_RESOLUTION_TICKS / dt;
            double dl = ticks_l * ENCODER_DISPLACEMENT, dr = ticks_r * ENCODER_DISPLACEMENT;
            double dtheta = (dr-dl) / WHELL_REAR_SEPARATION;
            xSemaphoreTake(xSemaphore_getSpeed, portMAX_DELAY);
            double heading_mid = global_total_theta + dtheta / 2;
            global_total_x += (dr+dl) / 2 * cos(heading_mid);
            global_total_y += (dr+dl) / 2 * sin(heading_mid);
            global_total_theta = atan2(sin(global_total_theta+dtheta), cos(global_total_theta+dtheta));
            global_motor_angular_speed_left = speed_l;
            global_motor_angular_speed_right = speed_r;
            global_time_stamp_miliseconds = (uint32_t)(now / 1000);
            xSemaphoreGive(xSemaphore_getSpeed);
            if (moving) {
                float duty_l = 0, duty_r = 0;
                if (fabsf(target_l) > 0.001f) ESP_ERROR_CHECK(lp.pid_calculate(&lp, speed_l, target_l, &duty_l));
                else reset_pid(&lp);
                if (fabsf(target_r) > 0.001f) ESP_ERROR_CHECK(rp.pid_calculate(&rp, speed_r, target_r, &duty_r));
                else reset_pid(&rp);
                pwm_actuate(ESQ, duty_l); pwm_actuate(DIR, duty_r);
            }
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}
