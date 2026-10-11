#include "types.h"
#include "pwm.h"
#include "initializers.h"
#include "hardware_config.h"
#include "esp_rom_gpio.h"

void pwm_motors_init(void) {
    gpio_set_direction(STAND_BY, GPIO_MODE_OUTPUT);
    gpio_set_level(STAND_BY, 0);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, !LED_ACTIVE_LEVEL);
    for (int motor=ESQ;motor<=DIR;++motor) {
        ledc_timer_config_t timer=ledc_timer_config_info(timer_choose(motor));
        ledc_channel_config_t channel=ledc_channel_config_info(enable_choose(motor),channel_choose(motor),timer_choose(motor));
        ESP_ERROR_CHECK(ledc_timer_config(&timer));
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
        gpio_set_direction(motor_choose_in_primary(motor),GPIO_MODE_OUTPUT);
        gpio_set_direction(motor_choose_in_secundary(motor),GPIO_MODE_OUTPUT);
        pwm_actuate(motor,0);
    }
    servo_config_t config=servo_config_info(SERVO_PWM_TIMER,SERVO_PWM_CHANNEL,SERVO_DUTY_PIN);
    ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE,&config));
    ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE,SERVO_PWM_CHANNEL,SERVO_CENTER_DEG));
}
void pwm_actuate(int motor,float duty) {
    if (!isfinite(duty)) duty=0;
    duty=fmaxf(-MAX_DUTY,fminf(MAX_DUTY,duty));
    bool reverse=duty<0;
    if (duty==0) {
        gpio_set_level(motor_choose_in_primary(motor),0);
        gpio_set_level(motor_choose_in_secundary(motor),0);
    } else {
        gpio_set_level(motor_choose_in_primary(motor),reverse?0:1);
        gpio_set_level(motor_choose_in_secundary(motor),reverse?1:0);
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,channel_choose(motor),(uint32_t)fabsf(duty)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,channel_choose(motor)));
}
