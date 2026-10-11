#include "pwm.h"
#include "initializers.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <math.h>

void pwm_motors_init(){
    // Inicialização do motor esquerdo 
    ledc_timer_config_t timer_config = ledc_timer_config_info(timer_choose(ESQ));
    ledc_channel_config_t channel_config = ledc_channel_config_info(ENABLE_ESQ, channel_choose(ESQ), timer_choose(ESQ));
    
    ledc_timer_config(&timer_config);
    ledc_channel_config(&channel_config);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_choose(ESQ), 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_choose(ESQ));

    esp_rom_gpio_pad_select_gpio(motor_choose_in_primary(ESQ));
    gpio_set_direction(motor_choose_in_primary(ESQ), GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(motor_choose_in_secundary(ESQ));
    gpio_set_direction(motor_choose_in_secundary(ESQ), GPIO_MODE_OUTPUT);

    gpio_set_level(motor_choose_in_primary(ESQ), 0); 
    gpio_set_level(motor_choose_in_secundary(ESQ), 0);

    // Inicialização do motor direito
    timer_config = ledc_timer_config_info(timer_choose(DIR));
    channel_config = ledc_channel_config_info(ENABLE_DIR, channel_choose(DIR), timer_choose(DIR));

    ledc_timer_config(&timer_config);
    ledc_channel_config(&channel_config);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_choose(DIR), 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_choose(DIR));

    esp_rom_gpio_pad_select_gpio(motor_choose_in_primary(DIR));
    gpio_set_direction(motor_choose_in_primary(DIR), GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(motor_choose_in_secundary(DIR));
    gpio_set_direction(motor_choose_in_secundary(DIR), GPIO_MODE_OUTPUT);

    gpio_set_level(motor_choose_in_primary(DIR), 0); 
    gpio_set_level(motor_choose_in_secundary(DIR), 0);

    // Inicialização do Servo usando o iot_servo nativo do seu projeto
    servo_config_t servo_config = servo_config_info(SERVO_PWM_TIMER, SERVO_PWM_CHANNEL, SERVO_DUTY_PIN);

    ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_config));
    ESP_ERROR_CHECK(iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_PWM_CHANNEL, (float)(SERVO_INITIAL_ANGLE + SERVO_OFFSET)));
}

void pwm_actuate(int channel, float duty){
    if (!isfinite(duty)) {
        duty = 0.0f;
    }

    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < -MAX_DUTY) duty = -MAX_DUTY;

    bool clockwise = (duty < 0);

    if(clockwise){
        duty *= -1;
    }

    // Define a direção de rotação dos motores DC
    gpio_set_level(motor_choose_in_primary(channel), (clockwise) ? 0 : 1); 
    gpio_set_level(motor_choose_in_secundary(channel), (clockwise) ? 1 : 0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_choose(channel), ((uint32_t) duty));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_choose(channel));
}

void pwm_servo_actuate(float angle) {
    // Aplica limites de segurança para o ângulo
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 68.0f) angle = 68.0f;

    // Utiliza a função oficial iot_servo do projeto para mover o servo fisicamente
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_PWM_CHANNEL, angle);
}

void pwm_set_angle(float angle) {
    // Redireciona para a função principal de atuação do servo
    pwm_servo_actuate(angle);
}

void led_init(void) {
    // Configura o pino do LED (GPIO 23) como saída padrão
    gpio_set_direction(GPIO_NUM_23, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(GPIO_NUM_23, 0);
}
