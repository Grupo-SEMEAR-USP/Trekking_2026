#pragma once

void pwm_motors_init();
void pwm_actuate(int channel, float duty);
void pwm_servo_actuate(float angle);
void led_init(void);