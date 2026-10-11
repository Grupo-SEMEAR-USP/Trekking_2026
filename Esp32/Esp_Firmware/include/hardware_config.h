#pragma once
/* Valores confirmados pelo usuário: 34 reto, 0 mínimo, 68 máximo;
 * aumentar o ângulo vira para a direita. LED GPIO23. */
#define SERVO_CENTER_DEG 34.0f
#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 68.0f
#define SERVO_DIRECTION_SIGN (-1.0f)
/* Ganho inicial: calibrar ângulo real das rodas versus eixo do servo. */
#define SERVO_DEG_PER_WHEEL_RAD 57.295779513f
#define MAX_STEERING_RAD 0.35f
#define MAX_WHEEL_RAD_S 10.0f
#define LED_GPIO 23
#define LED_ACTIVE_LEVEL 1
#define COMMAND_TIMEOUT_US 250000LL
#define ENCODER_LEFT_SIGN 1
#define ENCODER_RIGHT_SIGN 1
#define ENCODER_FILTER_US 1
