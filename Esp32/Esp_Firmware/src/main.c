#include "types.h"
#include "pwm.h"
#include "uart.h"
#include "global_variables.h"
#include "sycronization.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <stdlib.h>

/* Teste UART -> PWM em malha aberta, sem PID, encoders ou odometria.
 * O protocolo continua sendo: float left, float right, float servo, byte LED.
 * left/right chegam em rad/s; servo chega em radianos relativos ao centro.
 * Este ganho e apenas para bancada: nao garante a velocidade fisica pedida.
 */
#define TEST_PWM_PER_RAD_S 400.0f
#define TEST_MAX_PWM 4000.0f
#define TEST_UART_TIMEOUT_MS 500U
#define TEST_MAX_STEERING_RAD 0.60f
// Referencia do teste anterior. Ajustar ao centro mecanico real da direcao.
#define TEST_SERVO_CENTER_DEG 32.0f
#define TEST_SERVO_SIGN 1.0f

// uart.c atualiza estas variaveis somente apos validar o pacote completo.
float global_ros_angular_speed_left = 0.0f;
float global_ros_angular_speed_right = 0.0f;
float global_ros_servo_angle = 0.0f;
uint8_t led_byte = 0;
uint32_t global_ros_last_rx_ms = 0;
SemaphoreHandle_t xSemaphore_getRosSpeed;

static float clamp_value(float value, float minimum, float maximum)
{
    return fminf(maximum, fmaxf(minimum, value));
}

static float speed_to_pwm(float speed_rad_s)
{
    return clamp_value(speed_rad_s * TEST_PWM_PER_RAD_S,
                       -TEST_MAX_PWM, TEST_MAX_PWM);
}

void app_main(void)
{
    // Mantem a ponte desabilitada enquanto configura os PWM e a UART.
    ESP_ERROR_CHECK(gpio_set_direction(STAND_BY, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(STAND_BY, 0));
    pwm_motors_init();
    pwm_actuate(ESQ, 0.0f);
    pwm_actuate(DIR, 0.0f);
    ESP_ERROR_CHECK(iot_servo_write_angle(
        LEDC_LOW_SPEED_MODE, SERVO_PWM_CHANNEL, TEST_SERVO_CENTER_DEG));

    xSemaphore_getRosSpeed = xSemaphoreCreateMutex();
    if (xSemaphore_getRosSpeed == NULL) {
        abort();
    }
    ESP_ERROR_CHECK(uart_init());

    while (1) {
        // Usa o parser existente, incluindo checksum, LED e timestamp.
        // Esta leitura espera no maximo 20 ms por bytes.
        uart_read_motors();

        xSemaphoreTake(xSemaphore_getRosSpeed, portMAX_DELAY);
        float left = global_ros_angular_speed_left;
        float right = global_ros_angular_speed_right;
        float steering_rad = global_ros_servo_angle;
        uint32_t last_rx_ms = global_ros_last_rx_ms;
        xSemaphoreGive(xSemaphore_getRosSpeed);

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool valid = last_rx_ms != 0 &&
                     (uint32_t)(now_ms - last_rx_ms) <= TEST_UART_TIMEOUT_MS &&
                     isfinite(left) && isfinite(right) && isfinite(steering_rad);

        // O uart_comm.py envia 90 antes do primeiro /velocity_command.
        // Isso nao e um angulo valido em radianos: aguarda um comando real.
        valid = valid && fabsf(steering_rad) <= TEST_MAX_STEERING_RAD + 0.001f;

        if (!valid) {
            ESP_ERROR_CHECK(gpio_set_level(STAND_BY, 0));
            pwm_actuate(ESQ, 0.0f);
            pwm_actuate(DIR, 0.0f);
        } else {
            float pwm_left = speed_to_pwm(left);
            float pwm_right = speed_to_pwm(right);
            float servo_deg = TEST_SERVO_CENTER_DEG +
                              TEST_SERVO_SIGN * steering_rad * (180.0f / PI);
            servo_deg = clamp_value(servo_deg, 0.0f, 180.0f);

            ESP_ERROR_CHECK(iot_servo_write_angle(
                LEDC_LOW_SPEED_MODE, SERVO_PWM_CHANNEL, servo_deg));
            pwm_actuate(ESQ, pwm_left);
            pwm_actuate(DIR, pwm_right);
            ESP_ERROR_CHECK(gpio_set_level(
                STAND_BY, pwm_left != 0.0f || pwm_right != 0.0f));
        }

        // Nao envia odometria ficticia nem logs na UART do protocolo binario.
        // O timeout acima detecta perda de UART. O timeout de /cmd_vel
        // continua sendo responsabilidade do twist2ackermann.py, pois
        // uart_comm.py retransmite o ultimo comando mesmo sem dados novos.
        vTaskDelay(pdMS_TO_TICKS(UART_DELAY));
    }
}
