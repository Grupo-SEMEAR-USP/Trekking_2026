#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include "esp_err.h"
#include "types.h"

esp_err_t uart_init();
esp_err_t uart_send_frame(data_to_send_t *cmd, size_t size);
void uart_send_motors(double *total_x_displacement, double *total_y_displacement, double *total_angular_displacement);
void uart_read_motors();
void uart_send_led(bool led_state);
void uart_read_led();

#endif