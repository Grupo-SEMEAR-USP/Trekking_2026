#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include "esp_err.h"
#include "types.h"

esp_err_t uart_init();
esp_err_t uart_send_frame(data_to_send_t *cmd, size_t size);
void uart_send(double *total_x_displacement, double *total_y_displacement, double *total_angular_displacement);
void uart_read();

#endif