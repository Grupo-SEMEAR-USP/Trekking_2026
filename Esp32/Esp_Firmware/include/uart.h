#pragma once
#include "esp_err.h"
esp_err_t uart_init(void);
void uart_read_motors(void);
void uart_send_motors(void);
