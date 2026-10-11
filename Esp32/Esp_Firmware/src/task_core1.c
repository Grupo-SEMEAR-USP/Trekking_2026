#include "types.h"
#include "task_core1.h"
#include "sycronization.h"
#include "uart.h"

void core1functions(void *params) {
    (void)params;
    xEventGroupWaitBits(initialization_groupEvent, task0_init_done, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_ERROR_CHECK(uart_init());
    xEventGroupSetBits(initialization_groupEvent, task1_init_done);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        uart_read_motors();
        uart_send_motors();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(20));
    }
}
