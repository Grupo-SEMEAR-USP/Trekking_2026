#include "types.h"
#include "uart.h"
#include "global_variables.h"
#include "sycronization.h"
#include "hardware_config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <limits.h>
#include <stdint.h>
#include <string.h>

_Static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");
static uint8_t rx_frame[16];
static size_t rx_used = 0;
static int64_t last_byte_us = 0;

static bool accept_frame(const uint8_t *frame) {
    uint8_t checksum = 0;
    for (int i=1; i<=13; ++i) checksum += frame[i];
    if (frame[0] != FRAME_SOF || frame[15] != FRAME_EOF || frame[14] != checksum) return false;
    float l, r, angle;
    memcpy(&l, frame+1, 4); memcpy(&r, frame+5, 4); memcpy(&angle, frame+9, 4);
    if (!isfinite(l) || !isfinite(r) || !isfinite(angle) ||
        fabsf(l)>MAX_WHEEL_RAD_S+0.00001f || fabsf(r)>MAX_WHEEL_RAD_S+0.00001f ||
        fabsf(angle)>MAX_STEERING_RAD+0.00001f || frame[13]>1) {
        /* Frame válido porém comando inválido revoga imediatamente o movimento. */
        xSemaphoreTake(xSemaphore_getRosSpeed, portMAX_DELAY);
        global_last_command_us = 0;
        xSemaphoreGive(xSemaphore_getRosSpeed);
        return true;
    }
    xSemaphoreTake(xSemaphore_getRosSpeed, portMAX_DELAY);
    global_ros_angular_speed_left=l; global_ros_angular_speed_right=r;
    global_ros_servo_angle=angle; led_byte=frame[13];
    global_last_command_us=esp_timer_get_time();
    xSemaphoreGive(xSemaphore_getRosSpeed);
    return true;
}

esp_err_t uart_init(void) {
    uart_config_t config = {
        .baud_rate=115200, .data_bits=UART_DATA_8_BITS, .parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1, .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
        .source_clk=UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &config));
    return uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0);
}

void uart_read_motors(void) {
    uint8_t data[256];
    int64_t now=esp_timer_get_time();
    if (now-last_byte_us > 100000) rx_used=0;
    int n=uart_read_bytes(UART_PORT_NUM, data, sizeof(data), 0);
    for (int i=0; i<n; ++i) {
        last_byte_us=now;
        if (rx_used==0 && data[i]!=FRAME_SOF) continue;
        rx_frame[rx_used++]=data[i];
        if (rx_used==sizeof(rx_frame)) {
            if (accept_frame(rx_frame)) rx_used=0;
            else {
                memmove(rx_frame,rx_frame+1,--rx_used);
                while (rx_used && rx_frame[0]!=FRAME_SOF) memmove(rx_frame,rx_frame+1,--rx_used);
            }
        }
    }
}

static int32_t scaled_int(double value) {
    double scaled=value*1000.0;
    if (scaled>INT32_MAX) return INT32_MAX;
    if (scaled<INT32_MIN) return INT32_MIN;
    return (int32_t)llround(scaled);
}

void uart_send_motors(void) {
    int32_t x,y,theta;
    uint32_t stamp;
    xSemaphoreTake(xSemaphore_getSpeed, portMAX_DELAY);
    x=scaled_int(global_total_x); y=scaled_int(global_total_y);
    theta=scaled_int(global_total_theta); stamp=global_time_stamp_miliseconds;
    xSemaphoreGive(xSemaphore_getSpeed);
    uint8_t frame[19]={FRAME_SOF};
    memcpy(frame+1,&x,4); memcpy(frame+5,&y,4); memcpy(frame+9,&theta,4); memcpy(frame+13,&stamp,4);
    for (int i=1;i<=16;++i) frame[17]+=frame[i];
    frame[18]=FRAME_EOF;
    uart_write_bytes(UART_PORT_NUM,frame,sizeof(frame));
}
