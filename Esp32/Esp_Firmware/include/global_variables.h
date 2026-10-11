#pragma once
#include <stdint.h>
#include <stdbool.h>
extern float global_ros_angular_speed_left, global_ros_angular_speed_right;
extern float global_ros_servo_angle; /* Ângulo equivalente das rodas em radianos. */
extern float global_motor_angular_speed_left, global_motor_angular_speed_right;
extern double global_total_x, global_total_y, global_total_theta; /* mm, mm, rad */
extern uint32_t global_timer_miliseconds, global_time_stamp_miliseconds;
extern int64_t global_last_command_us;
extern uint8_t led_byte;
