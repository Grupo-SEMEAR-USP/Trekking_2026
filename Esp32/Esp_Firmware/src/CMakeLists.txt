idf_component_register(
  SRCS "main.c" "task_core0.c" "task_core1.c" "uart.c" "pwm.c" "pid.c" "initializers.c"
  INCLUDE_DIRS "../include"
  REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart esp_timer servo pid_ctrl rotary_encoder)
