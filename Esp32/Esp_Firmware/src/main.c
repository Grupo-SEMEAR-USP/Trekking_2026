#include "types.h"
#include "task_core0.h"
#include "task_core1.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "initializers.h"

    //declaring locks
    SemaphoreHandle_t xSemaphore_getSpeed;
    SemaphoreHandle_t xSemaphore_getRosSpeed;
    SemaphoreHandle_t xSemaphore_getLed;
    EventGroupHandle_t initialization_groupEvent;

    const int task0_init_done = 0b01;
    const int task1_init_done = 0b10;

    //declaring and initializing global variables
    float global_ros_angular_speed_left = 0;
    float global_ros_angular_speed_right = 0;

    float global_motor_angular_speed_left = 0 ;
    float global_motor_angular_speed_right = 0;

    bool global_led_state = 0;

    double global_total_x = 0;
    double global_total_y = 0;
    double global_total_theta = 0;//PI/2;

    float global_servo_angle = (float) SERVO_INITIAL_ANGLE;
    float global_ros_servo_angle = (float) SERVO_INITIAL_ANGLE;

    uint32_t global_timer_miliseconds = 0;
    uint32_t global_time_stamp_miliseconds = 0;


void app_main() {

    // gpio_set_direction(STAND_BY, GPIO_MODE_DEF_OUTPUT);

    // esp_err_t err = gpio_set_level(STAND_BY, 1);

    // //initializing locks
    // xSemaphore_getSpeed = xSemaphoreCreateMutex(); 
    // xSemaphore_getRosSpeed = xSemaphoreCreateMutex();

    // initialization_groupEvent = xEventGroupCreate(); //it's perhaps not necessary

    // // //Inicializar as tasks
    // xTaskCreatePinnedToCore(&core0fuctions, "task que inicializa pwm,encoders e pid no core 0", 2048, NULL, 1, NULL, 0);
    // xTaskCreatePinnedToCore(&core1functions, "task que inicializa o i2c no core 1", 8192, NULL, 1, NULL, 1);

    //variables for intializing encoder
    rotary_encoder_t *encoder_left;
    rotary_encoder_t *encoder_right;

    pwm_motors_init();

        //selecting pcnt units
        uint32_t pcnt_unit_0 = 0;
        uint32_t pcnt_unit_1 = 1;

        // Create rotary encoder instances
        rotary_encoder_config_t config_encoder_left = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_0, PCNT_CHA_LEFT, PCNT_CHB_LEFT);
        encoder_left = NULL;
        ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_left, &encoder_left));

        rotary_encoder_config_t config_encoder_right = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_1, PCNT_CHA_RIGHT, PCNT_CHB_RIGHT);
        encoder_right = NULL;
        ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_right, &encoder_right));

        // Filter out glitch (1us)
        // ESP_ERROR_CHECK(encoder_left->set_glitch_filter(encoder_left, 10));
        // ESP_ERROR_CHECK(encoder_right->set_glitch_filter(encoder_right, 10));

        // Start encoder
        ESP_ERROR_CHECK(encoder_left->start(encoder_left));
        ESP_ERROR_CHECK(encoder_right->start(encoder_right));
    
    while(1){
        vTaskDelay(pdMS_TO_TICKS(100));
        pwm_actuate(ESQ,1023);
        pwm_actuate(DIR,1023);
        
    }
}

