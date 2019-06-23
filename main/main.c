#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "extended_mcpwm.h"
//#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

/* Pin Definitions*/
#define GPIO_PWM0A_OUT 15 //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16 //Set GPIO 16 as PWM0B

static const char *TAG = "SinglePhase";
const double debug_table[] = {1.0, 50.0, 99.0, 50.0};
const uint8_t tab_length = 4;

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void update_pwm(const double *table, const uint8_t tab_len, const mcpwm_unit_t mcpwm_num, const mcpwm_timer_t timer_num)
{
    /* Output next sample */
    static uint8_t table_pointer = 0;
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, table[table_pointer]);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, table[table_pointer]);
    ++table_pointer;
    table_pointer %= tab_len;
}

static void main_loop(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    ESP_LOGI(TAG, "Initialized GPIO");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 1000, 1000); //10us
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    ESP_LOGI(TAG, "Initialized MCPWM");
    ESP_LOGI(TAG, "Deadtime Enabled");
    while (1)
    {
        update_pwm(debug_table, tab_length, MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(main_loop, "three_phase_inverter_controller", 4096, NULL, 5, NULL);
}