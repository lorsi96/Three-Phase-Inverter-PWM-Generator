#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"

#include "extended_mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_log.h"

/* Pin Definitions*/
#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B
#define CAP_SIG_NUM 1       //Number of Capture signals
#define CAP0_INT_EN BIT(27) //Capture 0 interrupt bit
#define TIMER0_TEZ_INT_EN BIT(3)
#define TIMER1_TEZ_INT_EN BIT(4)
#define TIMER2_TEZ_INT_EN BIT(5)

/* Const Definitions */
#define CUSTOM_DEADTIME 1000 //In ns
xQueueHandle timer_queue;

static const char *TAG = "SinglePhase";
const double debug_table[] = {1.0, 50.0, 99.0, 50.0};
const uint8_t tab_length = 4;
static mcpwm_dev_t *MCPWM[] = {&MCPWM0};

static void IRAM_ATTR isr_handler()
{
    uint32_t evt = MCPWM[0]->int_st.val;
    xQueueSendFromISR(timer_queue, &evt, NULL);
    MCPWM[0]->int_clr.val = evt;
}

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}
static void dispatch_timer_evt(const double *table, const uint8_t tab_len)
{
    static uint8_t table_pointer = 0;
    uint32_t evt = 0;
    while (1)
    {
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        if (evt)
        {
            ESP_LOGI(TAG, "Some Event %d", evt);
        }
        if (evt & TIMER0_TEZ_INT_EN)
        {
            ESP_LOGI(TAG, "Desired Queue Received");
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM0A, MCPWM_OPR_A, table[table_pointer]);
            ++table_pointer;
            table_pointer %= tab_len;
            MCPWM[0]->int_ena.val = 0;
        }
    }
}

static void main_loop(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    ESP_LOGI(TAG, "Initialized GPIO");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    if (mcpwm_deadtime_enable2(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE_FROM_PWMXA, CUSTOM_DEADTIME, CUSTOM_DEADTIME))
        ESP_LOGE(TAG, "Deadtime Failed");
    ; //10us
    if (mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config) == ESP_ERR_INVALID_ARG)
        ESP_LOGE(TAG, "Initialization Failed");
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM0A, MCPWM_OPR_A, 50.0);
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    ESP_LOGI(TAG, "Initialized MCPWM");
    MCPWM[0]->int_ena.val = TIMER0_TEZ_INT_EN;
    dispatch_timer_evt(debug_table, tab_length);
}

void app_main()
{
    timer_queue = xQueueCreate(1, sizeof(uint32_t));
    printf("Testing brushed motor...\n");
    xTaskCreate(main_loop, "three_phase_inverter_controller", 4096, NULL, 5, NULL);
}