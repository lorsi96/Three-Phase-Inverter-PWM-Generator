#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#include "extended_mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_log.h"
#include "sine_table.h"

/* Pin Definitions*/
#define GPIO_PWM0A_OUT 21
#define GPIO_PWM1A_OUT 19
#define GPIO_PWM2A_OUT 18
#define GPIO_PWM0B_OUT 5
#define GPIO_PWM1B_OUT 17
#define GPIO_PWM2B_OUT 16
#define GPIO_EN_A 4
#define GPIO_EN_B 2
#define GPIO_EN_C 15
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_EN_A) | (1ULL << GPIO_EN_B) | (1ULL << GPIO_EN_C))

/* ISR Masks Definitions */
#define TIMER0_TEZ_INT_EN BIT(3)
#define TIMER1_TEZ_INT_EN BIT(4)
#define TIMER2_TEZ_INT_EN BIT(5)

/* Three Phase Control Config & Default Parameters */
#define CUSTOM_DEADTIME 500 //In ns
#define LINE_FREQ 1         //In Hz
#define DEFAULT_MF 1
#define DEFAULT_MA 1
#define DEFAULT_FS 1000
#define MASTER_PERIOD 200

/* Flow Variables */
//xQueueHandle timer_queue;
static const char *TAG = "Inverter";
//static mcpwm_dev_t *MCPWM[] = {&MCPWM0};
static int r;
static int s;
static int t;

/* Debug Stuff */
//const double debug_table[] = sinewave;
//const uint32_t tab_length = sine_len;

/*static void IRAM_ATTR isr_handler()
{
    //99. Interruption handler
    uint32_t evt = MCPWM[0]->int_st.val;
    xQueueSendFromISR(timer_queue, &evt, NULL);
    MCPWM[0]->int_clr.val = evt;
}*/

static void three_phase_inverter_gpio_initialize()
{
    //1.1 GPIO Initialization
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    ESP_LOGI(TAG, "\t1.1 Initialized GPIO");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_EN_A, 0);
    gpio_set_level(GPIO_EN_B, 0);
    gpio_set_level(GPIO_EN_C, 0);
}

static void three_phase_inverter_pwm_initialize()
{
    //2.1 PWM Config for each phase
    mcpwm_config_t pwm_config;
    pwm_config.frequency = DEFAULT_FS;
    pwm_config.cmpr_a = 90.0;
    pwm_config.cmpr_b = 90.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    pwm_config.frequency = DEFAULT_FS;
    pwm_config.cmpr_a = 25.0;
    pwm_config.cmpr_b = 25.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    pwm_config.frequency = DEFAULT_FS;
    pwm_config.cmpr_a = 75.0;
    pwm_config.cmpr_b = 75.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    ESP_LOGI(TAG, "\t2.1 PWMs Configured");
    //2.2 Deadtime and 180° phase between complementary switches config
    mcpwm_deadtime_enable2(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, CUSTOM_DEADTIME, CUSTOM_DEADTIME);
    mcpwm_deadtime_enable2(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, CUSTOM_DEADTIME, CUSTOM_DEADTIME);
    mcpwm_deadtime_enable2(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, CUSTOM_DEADTIME, CUSTOM_DEADTIME);
    ESP_LOGI(TAG, "\t2.2 Phase & Deadtime Configured");
    //2.3 Synchronization
    //mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 0);
    //mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_SYNC0, 0);
    ESP_LOGI(TAG, "\t2.3 Signals in Phase");
    // 2.4 Interrupts enable & Handler attachment
    /*MCPWM[0]->int_ena.val = TIMER0_TEZ_INT_EN;
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    ESP_LOGI(TAG, "\t2.3 Interruptions Enabled & Handler Configured");*/
}

static void dispatch_evt(void *arg)
{
    //3.1 Dispatcher Phases initializer
    /* static uint8_t toggler = 0;
    gpio_set_level(GPIO_TRIGGER, toggler++);
    toggler %= 2; */
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, table[r]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, table[r]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, table[s]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, table[s]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, table[t]);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, table[t]);
    r += 10;
    s += 10;
    t += 10;
    if (r >= tab_len)
        r = 0;
    if (s >= tab_len)
        s = 0;
    if (t >= tab_len)
        t = 0;
    //ESP_LOGI(TAG, "\tInterr %d", r);
}

static void three_phase_inverter_timers_table_init(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &dispatch_evt,
        .name = "master_timer"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_LOGI(TAG, "> Timer Created");

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MASTER_PERIOD));
    ESP_LOGI(TAG, "> Timer Initialized");

    r = 0;
    s = tab_len / 3;
    t = tab_len * 2 / 3;
    ESP_LOGI(TAG, "\t3.1 Phase Check :%d°:%d°:%d°", r * 360 / tab_len, s * 360 / tab_len, t * 360 / tab_len);
}

static void main_loop(void *arg)
{
    /*-- Setup --*/
    //1. GPIO Initialization
    three_phase_inverter_gpio_initialize();
    //2. Three Phase PWM Initialization
    three_phase_inverter_pwm_initialize();
    //3. Timer & Tab Indexes inialization
    three_phase_inverter_timers_table_init();
    /*--Loop--*/
    //4. Dispatcher Call -- Blocking
    while (1)
    {
        vTaskDelay(2000 / portTICK_RATE_MS);
        //ESP_LOGI(TAG, "\t3.1 Phase Check :%d°:%d°:%d°", r * 360 / tab_len, s * 360 / tab_len, t * 360 / tab_len);
    }
}

void app_main()
{
    //timer_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(main_loop, "three_phase_inverter_controller", 4096, NULL, 5, NULL);
}