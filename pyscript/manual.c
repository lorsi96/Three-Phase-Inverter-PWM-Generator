#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "sine_table.h"

//Parameters
#define LINE_FREQ 50 //Hz
#define MF 21
#define TRIANG_FREQ (MF * LINE_FREQ)
#define PWM_RESOLUTION 100
#define MASTER_PERIOD (int(1000000 / (TRIANG_FREQ * PWM_RESOLUTION)))
#define DEAD_TIME_SAMPLES (int(10 / MASTER_PERIOD))
#define TABLE_SIZE 2100

#define A_BASE_IND 0
#define B_BASE_IND (int(TABLE_SIZE / 3))
#define C_BASE_IND (int(TABLE_SIZE * 2 / 3))

//Gpio definitions -- A & B refer to differential pairs
#define PWM_1A 21
#define PWM_1B 19
#define PWM_2A 18
#define PWM_2B 5
#define PWM_3A 17
#define PWM_3B 16
#define GPIO_OUTPUT_PIN_SEL ((1ULL << PWM_1A) | (1ULL << PWM_1B) | (1ULL << PWM_2A) | (1ULL << PWM_2B) | (1ULL << PWM_3A) | (1ULL << PWM_3B))
static void periodic_timer_callback(void *arg);

static const char *TAG = "example";

void init_timer(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "master_timer"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_LOGI(TAG, "> Timer Created");

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MASTER_PERIOD));
    ESP_LOGI(TAG, "> Timer Initialized");
}

void init_gpio(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pin_bit_mask = ;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void periodic_timer_callback(void *arg)
{
    static uint32_t master_counter = 0;
    static uint32_t aind = A_BASE_IND;
    static uint32_t bind = B_BASE_IND;
    static uint32_t cind = C_BASE_IND;
    static uint32_t offset = 10;
    if (!master_counter)
    {
        gpio_set_level(PWM_1B, 0);
        gpio_set_level(PWM_2B, 0);
        gpio_set_level(PWM_3B, 0);
        usleep(10);
        gpio_set_level(PWM_1A, 1);
        gpio_set_level(PWM_2A, 1);
        gpio_set_level(PWM_3A, 1);
    }
    else if (master_counter > sinewave[aind])
    {
        gpio_set_level(PWM_1A, 0);
        usleep(10);
        gpio_set_level(PWM_1B, 1);
    }
    if (master_counter > sinewave[bind])
    {
        gpio_set_level(PWM_2A, 0);
        usleep(10);
        gpio_set_level(PWM_2B, 1);
    }
    if (master_counter > sinewave[cind])
    {
        gpio_set_level(PWM_3A, 0);
        usleep(10);
        gpio_set_level(PWM_3B, 1);
    }
    aind += offset;
    bind += offset;
    cind += offset;
    master_counter++;
    master_counter %= PWM_RESOLUTION;
    aind %= TABLE_SIZE;
    bind %= TABLE_SIZE;
    cind %= TABLE_SIZE;
}

void main_loop(void)
{
    init_gpio();
    init_timer();
    while (1)
    {
        usleep(2000000);
    }
}

void app_main()
{
    //timer_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(main_loop, "three_phase_inverter_controller", 4096, NULL, 5, NULL);
}
