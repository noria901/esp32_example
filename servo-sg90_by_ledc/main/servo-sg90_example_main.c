/* Drive servo SG90 by LEDC (LED PWM Controller) Example */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "drv8830";
/* stdin define */
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO5 as LEDC output,
 *    and it will change the duty repeatedly.
 */
#define LEDC_SERVO_CH0_GPIO      5
#define SG90_FREEQENCY            50
#define SG90_DUTY_MAX             1000
#define SG90_DUTY_MIN             0

/*
 * Prepare individual configuration
 * for each channel of LED Controller
 * by selecting:
 * - controller's channel number
 * - output duty cycle, set initially to 0
 * - GPIO number where LED is connected to
 * - speed mode, either high or low
 * - timer servicing selected channel
 *   Note: if different channels use one timer,
 *         then frequency and bit_num of these channels
 *         will be the same
 */
ledc_channel_config_t ledc_channel = {
    .channel    = LEDC_CHANNEL_0,
    .duty       = (SG90_DUTY_MAX + SG90_DUTY_MIN) / 2,
    .gpio_num   = LEDC_SERVO_CH0_GPIO,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .timer_sel  = LEDC_TIMER_0
};

void servo_init()
{
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = SG90_FREEQENCY,             // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0              // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);
    // Initialize fade service.
    ledc_fade_func_install(0);
}

esp_err_t servo_update_duty(int duty_delta)
{
    esp_err_t ret;

    ledc_channel.duty += duty_delta;
    ESP_LOGI(TAG, "update duty to %d", ledc_channel.duty);
    ret = ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, ledc_channel.duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    return ret;
}

esp_err_t servo_stop()
{
    esp_err_t ret = ledc_stop(ledc_channel.speed_mode, ledc_channel.channel,0x00);
    ESP_LOGI(TAG, "try ledc stop  %d", ret);

    return ret;
}

/* event handle*/
void on_uart_data_event(size_t size, const char* data)
{
    if (size <= 0) {
        ESP_LOGI(TAG, "data not found");
        return;
    }

    int ret = ESP_OK;
    switch(data[0]) {
        case 'a':
            ret = servo_update_duty(-10);
            break;
        case 'd':
            ret = servo_update_duty(10);
            break;
        case 'q':
            ret = servo_stop();
            break;
    }

    if(ret != ESP_OK) {
        ESP_LOGI(TAG, "drv write may error:%d",ret);
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    on_uart_data_event(event.size, (const char*) dtmp);
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void uart_init()
{
   /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    servo_init();
    uart_init();
     //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
