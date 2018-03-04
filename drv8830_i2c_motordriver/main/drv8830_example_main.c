/* i2c motor driver dri8830 - Example
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "drv8830";
/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C motor driver drv8830 module by running tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of drv8830 with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

SemaphoreHandle_t print_mux = NULL;

/* stdin define */
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

/* DRV8830 define*/
#define DRV8830_ADDR                       0x64             /*!< slave address for DRV8830 */
/* DRV8830 sub_addr*/
#define DRV_CTR_ADDR                        0x00             /*!< Sets state of outputs and output voltage */
#define DRV_FLT_ADDR                        0x01            /*!< Allows reading and clearing of fault conditions */
/* DRV8830 data */
#define DRV8830_VSET_MAX                 0x3F  // 5.06v in 5V
#define DRV8830_VSET_MIN                 0x06  // 0.48v in 5V
#define DRV8830_CTR_STANDBY              0x00 //
#define DRV8830_CTR_BACK                 0x01 //
#define DRV8830_CTR_FORWARD              0x02 //
#define DRV8830_CTR_BRAKE                0x03 //

/**
 * @brief code to write drv8830
 * _______________________________________________________________________________________
 * | start | slave_addr + ack | sub_addr + ack | data + ack | data + ack  | stop |
 * --------|--------------------|------------------|-------------|--------------|-------
 *
 * IN1 IN2 OUT1 OUT2 Function
 * 0   0   Z     Z    Standby/coast
 * 0   1   L     H    Reverse
 * 1   0   H     L    Forward
 * 1   1   H     H      Brake
 *
 *  D7 - D2 D1 D0
 * VSET[5..0] IN2 IN1
 */
static esp_err_t drv8830_ctr(uint8_t ctr, uint8_t vset)
{
    ESP_LOGI(TAG, "drv8830_ctr ctr: %d  vset: %d", ctr, vset);
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DRV8830_ADDR  << 1 | WRITE_BIT, ACK_CHECK_EN); // slave_addr
    i2c_master_write_byte(cmd, DRV_CTR_ADDR, ACK_CHECK_EN); // sub_addr
    i2c_master_write_byte(cmd, ( vset << 2 ) | ctr, ACK_CHECK_EN); // data
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "drv8830_ctr ret:%d", ret);
    vTaskDelay(30 / portTICK_RATE_MS);
    
    return ret;    
}

static esp_err_t drv8830_flt(uint8_t* data_rd)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DRV8830_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, DRV_FLT_ADDR, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, ACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t drv8830_flt_reset()
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, DRV8830_ADDR << 1  | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, DRV_FLT_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "drv8830_flt_reset ret:%d", ret);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int ret;
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    ret = i2c_param_config(i2c_master_port, &conf);
    ESP_LOGI(TAG, "init config %d", ret);
    ret = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    ESP_LOGI(TAG, "init install %d", ret);
}

uint8_t last_ctrl = DRV8830_CTR_STANDBY;
uint8_t current_vset = DRV8830_VSET_MIN;

esp_err_t update_driving(uint8_t ctr)
{
    int ret;
    if ( last_ctrl != ctr)
    {
        current_vset =  DRV8830_VSET_MIN;
        last_ctrl = ctr;
    }
    ret = drv8830_ctr(last_ctrl , current_vset);
    current_vset += 0x05;
    if(current_vset > DRV8830_VSET_MAX)
        current_vset = DRV8830_VSET_MAX;    

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
        case 'w':
            ESP_LOGI(TAG, "foword");
            ret = update_driving(DRV8830_CTR_FORWARD);
            break;
        case 's':
            ESP_LOGI(TAG, "back");
            ret = update_driving(DRV8830_CTR_BACK);
            break;
        case 'b':
            ESP_LOGI(TAG, "brake");
            ret = update_driving(DRV8830_CTR_BRAKE);
            break;
        case 'q':
            ESP_LOGI(TAG, "standby");
            ret = update_driving(DRV8830_CTR_STANDBY);
            break;
        case 'r':
            ESP_LOGI(TAG, "flt reset");
            ret = drv8830_flt_reset();
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

    i2c_master_init();
    drv8830_flt_reset();
    uart_init()
    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

