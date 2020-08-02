#include "in_serial.h"
#include <stdio.h>
#include <string.h>
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"

#include "errno.h"


static const char *TAG = "in_serial";

ESP_EVENT_DEFINE_BASE(SERIAL_EVENT);     

static QueueHandle_t uart_queue;

static void in_serial_task(void *pvParameters)
{
    uart_event_t event;
    in_serial_buffer_t *in_serial_buffer; 
    in_serial_buffer = (in_serial_buffer_t *)malloc(sizeof(in_serial_buffer_t));
    uint8_t dtmp[IN_SERIAL_BUFFER_SIZE];

    bzero(in_serial_buffer->line_buffer, IN_SERIAL_BUFFER_SIZE);
    while (true)
    {

        if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            ESP_LOGI(TAG, "uart[%d] event:%d", CONFIG_ESP_CONSOLE_UART_NUM,event.type);

            if (event.type == UART_DATA)
            {
                bzero(dtmp, IN_SERIAL_BUFFER_SIZE);
                uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, dtmp, event.size, portMAX_DELAY);
                for(uint16_t p=0;p < event.size; p++, in_serial_buffer->i_line++){
                    in_serial_buffer->line_buffer[in_serial_buffer->i_line]=dtmp[p];
//                    ESP_LOGI(TAG,"%02X/%d/%d",dtmp[p],in_serial_buffer->i_line, event.size);;
                    if(dtmp[p] == '\n' || in_serial_buffer->i_line >= IN_SERIAL_BUFFER_SIZE-1 ){
//                        ESP_LOGI(TAG, "(Invio:%s %d %02X:%02X)", in_serial_buffer->line_buffer, in_serial_buffer->i_line+1
//                        ,in_serial_buffer->line_buffer[in_serial_buffer->i_line-1]
//                        ,in_serial_buffer->line_buffer[in_serial_buffer->i_line]
//                        );
                        in_serial_buffer->i_line++;
                        ESP_ERROR_CHECK(esp_event_post(SERIAL_EVENT, SERIAL_EVENT_LINE, in_serial_buffer, sizeof(in_serial_buffer_t), portMAX_DELAY));
                        in_serial_buffer->i_line = 0;
                        bzero(in_serial_buffer->line_buffer, IN_SERIAL_BUFFER_SIZE);
                    }
                }
            } // if event.type

        } //if xQuequeReceive
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

esp_err_t initialize_in_serial(void)
{

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .use_ref_tick = true};
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                                        256, 0, 256, &uart_queue, 0));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    if (xTaskCreate(in_serial_task, "in_serial", 4096, NULL, 10, NULL) == pdPASS)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}