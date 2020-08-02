#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "errno.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "in_serial.h"
#include "tcp_serial.h"

static const char *TAG = "tcp_serial";

static int sock = -1;

static void udp_server_task(void *pvParameters)
{
    int ip_protocol = 0;

    struct sockaddr_in dest_addr_ip4;
    dest_addr_ip4.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4.sin_family = AF_INET;
    dest_addr_ip4.sin_port = htons(CONFIG_TCP_SERIAL_PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr_ip4, sizeof(dest_addr_ip4));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", CONFIG_TCP_SERIAL_PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket listening");

    char serial_buffer[256];
//    struct sockaddr_in source_addr;
//    uint addr_len = sizeof(source_addr);
    ESP_LOGI(TAG, "Serial Socket listening");

    while (1)
    {

        struct sockaddr_in source_addr;
        uint addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1)
        {
            int len = recv(sock, serial_buffer, sizeof(serial_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0)
            {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else
            {
                serial_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from |%s|", len, serial_buffer);

                for (uint8_t p = 0; p < len; p++)
                    putchar(serial_buffer[p]);
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

CLEAN_UP:
    if (listen_sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(listen_sock, 0);
        close(listen_sock);
    }

    vTaskDelete(NULL);
}

static void in_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    in_serial_buffer_t *line_data = (in_serial_buffer_t *)event_data;
//    ESP_LOGI(TAG, "(Ricevuto:%s %d %02X:%02X)", line_data->line_buffer, line_data->i_line
//                        ,line_data->line_buffer[line_data->i_line-2]
//                        ,line_data->line_buffer[line_data->i_line-1]);
    if(sock == -1)return;
//    ESP_LOGI(TAG, "%s %d", line_data->line_buffer, line_data->i_line);
    int err = send(sock, line_data->line_buffer, line_data->i_line, MSG_DONTWAIT);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}

esp_err_t startTcpSerial(void)
{
    if (xTaskCreate(udp_server_task, "udp_serial", 4096, NULL, 5, NULL) == pdPASS)
    {
        ESP_ERROR_CHECK(esp_event_handler_register(SERIAL_EVENT, SERIAL_EVENT_LINE, &in_event_handler, NULL));
        ESP_LOGD(TAG, "Create UDP_SERIAL server task");
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}