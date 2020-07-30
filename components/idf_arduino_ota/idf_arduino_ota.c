/* Based on OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_ota_ops.h"
//#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
//#include "nvs.h"
//#include "nvs_flash.h"
//#include "driver/gpio.h"
#include "errno.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "idf_arduino_ota.h"

enum arduino_ota_command
{
    FLASH = 0,
    SPIFFS = 100,
    AUTH = 200
};

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */

static const char *TAG = "idf_arduino_ota";
/*an ota data write buffer ready to write to the flash*/
//static char ota_write_data[BUFFSIZE + 1] = {0};
//extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
//extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

/*
static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}
*/

/* Event source periodic timer related definitions */
ESP_EVENT_DEFINE_BASE(OTA_EVENT);

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1)
    {
        ;
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

/*
static void infinite_loop(void)
{
    int i = 0;
    ESP_LOGI(TAG, "When a new firmware is available on the server, press the reset button to download it");
    while(1) {
        ESP_LOGI(TAG, "Waiting for a new firmware ... %d", ++i);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
*/

static void ota_task(void *pvParameter)
{
    ota_file_info_t *ota_file_info = (ota_file_info_t *)pvParameter;
    char rx_buffer[1460]; // From espota.py
 //   char addr_str[128];
    int addr_family;
    int ip_protocol;

    // Prepare Update
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA");
    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_STARTED,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    //

    while (1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(ota_file_info->addr_str);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(ota_file_info->port);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, ota_file_info->addr_str, sizeof(ota_file_info->addr_str) - 1);

        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
            break;
        }

        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", ota_file_info->addr_str, ota_file_info->port);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        update_partition = esp_ota_get_next_update_partition(NULL);
        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
                 update_partition->subtype, update_partition->address);
        assert(update_partition != NULL);

        int binary_file_length = 0;
        bool image_header_was_checked = false;
        while (1)
        {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                break;
            }
            else if (len > 0)
            {

                // Write Partition
                if (image_header_was_checked == false)
                {
                    esp_app_desc_t new_app_info;
                    if (len > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                    {
                        // check current version with downloading
                        memcpy(&new_app_info, &rx_buffer[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                        ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                        esp_app_desc_t running_app_info;
                        if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                        }

                        vTaskMissedYield();

                        const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalid_app_info;
                        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                        }

                        vTaskMissedYield();


                        // check current version with last invalid partition
                        if (last_invalid_app != NULL)
                        {
                            if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                            {
                                ESP_LOGW(TAG, "New version is the same as invalid version.");
                                ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                                ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                                ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                                break;
                            }
                        }

                        vTaskMissedYield();

#ifndef CONFIG_ARDUINO_OTA_SKIP_VERSION_CHECK
                        if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                            ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                            break;
                        }
#endif

                        image_header_was_checked = true;

                        err = esp_ota_begin(update_partition, ota_file_info->size, &update_handle);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                            ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                            break;
                        }
                        ESP_LOGI(TAG, "esp_ota_begin succeeded");
                    }
                    else
                    {
                        ESP_LOGE(TAG, "received package is not fit len");
                        ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                        break;
                    }
                }
                err = esp_ota_write(update_handle, (const void *)rx_buffer, len);
                if (err != ESP_OK)
                {
                    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                    task_fatal_error();
                }
                binary_file_length += len;
                ESP_LOGD(TAG, "Written image length %d", binary_file_length);
                ota_file_info->flashed = binary_file_length;
                ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_PROGRESS, ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));


                // ACK
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                int err = send(sock, "O", 1, 0);
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                    break;
                }
                vTaskMissedYield();
            }

            
            if (len == 0 || binary_file_length == ota_file_info->size)
            {
                ESP_LOGI(TAG, "Total Write binary data length: %d/%d", binary_file_length,ota_file_info->size);

                if (binary_file_length != ota_file_info->size)
                {
                    ESP_LOGE(TAG, "Error in receiving complete file");
                    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                    send(sock, "ERROR: incomplete file",23, 0);
                    break;
                }

                err = esp_ota_end(update_handle);
                if (err != ESP_OK)
                {
                    if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                    {
                        ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                    }
                    ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
                    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                    send(sock, "ERROR: esp_ota_end",19, 0);
                    break;
                }

                err = esp_ota_set_boot_partition(update_partition);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                    ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_ERROR,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));
                    send(sock, "ERROR: esp_ota_set_boot_partition",34, 0);
                    task_fatal_error();
                }

                ESP_ERROR_CHECK(esp_event_post(OTA_EVENT, OTA_EVENT_END,  ota_file_info, sizeof(ota_file_info_t), portMAX_DELAY));

                // Final ACK
                int err = send(sock, "OK", 3, 0);
                if (err < 0)
                {
                    ESP_LOGW(TAG, "Error occurred during sending final ACK: errno %d", errno);
                }

                ESP_LOGI(TAG, "Prepare to restart system! (5sec)");
                vTaskDelay(5000/portTICK_PERIOD_MS);

                esp_restart();
                return;
            }

            vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void tcp_server_task(void *pvParameters)
{
    int ip_protocol = 0;

    struct sockaddr_in dest_addr_ip4;
    dest_addr_ip4.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4.sin_family = AF_INET;
    dest_addr_ip4.sin_port = htons(CONFIG_ARDUINO_OTA_TRIGGER_PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_DGRAM, ip_protocol);
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
    ESP_LOGI(TAG, "Socket bound, port %d", CONFIG_ARDUINO_OTA_TRIGGER_PORT);


    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in source_addr;
        uint addr_len = sizeof(source_addr);
        
        int recv_len;
        char ota_inv_buff[256];
        ota_file_info_t* ota_file_info = (ota_file_info_t *) malloc(sizeof(ota_file_info_t));

        recv_len = recvfrom(listen_sock, ota_inv_buff, sizeof(ota_inv_buff) - 1, 0, (struct sockaddr *)&source_addr, &addr_len);
        if (recv_len > 0)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, ota_file_info->addr_str, sizeof(ota_file_info->addr_str) - 1);
            ota_inv_buff[recv_len] = 0; /// Null-terminate whatever we received and treat like a string...
            ESP_LOGI(TAG, "Received %d bytes from %s:", recv_len, ota_file_info->addr_str);
            ESP_LOGI(TAG, "Invitation '%s'  ", ota_inv_buff);
            sscanf(ota_inv_buff, "%" SCNu8 " %" SCNu16 " %d %s", &ota_file_info->command, &ota_file_info->port, &ota_file_info->size, ota_file_info->filemd5);
            ESP_LOGI(TAG, "Command %d", ota_file_info->command);
            ESP_LOGI(TAG, "Port %d", ota_file_info->port);
            ESP_LOGI(TAG, "Size %d", ota_file_info->size);
            ESP_LOGI(TAG, "FileMD5 %s", ota_file_info->filemd5);
            if (ota_file_info->command == FLASH)
            {
                ESP_LOGE(TAG, "Command FLASH!");

                int err = sendto(listen_sock, "OK", 2, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                
                xTaskCreate(&ota_task, "ota_task", 8192, ota_file_info, 5, NULL);

            }
            else
            {
                ESP_LOGE(TAG, "Command not implemented!");
            }
        }
        else if (recv_len < 0)
        {
            ESP_LOGE(TAG, "Error: recv data error! errno=%d", errno);
            break;
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


esp_err_t start_arduino_ota()
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for the partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");


    if (xTaskCreate(tcp_server_task, "arduno_ota_watch", 4096, NULL, 5, NULL) == pdPASS)
    {
        ESP_LOGD(TAG, "Create OTA server task");
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

/*
void stop_arduino_ota()
{
    ESP_LOGD(TAG, "Stop OTA server task");
    if (ota_server_handle.task != NULL)
    {
        vTaskDelete(ota_server_handle.task);
        ota_server_handle.task = NULL;
    }
    close(ota_server_handle.server_socket); // FIXME This is causing mayhem
    close(ota_server_handle.connect_socket);
}
*/