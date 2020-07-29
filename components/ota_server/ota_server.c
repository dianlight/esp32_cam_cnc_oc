#include <string.h>

#include "esp_system.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "ota_server.h"

static const char * TAG = OTA_TASK_NAME;

/*socket*/
static ota_server_handle_t ota_server_handle;

static int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

static int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);

    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}

static esp_err_t create_tcp_server(ota_server_handle_t * ota_server)
{
    ESP_LOGI(TAG, "server socket....port=%d", OTA_LISTEN_PORT);
    ota_server->server_socket = 0;
    struct sockaddr_in server_addr;
    ota_server->server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (ota_server->server_socket < 0) {
        show_socket_error_reason("create_server", ota_server->server_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(OTA_LISTEN_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(ota_server->server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("bind_server", ota_server->server_socket);
        close(ota_server->server_socket);
        return ESP_FAIL;
    }
    if (listen(ota_server->server_socket, 5) < 0) {
        show_socket_error_reason("listen_server", ota_server->server_socket);
        close(ota_server->server_socket);
        return ESP_FAIL;
    }
    struct sockaddr_in client_addr;
    unsigned int socklen = sizeof(client_addr);
    ota_server->connect_socket = accept(ota_server->server_socket, (struct sockaddr *)&client_addr, &socklen);
    if (ota_server->connect_socket < 0) {
        show_socket_error_reason("accept_server", ota_server->connect_socket);
        close(ota_server->server_socket);
        return ESP_FAIL;
    }
    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

void ota_server_listen(ota_server_handle_t * ota_server)
{
    ESP_ERROR_CHECK( create_tcp_server(ota_server) );

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
    	update_partition->subtype, update_partition->address);

    int recv_len;
    char ota_buff[OTA_BUFF_SIZE] = {0};
    bool is_req_body_started = false;
    int content_length = -1;
    int content_received = 0;

    esp_ota_handle_t ota_handle;
    do {
        recv_len = recv(ota_server->connect_socket, ota_buff, OTA_BUFF_SIZE, 0);
        if (recv_len > 0) {
            if (!is_req_body_started) {
                const char *content_length_start = "Content-Length: ";
                char *content_length_start_p = strstr(ota_buff, content_length_start) + strlen(content_length_start);
                sscanf(content_length_start_p, "%d", &content_length);
                ESP_LOGI(TAG, "Detected content length: %d", content_length);
                ESP_ERROR_CHECK( esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) );
                const char *header_end = "\r\n\r\n";
                char *body_start_p = strstr(ota_buff, header_end) + strlen(header_end);
                int body_part_len = recv_len - (body_start_p - ota_buff);
                esp_ota_write(ota_handle, body_start_p, body_part_len);
                content_received += body_part_len;
                is_req_body_started = true;
            } else {
                esp_ota_write(ota_handle, ota_buff, recv_len);
                content_received += recv_len;
            }
        }
        else if (recv_len < 0) {
            ESP_LOGE(TAG, "Error: recv data error! errno=%d", errno);
        }
        taskYIELD();
//        vTaskDelay(10);
    } while (recv_len > 0 && content_received < content_length);

    ESP_LOGI(TAG, "Binary transferred finished: %d bytes", content_received);

    ESP_ERROR_CHECK( esp_ota_end(ota_handle) );
    esp_err_t err = esp_ota_set_boot_partition(update_partition);
    char res_buff[60];
    int send_len;
    if (err == ESP_OK) {
        send_len = sprintf(res_buff, "200 OK\n\nSuccess. Next boot partition is %s\n", update_partition->label);
    } else {
        send_len = sprintf(res_buff, "400 Bad Request\n\nFailure. Error code: 0x%x\n", err);
    }
    send(ota_server->connect_socket, res_buff, send_len, 0);
    close(ota_server->connect_socket);

    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    ESP_LOGI(TAG, "Next boot partition subtype %d at offset 0x%x",
    	boot_partition->subtype, boot_partition->address);
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
}

void ota_server_task(void * ota_server) {
    ota_server_listen((ota_server_handle_t *) ota_server);
    ESP_LOGE(TAG, "OTA server returned from listening");
    vTaskDelay(portMAX_DELAY);
}

esp_err_t ota_server_start()
{
    if(xTaskCreate(ota_server_task, OTA_TASK_NAME, OTA_TASK_STACK_SIZE, (void *) &ota_server_handle, OTA_TASK_PRIORITY, &ota_server_handle.task) == pdPASS) {
        ESP_LOGD(TAG, "Create OTA server task");
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

void ota_server_stop()
{
    ESP_LOGD(TAG, "Stop OTA server task");
    if(ota_server_handle.task != NULL) {
        vTaskDelete(ota_server_handle.task);
        ota_server_handle.task = NULL;
    }
    close(ota_server_handle.server_socket); // FIXME This is causing mayhem
    close(ota_server_handle.connect_socket);
}