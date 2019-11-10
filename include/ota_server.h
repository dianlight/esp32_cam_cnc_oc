#ifndef _OTA_SERVER_H_
#define _OTA_SERVER_H_

#define OTA_LISTEN_PORT     8032
#define OTA_BUFF_SIZE       8192
#define OTA_TASK_NAME       "ota_server"
#define OTA_TASK_PRIORITY   10
#define OTA_TASK_STACK_SIZE (OTA_BUFF_SIZE + 4096)

typedef struct {
    int server_socket;
    int connect_socket;
    TaskHandle_t task;
} ota_server_handle_t;

esp_err_t ota_server_start();
void ota_server_stop();


#endif /* _OTA_SERVER_H_ */
