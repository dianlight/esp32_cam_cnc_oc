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
#include "infoDisplay.h"

#include "errno.h"

static const char *TAG = "in_serial";

ESP_EVENT_DEFINE_BASE(SERIAL_EVENT);

static QueueHandle_t uart_queue;

bool nextParam(uint8_t *cursor,char *line,char* name,char args[3][10]){
    for(int y=0; y< 3; y++)
      memset(&args[y], 0x00, sizeof(args[y]));
    char pmstr[255];
    int r = sscanf(&line[*cursor],"%255[^|>]",pmstr);
    sscanf(pmstr,"%10[^:]:%10[^,],%10[^,],%10[^,]",name,args[0],args[1],args[2]);
    (*cursor)+=strlen(pmstr)+1;
    return r != -1;
}

// Parsing GRBL status message <...>
void parsingStatusMessage(char *data, uint16_t size)
{
    uint8_t cursor = 0;
    char name[10];
    char argv[3][10] = {0};
    while(nextParam(&cursor,data+1,name,argv)){
            ESP_LOGI(TAG,"Param %d %s \n\tArgs:",cursor,name);
            for(int v=0;v < 3 && argv[v][0] != 0x00;v++){
                ESP_LOGI(TAG,"[%s]",argv[v]);
            }

            // Status
            if(strcmp(name,"Idle") == 0){
                info_display_handle.status = GRBL_IDLE;
                info_display_handle.status_blink = false;
            } else if (strcmp(name,"Run") == 0){
                info_display_handle.status = GRBL_RUN;
                info_display_handle.status_blink = false;
            } else if (strcmp(name,"Hold") == 0){
                info_display_handle.status = GRBL_HOLD;
                if(argv[0][0]=='0'){ // Hold completed
                    info_display_handle.status_blink = false;
                } else {             // Hold in progress
                    info_display_handle.status_blink = true;
                }
            } else if (strcmp(name,"Home") == 0){
                info_display_handle.status = GRBL_HOME;
                info_display_handle.status_blink = false;
            } else if (strcmp(name,"Jog") == 0){
                info_display_handle.status = GRBL_JOG;
                info_display_handle.status_blink = false;
            } else if (strcmp(name,"Alarm") == 0){
                info_display_handle.status = GRBL_ALARM;
                info_display_handle.status_blink = true;
            } else if (strcmp(name,"Door") == 0){
                info_display_handle.status = GRBL_HOLD;
                if(argv[0][0]=='0'){ // Door completed
                    info_display_handle.status_blink = false;
                } else {             // Door in progress 1,2,3 (@see https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface#interacting-with-grbls-systems)
                    info_display_handle.status_blink = true;
                }
            } else if (strcmp(name,"Check") == 0){
                info_display_handle.status = GRBL_CHECK;
                info_display_handle.status_blink = true;
            } else if (strcmp(name,"Sleep") == 0){
                info_display_handle.status = GRBL_SLEEP;
                info_display_handle.status_blink = false;
            } else
            // MPos - Current Position
            if (strcmp(name,"MPos") == 0){
                info_display_handle.x = atof(argv[0]);
                info_display_handle.y = atof(argv[1]);
                info_display_handle.z = atof(argv[2]);
            } else 
            // WCO - Work Coordinate Offset
            if (strcmp(name,"WCO") == 0){
                info_display_handle.xco = atof(argv[0]);
                info_display_handle.yco = atof(argv[1]);
                info_display_handle.zco = atof(argv[2]);
            } else 
            // Bf - Buffer State
            if (strcmp(name,"Bf") == 0){
                info_display_handle.bf = atoi(argv[0]);
                info_display_handle.bfmax = atoi(argv[1]);
            } else 
            // Ln - Line number
            if (strcmp(name,"Ln") == 0){
                info_display_handle.ln = atoi(argv[0]);
            } else 
            // F FS - Current Feed and Speed
            if (strcmp(name,"F") == 0){
                info_display_handle.fr = atoi(argv[0]);
            } else if (strcmp(name,"FS") == 0){
                info_display_handle.fr = atoi(argv[0]);
                info_display_handle.speed = atoi(argv[1]);
            } else  
            // Pn - Input Pin State
            if (strcmp(name,"Pn") == 0){
                info_display_handle.limitXYZ[0] = false;    // Pin State                Pn:XYZ
                info_display_handle.limitXYZ[1] = false;    // Pin State                Pn:XYZ
                info_display_handle.limitXYZ[2] = false;    // Pin State                Pn:XYZ
                info_display_handle.limitP= false;         // Pin State                Pn:P
                info_display_handle.pinDoor= false;        // Pin State                Pn:D
                info_display_handle.pinHold= false;        // Pin State                Pn:H
                info_display_handle.pinReset= false;       // Pin State                Pn:R
                info_display_handle.pinStart= false;       // Pin State                Pn:S

                for(int c=0; c < strlen(argv[0]);c++){
                    switch(argv[0][c]){
                        case 'X': // Pin State                Pn:XYZ
                            info_display_handle.limitXYZ[0] = true;
                        break;
                        case 'Y': // Pin State                Pn:XYZ
                            info_display_handle.limitXYZ[1] = true;
                        break;
                        case 'Z': // Pin State                Pn:XYZ
                            info_display_handle.limitXYZ[2] = true;
                        break;
                        case 'P': // Pin State                Pn:P
                            info_display_handle.limitP= true;
                        break;
                        case 'D': 
                            info_display_handle.pinDoor = true;
                        break;
                        case 'H': 
                            info_display_handle.pinHold = true;
                        break;
                        case 'R': 
                            info_display_handle.pinReset = true;
                        break;
                        case 'S': 
                            info_display_handle.pinStart = true;
                        break;
                    }
                }
            } else 
            // Ov - Override Values
            if (strcmp(name,"Ov") == 0){
               info_display_handle.ofeed = atoi(argv[0]);
               info_display_handle.orapids = atoi(argv[1]);
               info_display_handle.ospeed = atoi(argv[2]);
            } else 
            // A - Accessory State
            if (strcmp(name,"A") == 0){
                info_display_handle.spindle = SPINDLE_OFF;
                info_display_handle.flood = false;
                info_display_handle.mist = false;
                for(int c=0; c < strlen(argv[0]);c++){
                    switch(argv[0][c]){
                        case 'S': // S indicates spindle is enabled in the CW direction. This does not appear with C.
                            info_display_handle.spindle = SPINDLE_CW;
                        break;
                        case 'C': // C indicates spindle is enabled in the CCW direction. This does not appear with S.
                            info_display_handle.spindle = SPINDLE_CCW;
                        break;
                        case 'F': // F indicates flood coolant is enabled.
                            info_display_handle.flood = true;
                        break;
                        case 'M': // M indicates mist coolant is enabled.
                            info_display_handle.mist = true;
                        break;
                    }
                }
            }
        }
}

static void in_serial_task(void *pvParameters)
{
    uart_event_t event;
    in_serial_buffer_t *in_serial_buffer;
    in_serial_buffer = (in_serial_buffer_t *)malloc(sizeof(in_serial_buffer_t));
    uint8_t dtmp[IN_SERIAL_BUFFER_SIZE];

    bzero(in_serial_buffer->line_buffer, IN_SERIAL_BUFFER_SIZE);
    in_serial_buffer->i_line = 0;
    bool in_status_message = false;
    while (true)
    {

        if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            ESP_LOGD(TAG, "uart[%d] event:%d", CONFIG_ESP_CONSOLE_UART_NUM, event.type);

            if (event.type == UART_DATA)
            {
                bzero(dtmp, IN_SERIAL_BUFFER_SIZE);
                uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, dtmp, event.size, portMAX_DELAY);
                for (uint16_t p = 0; p < event.size; p++, in_serial_buffer->i_line++)
                {
                    in_serial_buffer->line_buffer[in_serial_buffer->i_line] = dtmp[p];
//                    ESP_LOGI(TAG, "%02X/%d/%d", dtmp[p], in_serial_buffer->i_line, event.size);

                    // STATUS PARSING
                    if (in_serial_buffer->i_line == 0 && dtmp[p] == '<')
                    {
                        // Start Status Message
//                        ESP_LOGI(TAG,"Start status message!");
                        in_status_message = true;
                    }
                    else if (in_status_message && dtmp[p] == '>')
                    {
                        // Fine status Mesage - Parsing
                        ESP_LOGI(TAG,"Got status message! %s %d",in_serial_buffer->line_buffer,in_serial_buffer->i_line);
                        in_status_message = false;
                        info_display_handle.lastStatusUpdate = (unsigned long) (esp_timer_get_time() / 1000ULL);
                        parsingStatusMessage((char *)in_serial_buffer->line_buffer, in_serial_buffer->i_line);
                    }

                    if (dtmp[p] == '\n' || dtmp[p] == '\r' || in_serial_buffer->i_line >= IN_SERIAL_BUFFER_SIZE - 1)
                    {
                    //    ESP_LOGI(TAG, "(Invio:%s %d %02X)", in_serial_buffer->line_buffer, in_serial_buffer->i_line + 1, in_serial_buffer->line_buffer[in_serial_buffer->i_line]);
                        in_serial_buffer->i_line++;
                        ESP_ERROR_CHECK(esp_event_post(SERIAL_EVENT, SERIAL_EVENT_LINE, in_serial_buffer, sizeof(in_serial_buffer_t), portMAX_DELAY));
                        in_serial_buffer->i_line = -1;
                        in_status_message = false;
                        bzero(in_serial_buffer->line_buffer, IN_SERIAL_BUFFER_SIZE);
                    }
                }
            } // if event.type

        } //if xQuequeReceive
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

esp_err_t initialize_in_serial(void)
{

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);
    /* Disable buffering also on stdout */
    setvbuf(stdout, NULL, _IONBF, 0);

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