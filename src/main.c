/* Esp32_Cam_CNC_Offline_Controller
 * HW TODO:
 *  [x] Display
 *  [x] Wifi
 *  [ ] Bluetooth serial
 *  [x] Joysitck
 *  [x] Buttons
 *  [x] Cam
 *  [x] Himem (8Gb)
 * SW TODO:
 *  [x] Camera streaming 
 *  [x] Wifi configuration (Softap provisioning)
 *  [x] Display on own task (autorefresh)
 *  [x] Wifi Socket serial
 *  [x] Icon Menu
 *  [x] OTA
 *    [x] Correct watchdog conflict
 *    [x] Ota events!
 *  [x] mDNS
 * GRBL TODO:
 *  [ ] Log out of serial! 
 *      [x] GRBL compatible comment? ()
 *      [x] UDP log
 *  [ ] Display status
 *  [ ] Manual command
 *  [ ] Probe command
*/
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#ifdef CONFIG_UDP_LOG_SUPPORT
    #include "udp_logging.h"
#endif
#include "pinConfig.h"
#include <stdio.h>
//#include <dirent.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp32/himem.h"

#include "esp_http_server.h"
#include "esp_timer.h"
#include <esp_wifi.h>
#include <esp_event.h>

#include <nvs_flash.h>
#include <sys/param.h>
#include <i2cdev.h>
#include <mdns.h>

#include "grbl_friendly_log.h"

#include "infoDisplay.h"
#include "hid.h"
#include "mmc.h"
#include "in_serial.h"
#ifdef CONFIG_TCP_SERIAL_SUPPORT
    #include "tcp_serial.h"
#endif
#ifdef CONFIG_BLUETOOTH_SERIAL_SUPPORT
    #include "bluetooth.h"
#endif

#include "cam.h"

#include "wifi_prov_mgr.h"

#include "idf_arduino_ota.h"

static const char *TAG = "mainapp";

// WebServer and WiFi
httpd_uri_t uri_handler_jpg = {
    .uri = "/jpg",
    .method = HTTP_GET,
    .handler = jpg_stream_httpd_handler};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_ERROR_CHECK(mdns_service_add("WebCam", "_http", "_tcp", 80, NULL, 0));
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_handler_jpg);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void ota_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ota_file_info_t *ota_file_info = (ota_file_info_t*)event_data;
    static uint8_t operc = 0;
    uint8_t perc;
    switch (event_id)
    {
    case OTA_EVENT_STARTED:
        ESP_LOGI(TAG,"OTA Started %p",ota_file_info);
        operc = 0;
        otaDisplay(0);
        break;
    case OTA_EVENT_END:
        ESP_LOGI(TAG,"OTA Stopped");
        break;
    case OTA_EVENT_PROGRESS:
        perc = ota_file_info->flashed * 100 / ota_file_info->size;
        ESP_LOGD(TAG,"OTA Running %d %d/%d",perc,ota_file_info->flashed,ota_file_info->size);
        if(operc != perc){
            operc = perc;
            otaDisplay(perc);
        }
        break;
    case OTA_EVENT_ERROR:
        ESP_LOGE(TAG,"OTA Error");
        break;
    default:
        break;
    }
}


static void event_handler(void *arg, esp_event_base_t event_base,
                          int event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *ip_data = (ip_event_got_ip_t *)event_data;
        info_display_handle.wifi = true;

        start_arduino_ota();
        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, NULL));
//        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_STARTED, &ota_event_handler, NULL));
//        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_END, &ota_event_handler, NULL));
//        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_PROGRESS, &ota_event_handler, NULL));
//        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_ERROR, &ota_event_handler, NULL));

        /* Start the web server */
        if (*server == NULL)
        {
            ESP_LOGI(TAG, "Starting server: '%s:80'",
                     ip4addr_ntoa(&ip_data->ip_info.ip));

            *server = start_webserver();
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
        info_display_handle.wifi = false;
        esp_wifi_connect();
//        ota_server_stop();
        /* Stop the web server */
        if (*server)
        {
            stop_webserver(*server);
            *server = NULL;
        }
    }
}




// Main
void app_main(void)
{
    esp_log_set_vprintf(grbl_friendly_logging_vprintf);
    esp_log_level_set("*", ESP_LOG_DEBUG); // set all components to ERROR level
    esp_log_level_set("spiram", ESP_LOG_WARN);
    esp_log_level_set("efuse", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);                 
    esp_log_level_set("wpa", ESP_LOG_WARN);                   
    esp_log_level_set("httpd_parse", ESP_LOG_INFO);          
    esp_log_level_set("camera", ESP_LOG_INFO);               
    esp_log_level_set("cam", ESP_LOG_DEBUG);               
    esp_log_level_set("idisplay", ESP_LOG_DEBUG);               
//    esp_log_level_set("I2C_DEV", ESP_LOG_DEBUG);               
    esp_log_level_set("hid", ESP_LOG_DEBUG);               
    esp_log_level_set("mmc", ESP_LOG_DEBUG);               
    esp_log_level_set("mainapp", ESP_LOG_DEBUG); 
    esp_log_level_set("ota_server",ESP_LOG_DEBUG);
    esp_log_level_set("wifi_prov_mgr",ESP_LOG_INFO);
    esp_log_level_set("in_serial",ESP_LOG_DEBUG);
    esp_log_level_set("tcp_serial",ESP_LOG_INFO);

    ESP_LOGI(TAG,"Application Start");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(initDisplay());

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("(This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash)\n", (spi_flash_get_chip_size()+esp_himem_get_phys_size()) / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("(Free heap: %d)\n", esp_get_free_heap_size());

    printf("(Free heap size: %d)\n", (int)xPortGetFreeHeapSize());

    printf("(Free himem size: %d)\n", (int)esp_himem_get_free_size());


    static httpd_handle_t server = NULL;
   
    init_camera();
    initialize_in_serial();

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, &server));
    //    ESP_ERROR_CHECK(esp_event_loop_init(event_handler,&server ));
    wifi_prov_mgr();

    #ifdef CONFIG_UDP_LOG_SUPPORT
      udp_logging_init( CONFIG_LOG_UDP_IP, CONFIG_LOG_UDP_PORT, udp_logging_vprintf, grbl_friendly_logging_vprintf);
    #endif 


    printf("(Free heap size: %d)\n", (int)xPortGetFreeHeapSize());

    ssdtest();
    printf("(Free heap size: %d)\n", (int)xPortGetFreeHeapSize());


    ESP_ERROR_CHECK(initHID());

    infoDisplay();


#ifdef CONFIG_TCP_SERIAL_SUPPORT
    ESP_ERROR_CHECK(startTcpSerial());
#endif

#ifdef CONFIG_BLUETOOTH_SERIAL_SUPPORT
    app_main_bluetooth();
    printf("(Free heap size: %d)\n", (int)xPortGetFreeHeapSize());
#endif

}