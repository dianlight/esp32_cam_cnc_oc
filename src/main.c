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
 *  [ ] Wifi Socket serial
 *  [ ] Icon Menu
 *  [x] OTA
 *    [ ] Use Himem for ota
 *    [x] Correct watchdog conflict
 *    [x] Ota events!
 *  [x] mDNS
 * GRBL TODO:
 *  [ ] Log out of serial! // GRBL compatible prefix?
 *  [ ] Display status
 *  [ ] Manual command
 *  [ ] Probe command
*/
#include "pinConfig.h"
#include <stdio.h>
#include <dirent.h>
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
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <i2cdev.h>
#include <mcp23x17.h>
#include <ads111x.h>
#include <mdns.h>

#include "infoDisplay.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#ifdef CONFIG_BT_ENABLED
#include "bluetooth.h"
#endif
#include "cam.h"

#include "wifi_prov_mgr.h"

#include "idf_arduino_ota.h"


#define MOUNT_POINT "/sdcard"

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

        start_arduino_ota();
        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_STARTED, &ota_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_END, &ota_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_PROGRESS, &ota_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, OTA_EVENT_ERROR, &ota_event_handler, NULL));

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


void buttonsTask(void *pvParameters)
{
    mcp23x17_t dev;
    memset(&dev, 0, sizeof(mcp23x17_t));
    ESP_ERROR_CHECK(mcp23x17_init_desc(&dev, 0, MCP23X17_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup PORTA0 as input
    for (u8_t p = 0; p < 8; p++)
    {
        mcp23x17_set_mode(&dev, p, MCP23X17_GPIO_INPUT);
    }
    mcp23x17_port_set_pullup(&dev, 0x00FF);

    /*         
    // Setup interrupt on it
    mcp23x17_set_interrupt(&dev, 0, MCP23X17_INT_ANY_EDGE);

    gpio_set_direction(INTA_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(INTA_GPIO, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTA_GPIO, intr_handler, NULL);

    // Setup PORTB0 as output
    mcp23x17_set_mode(&dev, 8, MCP23X17_GPIO_OUTPUT);
    // do some blinkning
    bool on = true;
    while (1)
    {
        mcp23x17_set_level(&dev, 8, on);
        on = !on;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
*/

    i2c_dev_t device;
    memset(&device, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ads111x_init_desc(&device, ADS111X_ADDR_GND, 0, SDA_GPIO, SCL_GPIO));

    while (1)
    {
        uint32_t val = 0;
        for (u8_t p = 0; p < 8; p++)
        {
            if (mcp23x17_get_level(&dev, p, &val) == ESP_OK)
            {
                printf("%c", !val ? '1' : '0');
                if (p == 0 && !val)
                {
                    printf("\nFree heap size: %d\n", (int)xPortGetFreeHeapSize());
                }
            }
            else
                printf("*");
        }
        /*
        printf("%c%c%c%c%c%c%c%c ",
                  !mcp23x17_get_level(&dev,0,true) ? 'J' : '0',  // JOYSTICK 
                  !mcp23x17_get_level(&dev,1,true) ? '2' : '0', 
                  !mcp23x17_get_level(&dev,2,true) ? '3' : '0',
                  !mcp23x17_get_level(&dev,3,true) ? 'N' : '0',  // NEXT
                  !mcp23x17_get_level(&dev,4,true) ? 'B' : '0',  // BEFORE
                  !mcp23x17_get_level(&dev,5,true) ? 'I' : '0',  // INC
                  !mcp23x17_get_level(&dev,6,true) ? 'D' : '0',  // DEC 
                  !mcp23x17_get_level(&dev,7,true) ? 'S' : '0'); // SEL
        */
        ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_0_GND)); // positive = AIN0, negative = GND
        ads111x_set_comp_queue(&device, ADS111X_COMP_QUEUE_DISABLED);
        ads111x_set_comp_latch(&device, ADS111X_COMP_LATCH_DISABLED);
        ads111x_set_comp_polarity(&device, ADS111X_COMP_POLARITY_LOW);
        ads111x_set_comp_mode(&device, ADS111X_COMP_MODE_NORMAL);
        ads111x_set_data_rate(&device, ADS111X_DATA_RATE_128);
        ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT);
        ESP_ERROR_CHECK(ads111x_set_gain(&device, ADS111X_GAIN_4V096));

        vTaskDelay(10 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ads111x_start_conversion(&device));
        bool busy = true;
        do
        {
            ESP_ERROR_CHECK(ads111x_is_busy(&device, &busy));
        } while (busy);

        int16_t raw = 0;
        if (ads111x_get_value(&device, &raw) == ESP_OK)
        {
            //    float voltage = ads111x_gain_values[ADS111X_GAIN_4V096] / ADS111X_MAX_VALUE * raw;
            //    printf("Raw ADC value: %d, voltage: %.04f volts\n", raw, voltage);
            printf("Raw ADC value: %d,", raw);
        }
        else
            printf("Cannot read ADC value,");

        vTaskDelay(10 / portTICK_PERIOD_MS);

        busy = true;
        ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_1_GND)); // positive = AIN0, negative = GND
        ads111x_set_comp_queue(&device, ADS111X_COMP_QUEUE_DISABLED);
        ads111x_set_comp_latch(&device, ADS111X_COMP_LATCH_DISABLED);
        ads111x_set_comp_polarity(&device, ADS111X_COMP_POLARITY_LOW);
        ads111x_set_comp_mode(&device, ADS111X_COMP_MODE_NORMAL);
        ads111x_set_data_rate(&device, ADS111X_DATA_RATE_128);
        ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT);
        ESP_ERROR_CHECK(ads111x_set_gain(&device, ADS111X_GAIN_4V096));
        ESP_ERROR_CHECK(ads111x_start_conversion(&device));
        do
        {
            ESP_ERROR_CHECK(ads111x_is_busy(&device, &busy));
        } while (busy);

        if (ads111x_get_value(&device, &raw) == ESP_OK)
        {
            //    float voltage = ads111x_gain_values[ADS111X_GAIN_4V096] / ADS111X_MAX_VALUE * raw;
            //    printf("Raw ADC value: %d, voltage: %.04f volts\n", raw, voltage);
            printf("Raw ADC value: %d\n", raw);
        }
        else
            printf("Cannot read ADC value\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// SSD READER
void ssdtest(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    /*
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes
    */

    esp_err_t ret;
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Stuff
    ESP_LOGI(TAG, "Listing root directory: \n");

    DIR *d;
    struct dirent *dir;
    d = opendir(MOUNT_POINT "/");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            printf("%s\n", dir->d_name);
        }
        closedir(d);
    }

    // All done, unmount partition and disable SDMMC or SPI peripheral
    //    esp_vfs_fat_sdcard_unmount(mount_point, card);
    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGI(TAG, "Card unmounted");
}

// Main
void app_main(void)
{

    esp_log_level_set("*", ESP_LOG_DEBUG); // set all components to ERROR level
    esp_log_level_set("spiram", ESP_LOG_WARN);
    esp_log_level_set("efuse", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);                 
    esp_log_level_set("wpa", ESP_LOG_WARN);                   
    esp_log_level_set("httpd_parse", ESP_LOG_INFO);          
    esp_log_level_set("camera", ESP_LOG_INFO);               
    esp_log_level_set("cam", ESP_LOG_DEBUG);               
    esp_log_level_set("mainapp", ESP_LOG_DEBUG); 
    esp_log_level_set("ota_server",ESP_LOG_DEBUG);
    esp_log_level_set("wifi_prov_mgr",ESP_LOG_INFO);

    ESP_LOGI(TAG,"Application Start");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(initDisplay());
//    bootDisplay();

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", (spi_flash_get_chip_size()+esp_himem_get_phys_size()) / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());

    printf("Free heap size: %d\n", (int)xPortGetFreeHeapSize());

    printf("Free himem size: %d\n", (int)esp_himem_get_free_size());


    static httpd_handle_t server = NULL;
   
    init_camera();

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, &server));
    //    ESP_ERROR_CHECK(esp_event_loop_init(event_handler,&server ));
    wifi_prov_mgr();
    //initialise_wifi(&server);

    printf("Free heap size: %d\n", (int)xPortGetFreeHeapSize());

    ssdtest();
    printf("Free heap size: %d\n", (int)xPortGetFreeHeapSize());

#ifdef CONFIG_BT_ENABLED
    app_main_bluetooth();
    printf("Free heap size: %d\n", (int)xPortGetFreeHeapSize());
#endif

    ESP_ERROR_CHECK(i2cdev_init());
    // xTaskCreate(buttonsTask, "buttonsTask", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
 //   xTaskCreatePinnedToCore(buttonsTask, "buttonsTask", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL, APP_CPU_NUM);

//    ESP_ERROR_CHECK(startInfoDisplay());
    infoDisplay();
}