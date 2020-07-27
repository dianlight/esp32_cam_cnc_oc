/* Hello World Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include <esp_camera.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <u8g2.h>
#include <u8g2_esp32_hal.h>


static const char *TAG = "example:take_picture";

static camera_config_t camera_config = {
    .pin_pwdn = CONFIG_PWDN,
    .pin_reset = CONFIG_RESET,
    .pin_xclk = CONFIG_XCLK,
    .pin_sscb_sda = CONFIG_SDA,
    .pin_sscb_scl = CONFIG_SCL,

    .pin_d7 = CONFIG_D7,
    .pin_d6 = CONFIG_D6,
    .pin_d5 = CONFIG_D5,
    .pin_d4 = CONFIG_D4,
    .pin_d3 = CONFIG_D3,
    .pin_d2 = CONFIG_D2,
    .pin_d1 = CONFIG_D1,
    .pin_d0 = CONFIG_D0,
    .pin_vsync = CONFIG_VSYNC,
    .pin_href = CONFIG_HREF,
    .pin_pclk = CONFIG_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = CONFIG_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,   //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

// Camera
static esp_err_t init_camera()
{
//    //power up the camera if PWDN pin is defined
//    if(CAM_PIN_PWDN != -1){
//        gpio_pad_select_gpio(CAM_PIN_PWDN);
//        gpio_set_direction(CAM_PIN_PWDN,GPIO_MODE_OUTPUT);
//        gpio_set_level(CAM_PIN_PWDN, 0);
//    }

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed 0x%x",err);
        return err;
    }

    return ESP_OK;
}


// HTTP 
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }
        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
            (uint32_t)(_jpg_buf_len/1024),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    }

    last_frame = 0;
    return res;
}

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
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK)
  {
    // Set URI handlers
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

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  httpd_handle_t *server = (httpd_handle_t *)ctx;

  switch (event->event_id)
  {
  case SYSTEM_EVENT_STA_START:
    ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
    ESP_ERROR_CHECK(esp_wifi_connect());
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
    ESP_LOGI(TAG, "Got IP: '%s'",
             ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));

    /* Start the web server */
    if (*server == NULL)
    {
      *server = start_webserver();
    }
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
    ESP_ERROR_CHECK(esp_wifi_connect());

    /* Stop the web server */
    if (*server)
    {
      stop_webserver(*server);
      *server = NULL;
    }
    break;
  default:
    break;
  }
  return ESP_OK;
}

static void initialise_wifi(void *arg)
{
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, arg));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  wifi_config_t wifi_config = {
      .sta = {
          .ssid = CONFIG_WIFI_SSID,
          .password = CONFIG_WIFI_PASSWORD,
      },
  };
  
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

//U8G2_SSD1309_128X64_NONAME2_2_HW_I2C u8g2(U8G2_R0, /*reset*/ U8X8_PIN_NONE, /* clock */ 12, /* data */ 13);

// Main
void app_main(void)
{

    esp_log_level_set("*", ESP_LOG_WARN);        // set all components to ERROR level
    esp_log_level_set("spiram", ESP_LOG_WARN);
    esp_log_level_set("efuse", ESP_LOG_WARN);
    esp_log_level_set("wifi*", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
    esp_log_level_set("wpa", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
    esp_log_level_set("httpd_parse", ESP_LOG_INFO);     // enable INFO logs from DHCP client
    esp_log_level_set("camera", ESP_LOG_DEBUG);     // enable INFO logs from DHCP client
    esp_log_level_set("example:take_picture", ESP_LOG_DEBUG);     // enable INFO logs from DHCP client

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());

    printf("Free heap size: %d\n", (int) xPortGetFreeHeapSize());
//    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

 
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    init_camera();
    initialise_wifi(&server);



    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = 13; //PIN_SDA;
	u8g2_esp32_hal.scl  = 12; //PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1309_i2c_128x64_noname2_2(
//	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
    u8g2_FirstPage(&u8g2);
    do {
        ESP_LOGI(TAG, "u8g2_DrawBox");
        u8g2_DrawBox(&u8g2, 0, 26, 80,6);
        u8g2_DrawFrame(&u8g2, 0,26,100,6);

        ESP_LOGI(TAG, "u8g2_SetFont");
        u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
        ESP_LOGI(TAG, "u8g2_DrawStr");
        u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
    } while (u8g2_NextPage(&u8g2));
//	ESP_LOGI(TAG, "u8g2_SendBuffer");
//	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");
    printf("Free heap size: %d\n", (int) xPortGetFreeHeapSize());

/*

    u8g2_t u8g2;
    uint8_t *buf;
    u8g2_Setup_ssd1309_i2c_128x64_noname2_2(&u8g2,U8G2_R0,  u8x8_byte_sw_i2c, u8g2_esp32_gpio_and_delay_cb );
    // / *reset* / U8X8_PIN_NONE, / * clock * / 12, / * data * / 13);

    u8g2_Setup_ssd1309_i2c_128x64_noname2_2(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_lpc11u3x);  // init u8g2 structure
    buf = (uint8_t *)malloc(u8g2_GetBufferSize(&u8g2)); // dynamically allocate a buffer of the required size
    u8g2_SetBufferPtr(&u8g2, buf); // set the internal page buffer pointer to the newly allocated page buffer
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);


    u8g2.begin();

     u8g2.firstPage();
    do
    {
      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.drawStr(0, 14, "Hello World!");
/ *
      u8g2.setCursor(0, 26);
      u8g2.printf("%c%c%c%c%c%c%c%c.",
                  !mcp.digitalRead(0) ? 'J' : '0',  // JOYSTICK 
                  !mcp.digitalRead(1) ? '2' : '0', 
                  !mcp.digitalRead(2) ? '3' : '0',
                  !mcp.digitalRead(3) ? 'N' : '0',  // NEXT
                  !mcp.digitalRead(4) ? 'B' : '0',  // BEFORE
                  !mcp.digitalRead(5) ? 'I' : '0',  // INC
                  !mcp.digitalRead(6) ? 'D' : '0',  // DEC 
                  !mcp.digitalRead(7) ? 'S' : '0'); // SEL
      u8g2.setCursor(0, 38);
      u8g2.printf("%d %d",
                  ads.readADC_SingleEnded(0),
                  ads.readADC_SingleEnded(1));

      u8g2.setCursor(0, 50);
      // u8g2.printf("%ld",mcp.readGPIOAB());
* /
    } while (u8g2.nextPage());
*/
/*
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
*/    
}