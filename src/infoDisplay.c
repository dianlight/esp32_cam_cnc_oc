#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include "pinConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include <u8g2.h>

#include "infoDisplay.h"
#include "hid.h"
#include <i2cdev.h>

#define I2C_FREQ_HZ 1000000

static const char *TAG = "idisplay";

enum {
    ICON_STATUS = 0,
    ICON_WEBCAM = 7,
    ICON_TCP_SERIAL = 12,
    ICON_BLUTOOTH_SERIAL = 13,
    ICON_WIFI = 14
};

// IconBar On/Op1 | Op2 | Op2 | Op3 | Op4 | Op5 | Op6 | Op7 | Op8 | Op9 | Op10 | Off  Icon
static const uint16_t IconBar[15][11] = {
    {0x00BC, 0x00DF, 0x00EE, 0x00E9, 0x00D2, 0x00E5, 0x0059, 0x0134, 0x00CB, 0x0118, 0x0000}, // GRBL_UNKNOWN | GRBL_SLEEP | GRBL_IDLE | GRBL_RUN | GRBL_HOLD | GRBL_JOG | GRBL_HOME | GRBL_CHECK | GRBL_DOOR | GRBL_ALARM | 
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0114, 0x0000},                                                                 // Webcam Streeming
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x00DE, 0x0000},                                                                 // TCP/IP (Serial)
    {0x005E, 0x0000},                                                                 // Blutooth (Serial)
    {0x00F7, 0x0000},                                                                 // WiFi
};

static const uint16_t MenuIcon[7] = {
    0x00F3, // Cycle start
    0x00E0, // Free hold
    0x0082, // Probe
    0x0000, //
    0x00AB, // SD/File
    0x0000, //
    0x0081, // Config
};

u8g2_t u8g2; // a structure which will contain all the data for one display
info_display_handle_t info_display_handle;

void info_display_task(void *params);
void request_status_task(void *params);

static uint8_t menupos = 0; 

static void hid_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    hid_status_t *hid_status = (hid_status_t*)event_data;
    switch (event_id)
    {
    case HID_EVENT_BEFORE:
        if(menupos > 0)menupos--;
        else menupos = 7;
        break;
    case HID_EVENT_NEXT:
        menupos=(menupos+1)%7;
        break;
    case HID_EVENT_INC:
        ESP_LOGI(TAG,"INC pressed!");
        break;
    case HID_EVENT_DEC:
        ESP_LOGI(TAG,"DEC pressed!");
        break;
    case HID_EVENT_SEL:
        ESP_LOGI(TAG,"SEL pressed!");
        break;
    case HID_EVENT_JOY_BUTTON:
        ESP_LOGI(TAG,"JOY pressed!");
        break;
    case HID_EVENT_JOY_MOVE:
        ESP_LOGI(TAG,"JOY move! %d %d",hid_status->dx,hid_status->dy);
        break;
    default:
        break;
    }
}

/**
 * HAL Function for ESP32 using i2cdev
 */
uint8_t u8g2_i2cdev_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){
	ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

    static i2c_dev_t device;
    static uint8_t *buffer; 
    static uint16_t bufferpos =0;

	switch(msg) {
		case U8X8_MSG_BYTE_SET_DC: {
			break;
		}

		case U8X8_MSG_BYTE_INIT: {
            uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
            ESP_LOGD(TAG,"Request init of i2c device %02X.", i2c_address>>1);
            memset(&device, 0, sizeof(i2c_dev_t));
            device.port = 0;
            device.addr = i2c_address >> 1;
            device.cfg.sda_io_num = SDA_GPIO;
            device.cfg.scl_io_num = SCL_GPIO;
            device.cfg.master.clk_speed = I2C_FREQ_HZ;
            ESP_ERROR_CHECK(i2c_dev_create_mutex(&device));
            ESP_LOGD(TAG,"Done init of i2c device");
			break;
		}

		case U8X8_MSG_BYTE_SEND: {
            ESP_LOGD(TAG,"Send %d bytes to i2c",arg_int);
			uint8_t* data_ptr = (uint8_t*)arg_ptr;
			ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);
            memcpy(&buffer[bufferpos],arg_ptr,arg_int);
            bufferpos+=arg_int;
			break;
		}

		case U8X8_MSG_BYTE_START_TRANSFER: {
		    uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
            buffer = malloc(4096);
            bufferpos = 0;
            ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address>>1);
			break;
		}

		case U8X8_MSG_BYTE_END_TRANSFER: {
			ESP_LOGD(TAG, "End I2C transfer.");
            I2C_DEV_TAKE_MUTEX(&device);
            I2C_DEV_CHECK(&device, i2c_dev_write(&device,NULL,0,buffer, bufferpos));
            I2C_DEV_GIVE_MUTEX(&device);  
            free(buffer);   
            bufferpos=0;       
			break;
		}
	}
	return 0;
}

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is invoked
 * to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

	switch(msg) {
	// Initialize the GPIO and DELAY HAL functions.  If the pins for DC and RESET have been
	// specified then we define those pins as GPIO outputs.
		case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            /*
			uint64_t bitmask = 0;
			if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
				bitmask = bitmask | (1ull<<u8g2_esp32_hal.dc);
			}
			if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
				bitmask = bitmask | (1ull<<u8g2_esp32_hal.reset);
			}
			if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
				bitmask = bitmask | (1ull<<u8g2_esp32_hal.cs);
			}

            if (bitmask==0) {
            	break;
            }
			gpio_config_t gpioConfig;
			gpioConfig.pin_bit_mask = bitmask;
			gpioConfig.mode         = GPIO_MODE_OUTPUT;
			gpioConfig.pull_up_en   = GPIO_PULLUP_DISABLE;
			gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
			gpioConfig.intr_type    = GPIO_INTR_DISABLE;
			gpio_config(&gpioConfig);
            */
			break;
		}

	// Set the GPIO reset pin to the value passed in through arg_int.
		case U8X8_MSG_GPIO_RESET:
        /*
			if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
				gpio_set_level(u8g2_esp32_hal.reset, arg_int);
			}
            */
			break;
	// Set the GPIO client select pin to the value passed in through arg_int.
		case U8X8_MSG_GPIO_CS:
        /*
			if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
				gpio_set_level(u8g2_esp32_hal.cs, arg_int);
			}
            */
			break;
	// Set the Software I²C pin to the value passed in through arg_int.
		case U8X8_MSG_GPIO_I2C_CLOCK:
        /*
			if (u8g2_esp32_hal.scl != U8G2_ESP32_HAL_UNDEFINED) {
				gpio_set_level(u8g2_esp32_hal.scl, arg_int);
//				printf("%c",(arg_int==1?'C':'c'));
			}
            */
			break;
	// Set the Software I²C pin to the value passed in through arg_int.
		case U8X8_MSG_GPIO_I2C_DATA:
        /*
			if (u8g2_esp32_hal.sda != U8G2_ESP32_HAL_UNDEFINED) {
				gpio_set_level(u8g2_esp32_hal.sda, arg_int);
//				printf("%c",(arg_int==1?'D':'d'));
			}
        */    
			break;

	// Delay for the number of milliseconds passed in through arg_int.
		case U8X8_MSG_DELAY_MILLI:
			vTaskDelay(arg_int/portTICK_PERIOD_MS);
			break;
	}
	return 0;
} 

/**
 * END
 */



esp_err_t initDisplay(void)
{
    /* u8g2_esp32_hal solution * /
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = SDA_GPIO;
    u8g2_esp32_hal.scl = SCL_GPIO;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    / **/

    /* i2cdev solution */
    i2c_dev_t device;
    memset(&device, 0, sizeof(i2c_dev_t));

    device.port = 0;
    device.addr = 0x78;
    device.cfg.sda_io_num = SDA_GPIO;
    device.cfg.scl_io_num = SCL_GPIO;
    ESP_ERROR_CHECK(i2c_dev_create_mutex(&device));

    /*    
    u8g2_Setup_ssd1309_i2c_128x64_noname2_2(
        //	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        //u8x8_byte_sw_i2c,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
*/
/*
    u8g2_Setup_ssd1309_i2c_128x64_noname2_f(
        //	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        //u8x8_byte_sw_i2c,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
*/        

    u8g2_Setup_ssd1309_i2c_128x64_noname2_f(
        //	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        //u8x8_byte_sw_i2c,
        u8g2_i2cdev_byte_cb,
        u8g2_gpio_and_delay_cb); // init u8g2 structure

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    info_display_handle.page = DISPLAY_BOOT_PAGE;

    ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID, &hid_event_handler, NULL));

    xTaskCreate(request_status_task, "request_status_task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    if (xTaskCreate(info_display_task, "infoDisplay", configMINIMAL_STACK_SIZE * 6, (void *)&info_display_handle, 1, &info_display_handle.task) == pdPASS)
    {
        ESP_LOGD(TAG, "Create InfoDisplay task");
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

void infoDisplay(void){
    info_display_handle.page = DISPLAY_MAIN_PAGE;
}

void otaDisplay(uint8_t perc)
{
    info_display_handle.page = DISPLAY_OTA_PAGE;
    info_display_handle.percentual = perc;
}

void _bootPage(info_display_handle_t *data)
{
    u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 2, 17, "GRBL v0.031");

}

void _mainPage(info_display_handle_t *data)
{

    // GRBL <Idle|MPos:0.000,0.000,0.000|FS:0.0,0>
    // Icon Bar 16x16
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    for (int pi = 0; pi < 15; pi++)
    {
        switch (pi)
        {
        case ICON_STATUS:
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][info_display_handle.status]);
            break;
        case ICON_WIFI:
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][info_display_handle.wifi?0:1]);
            break;
        case ICON_TCP_SERIAL:
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][info_display_handle.tcp_serial?0:1]);
            break;
        case ICON_BLUTOOTH_SERIAL:
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][info_display_handle.bluetooth_serial?0:1]);
            break;
        case ICON_WEBCAM:
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][info_display_handle.webcam?0:1]);
            break;
        default:
            if (IconBar[pi][0] != 0x0000)
                u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][0]);
            break;
        }
    }
    // Separator Line
    u8g2_DrawLine(&u8g2, 0, 9, 128, 9);
    // Status Infos
    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    char dispStr[22];
    sprintf(&dispStr[0],"x%3.1f y%3.1f z%3.1f",info_display_handle.x,info_display_handle.y,info_display_handle.z);
    u8g2_DrawStr(&u8g2, 0, 9 + 12, dispStr);
    sprintf(&dispStr[0],"Feed %3d Rpm %5d",info_display_handle.fr,info_display_handle.speed);
    u8g2_DrawStr(&u8g2, 0, 9 + 12 * 2,dispStr);
    // X 0.000 Y 0.000 Z 0.000
    // Feed 500 Speed 8000
    // Line number: 9999999
    // Buffer Bar (Bf:)
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(&u8g2,0,9 + 12 * 3, 0x10E);
   // u8g2_DrawStr(&u8g2, 0, 9 + 12 * 3, "Buff");
    u8g2_DrawBox(&u8g2, 18, 48 - 10, 80, 6);
    u8g2_DrawFrame(&u8g2, 18, 48 - 10, 100, 6);
    // Separator Line
    //u8g2_DrawLine(&u8g2,0,48,128,48);
    // IconMenu Bar 32x32
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);
    for (int pi = 0; pi < 7; pi++)
    {
        if (MenuIcon[pi] == 0x00){
            if(pi == menupos)menupos=(menupos+1)%7;;
            continue;
        }
        u8g2_DrawGlyph(&u8g2, pi * 18, 64, MenuIcon[pi]);
        if (menupos == pi)
        {
            u8g2_SetDrawColor(&u8g2, 2);
            u8g2_DrawBox(&u8g2, pi * 18, 64 - 18, 18, 18);
            u8g2_SetDrawColor(&u8g2, 1);
        }
    }
}

void _otaPage(info_display_handle_t *data)
{
    u8g2_DrawBox(&u8g2, 0, 26, data->percentual, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    char perc[5];
    sprintf(perc,"%d%%",data->percentual);
    u8g2_DrawStr(&u8g2, 102, 26+6, perc);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 2, 17, "OTA update");
}

void info_display_task(void *params)
{
    info_display_handle_t *data = (info_display_handle_t *)params;
    while (true)
    {
        u8g2_ClearBuffer(&u8g2);
        switch (data->page)
        {
        case DISPLAY_BOOT_PAGE:
            _bootPage(data);
            break;
        case DISPLAY_MAIN_PAGE:
            _mainPage(data);
            break;
        case DISPLAY_OTA_PAGE:
            _otaPage(data);
            break;
        default:
            ESP_LOGW(TAG, "No page to display!");
            break;
        }
//        vTaskMissedYield();
        u8g2_SendBuffer(&u8g2);
        vTaskDelay(500 / portTICK_PERIOD_MS); // 500ms refresh for display
    }
}

void request_status_task(void *params){
    while(1){
        uint32_t current = (unsigned long) (esp_timer_get_time() / 1000ULL);
        if(current - info_display_handle.lastStatusUpdate > 500){
            printf("?\n");
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // 2Hz refresh for info request
    }
}

/*

esp_err_t startInfoDisplay()
{
    if (info_display_handle.task == NULL)
    {
        if (xTaskCreate(info_display_task, "infoDisplay", configMINIMAL_STACK_SIZE * 6, (void *)&info_display_handle, 1, &info_display_handle.task) == pdPASS)
        {
            ESP_LOGD(TAG, "Create InfoDisplay task");
            return ESP_OK;
        }
        else
        {
            return ESP_FAIL;
        }
    }
    else
    {
        vTaskResume(&info_display_handle.task);
        return ESP_OK;
    }
}

void stopInfoDisplay()
{
    ESP_LOGD(TAG, "Suspend InfoDisplay task");
    if (info_display_handle.task != NULL)
    {
        //        vTaskDelete(info_display_handle.task);
        //        info_display_handle.task = NULL;
        if (eTaskGetState(&info_display_handle.task) == eRunning)
            vTaskSuspend(&info_display_handle.task);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
*/
