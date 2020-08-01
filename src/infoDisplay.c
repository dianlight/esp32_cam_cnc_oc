#include "pinConfig.h"

#include <esp_log.h>
#include <string.h>

#include <u8g2.h>
#include <u8g2_esp32_hal.h>

#include "infoDisplay.h"
#include "hid.h"
#include <i2cdev.h>

#define I2C_FREQ_HZ 1000000

static const char *TAG = "display";

// IconBar On/Op1 | Op2 | Op2 | Op3 | Op4 | Op5 | Op6 | Op7 | Op8 | Op9 | Off  Icon
static const uint16_t IconBar[16][10] = {
    {0x00DF, 0x00EE, 0x00E9, 0x00D2, 0x00E5, 0x0059, 0x0134, 0x00CB, 0x0118, 0x0000}, //  GRBL_SLEEP | GRBL_IDLE | GRBL_RUN | GRBL_HOLD | GRBL_JOG | GRBL_HOME | GRBL_CHECK | GRBL_DOOR | GRBL_ALARM
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
static info_display_handle_t info_display_handle;

void info_display_task(void *params);

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
        ESP_LOGD(TAG,"INC pressed!");
        break;
    case HID_EVENT_DEC:
        ESP_LOGD(TAG,"DEC pressed!");
        break;
    case HID_EVENT_SEL:
        ESP_LOGD(TAG,"SEL pressed!");
        break;
    case HID_EVENT_JOY_BUTTON:
        ESP_LOGD(TAG,"JOY pressed!");
        break;
    case HID_EVENT_JOY_MOVE:
        ESP_LOGD(TAG,"JOY move!");
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
//            CHECK_ARG(&device);
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
 //           I2C_DEV_TAKE_MUTEX(&device);
 //           I2C_DEV_GIVE_MUTEX(&device);            
			break;
		}

		case U8X8_MSG_BYTE_START_TRANSFER: {
		    uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
            buffer = malloc(4096);
            bufferpos = 0;
            ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address>>1);
            /*
			handle_i2c = i2c_cmd_link_create();
			ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address>>1);
			ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
			ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
            */
			break;
		}

		case U8X8_MSG_BYTE_END_TRANSFER: {
			ESP_LOGD(TAG, "End I2C transfer.");
            /*
			ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
			ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, handle_i2c, I2C_TIMEOUT_MS / portTICK_RATE_MS));
			i2c_cmd_link_delete(handle_i2c);
            */
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
//uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
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
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    info_display_handle.page = DISPLAY_BOOT_PAGE;

    ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID, &hid_event_handler, NULL));

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
    u8g2_DrawStr(&u8g2, 2, 17, "GRBL v0.028");

}

void _mainPage(info_display_handle_t *data)
{

    // GRBL <Idle|MPos:0.000,0.000,0.000|FS:0.0,0>
    // Icon Bar 16x16
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    for (int pi = 0; pi < 15; pi++)
    {
        if (IconBar[pi][0] != 0x0000)
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][0]);
    }
    // Separator Line
    u8g2_DrawLine(&u8g2, 0, 9, 128, 9);
    // Status Infos
    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    u8g2_DrawStr(&u8g2, 0, 9 + 12, "x000.0 y000.0 z000.0");
    u8g2_DrawStr(&u8g2, 0, 9 + 12 * 2, "Feed 9999 Rpm 10000 ");
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
    u8g2_DrawStr(&u8g2, 102, 26+12, perc);

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
