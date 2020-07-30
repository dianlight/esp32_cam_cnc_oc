#include "pinConfig.h"

#include <esp_log.h>

#include <u8g2.h>
#include <u8g2_esp32_hal.h>

#include "infoDisplay.h"

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

esp_err_t initDisplay(void)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = SDA_GPIO;
    u8g2_esp32_hal.scl = SCL_GPIO;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    /*    
    u8g2_Setup_ssd1309_i2c_128x64_noname2_2(
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
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    info_display_handle.page = DISPLAY_BOOT_PAGE;

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
    u8g2_DrawStr(&u8g2, 2, 17, "GRBL v0.025");

}

void _mainPage(info_display_handle_t *data)
{
    static uint8_t menupos = 0;

    // GRBL <Idle|MPos:0.000,0.000,0.000|FS:0.0,0>
    // Icon Bar 16x16
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    for (int pi = 0; pi < 15; pi++)
    {
        if (IconBar[pi][0] != 0x0000)
            u8g2_DrawGlyph(&u8g2, pi * 8, 8, IconBar[pi][0]);
    }
    // Separator Line
    u8g2_DrawLine(&u8g2, 0, 8, 128, 8);
    // Status Infos
    u8g2_SetFont(&u8g2, u8g2_font_6x13_me);
    u8g2_DrawStr(&u8g2, 0, 8 + 13, "Ps 000.0 000.0 000.0");
    u8g2_DrawStr(&u8g2, 0, 8 + 13 * 2, "Feed 9999 Rpm 10000 ");
    // X 0.000 Y 0.000 Z 0.000
    // Feed 500 Speed 8000
    // Line number: 9999999
    // Buffer Bar (Bf:)
    u8g2_DrawStr(&u8g2, 0, 8 + 13 * 3, "Buff:");
    u8g2_DrawBox(&u8g2, 26, 48 - 8, 80, 6);
    u8g2_DrawFrame(&u8g2, 26, 48 - 8, 100, 6);
    // Separator Line
    //u8g2_DrawLine(&u8g2,0,48,128,48);
    // IconMenu Bar 32x32
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);
    for (int pi = 0; pi < 7; pi++)
    {
        if (MenuIcon[pi] == 0x00)
            continue;
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
        vTaskMissedYield();
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
