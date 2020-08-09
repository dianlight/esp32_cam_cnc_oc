#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "pinConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <i2cdev.h>
#include "hal/u8g2_i2c_hal.h"
#include <u8g2.h>

#include "infoDisplay.h"
#include "hid.h"
#include "jog.h"


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
    {0x0114, 0x0000},                                                                 // Webcam Streaming
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x0000, 0x0000},                                                                 //
    {0x00DE, 0x0000},                                                                 // TCP/IP (Serial)
    {0x005E, 0x0000},                                                                 // Blutooth (Serial)
    {0x00F7, 0x0000},                                                                 // WiFi
};

enum {
    MENU_CYCLE  = 0,
    MENU_JOG    = 1,
    MENU_PROBE  = 2,
    MENU_SSD    = 4,
    MENU_CONFIG = 6
};

static const uint16_t MenuIcon[7] = {
    0x00F3, // Cycle start
    0x00E0, // Jog
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

static uint8_t menupos = 0, selmenupos = 0; 
static int8_t selmenu_dir = 0; 


static void hid_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    info_display_handle_t *data = (info_display_handle_t *)handler_args;

    // Return on event!
//    if(info_display_handle.return_page != DISPLAY_NO_PAGE ){
//        info_display_handle.page = info_display_handle.return_page;
//        info_display_handle.return_page = DISPLAY_NO_PAGE;
//        info_display_handle.page_timeout = 0;
//    }

    hid_status_t *hid_status = (hid_status_t*)event_data;
    switch (event_id)
    {
    case HID_EVENT_BEFORE:
        selmenu_dir = -1;
        if(selmenupos+selmenu_dir < 0)selmenupos = 6;
        else selmenupos = (selmenupos + selmenu_dir) % 7;
        break;
    case HID_EVENT_NEXT:
        selmenu_dir = 1;
        selmenupos = (selmenupos + selmenu_dir) % 7;
        break;
    case HID_EVENT_INC:
        ESP_LOGI(TAG,"INC pressed!");
        if(data->page == DISPLAY_JOY_CALIBRATION_PAGE){
            hid_status->joy_ss +=5;
            data->js = hid_status->joy_ss;
            data->step = 0;
            commit_calibration(hid_status);
            ESP_LOGD(TAG,"INC joy ss %d",hid_status->joy_ss);
        }
        break;
    case HID_EVENT_DEC:
        ESP_LOGI(TAG,"DEC pressed!");
        if(data->page == DISPLAY_JOY_CALIBRATION_PAGE){
            hid_status->joy_ss -=5;
            data->js = hid_status->joy_ss;
            data->step = 0;
            commit_calibration(hid_status);
            ESP_LOGD(TAG,"DEC joy ss %d",hid_status->joy_ss);
        }
        break;
    case HID_EVENT_SEL:
        ESP_LOGD(TAG,"SEL pressed!");
        if(selmenupos != menupos){
            if(menupos == MENU_JOG) stopJog();
            if(selmenupos == MENU_JOG){
                if(startJog()){
                    menupos = selmenupos;
                    selmenu_dir = 0;
                }
            } else if (selmenupos == MENU_CONFIG){
                menupos = selmenupos;
                selmenu_dir = 0;
                joyCalibrationDisplay();
            } else {
                menupos = selmenupos;
                selmenu_dir = 0;
            }
        }
        break;
    case HID_EVENT_JOY_BUTTON:
        ESP_LOGI(TAG,"JOY pressed!");
        if(data->page == DISPLAY_JOY_CALIBRATION_PAGE){
            ESP_LOGD(TAG,"Exec Step %d",data->step);
            switch(data->step){
                case 0: // Center
                    hid_status->cx = hid_status->x;
                    hid_status->cy = hid_status->y;
                    data->step++;
                    commit_calibration(hid_status);
                break;
                case 1: // Max
                    hid_status->max_x = hid_status->x;
                    hid_status->max_y = hid_status->y;
                    data->step++;
                    commit_calibration(hid_status);
                break;
                case 2:
                    ESP_LOGD(TAG,"End wizard!");
                    saveCalibration();
                    
                    data->page = data->return_page;
                    ESP_ERROR_CHECK_WITHOUT_ABORT(stopJoytickHID());
                break;
            }
        }
        break;
    case HID_EVENT_JOY_MOVE:
        ESP_LOGI(TAG,"JOY move! %d(%d) %d(%d)",hid_status->x,hid_status->dx,hid_status->y,hid_status->dy);
        if(data->page == DISPLAY_JOY_CALIBRATION_PAGE){
            if(data->step == 0){
                data->jx = 128 - ( hid_status->x *128 / (UINT16_MAX / hid_status->joy_ss));
                data->jy = hid_status->y *64 / (UINT16_MAX / hid_status->joy_ss);
                data->js = hid_status->joy_ss;
                ESP_LOGI(TAG,"Cursor Step0 at %d %d %d",data->jx,data->jy,data->js);
            } else if(data->step == 1){
                data->jx = 128 - (hid_status->x *128 / (hid_status->cx * 2));
                data->jy = hid_status->y *64 / (hid_status->cy *2);
                data->js = hid_status->joy_ss;
                ESP_LOGI(TAG,"Cursor Step1 at %d %d %d",data->jx,data->jy,data->js);
            } else {
                data->jx = 128 - (hid_status->x *128 / hid_status->max_x);
                data->jy = hid_status->y *64 / hid_status->max_y;
                data->js = hid_status->joy_ss;
                ESP_LOGI(TAG,"Cursor Step2 at %d %d %d",data->jx,data->jy,data->js);
            }
        }
        break;
    case HID_EVENT_JOY_NEED_CALIBRATION:
        if(data->page != DISPLAY_JOY_CALIBRATION_PAGE){
            data->step =0;
            ESP_LOGI(TAG,"JOY need calibration!");
            joyCalibrationDisplay();
        } else {
            ESP_LOGD(TAG,"Already on Calibration Page!");
        }
        break;
    default:
        break;
    }
}


esp_err_t initDisplay(void)
{
    /* i2cdev solution */
    i2c_dev_t device;
    memset(&device, 0, sizeof(i2c_dev_t));

    device.port = 0;
    device.addr = 0x78;
    device.cfg.sda_io_num = SDA_GPIO;
    device.cfg.scl_io_num = SCL_GPIO;
    ESP_ERROR_CHECK(i2c_dev_create_mutex(&device));

    u8g2_Setup_ssd1309_i2c_128x64_noname2_f(
    //	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        u8g2_i2cdev_byte_cb,
        u8g2_gpio_and_delay_cb); // init u8g2 structure

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

    u8g2_SetPowerSave(&u8g2, 0); // wake up display

    info_display_handle.page = DISPLAY_BOOT_PAGE;
    info_display_handle.return_page = DISPLAY_NO_PAGE;

    ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID, &hid_event_handler, (void *)&info_display_handle));

    xTaskCreate(request_status_task, "request_status_task", configMINIMAL_STACK_SIZE * 6, NULL, 1, NULL);
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

/**
 * Display Functions 
 **/

void infoDisplay(void){
    if(info_display_handle.page == DISPLAY_BOOT_PAGE) {
        info_display_handle.page = DISPLAY_MAIN_PAGE;
    }
    else 
    {
        info_display_handle.return_page = DISPLAY_MAIN_PAGE; 
    }
    
}

void otaDisplay(uint8_t perceptual)
{
    info_display_handle.page = DISPLAY_OTA_PAGE;
    info_display_handle.perceptual = perceptual;
}

static uint64_t lastPageTime;

void messageDisplay(char *message, uint16_t timeout){
    strncpy(info_display_handle.message,message,sizeof(info_display_handle.message));
    info_display_handle.return_page = info_display_handle.page;
    info_display_handle.page = DISPLAY_MESSAGE_PAGE;
    info_display_handle.page_timeout = timeout;
    lastPageTime = (esp_timer_get_time() / 1000ULL);
}

void joyCalibrationDisplay(){
    info_display_handle.page_timeout = 0;
    info_display_handle.step = 0;
    info_display_handle.return_page = info_display_handle.page;
    info_display_handle.page = DISPLAY_JOY_CALIBRATION_PAGE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(startJoytickHID());
}

/**
 * Rendering functions
 **/

void _bootPage(info_display_handle_t *data)
{
    u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 2, 17, "GRBL v0.034");

}

static uint64_t last_mainPage_millis = 0;
static bool blink = false;

void _mainPage(info_display_handle_t *data)
{
    uint64_t current =  (esp_timer_get_time() / 1000ULL);
    if( current - last_mainPage_millis > 500){
        last_mainPage_millis = current;
        blink = !blink;
    }

    // GRBL <Idle|MPos:0.000,0.000,0.000|FS:0.0,0>
    // Icon Bar 16x16
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    for (int pi = 0; pi < 15; pi++)
    {
        switch (pi)
        {
        case ICON_STATUS:
            if(!info_display_handle.status_blink && blink)
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
    sprintf(&dispStr[0],"MPos x%03.1f y%03.1f z%03.1f",info_display_handle.x,info_display_handle.y,info_display_handle.z);
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
            if(pi == selmenupos)selmenupos=(selmenupos+selmenu_dir)%7;;
            continue;
        }
        u8g2_DrawGlyph(&u8g2, pi * 18, 64, MenuIcon[pi]);
        if (menupos == pi)
        {
            u8g2_SetDrawColor(&u8g2, 2);
            u8g2_DrawBox(&u8g2, pi * 18, 64 - 18, 18, 18);
            u8g2_SetDrawColor(&u8g2, 1);
        }
        if (menupos != selmenupos && selmenupos == pi && blink)
        {
            u8g2_SetDrawColor(&u8g2, 2);
            u8g2_DrawBox(&u8g2, pi * 18, 64 - 18, 18, 18);
            u8g2_SetDrawColor(&u8g2, 1);
        }
    }
}

void _otaPage(info_display_handle_t *data)
{
    u8g2_DrawBox(&u8g2, 0, 26, data->perceptual, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    char perceptual[5];
    sprintf(perceptual,"%d%%",data->perceptual);
    u8g2_DrawStr(&u8g2, 102, 26+6, perceptual);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
    u8g2_DrawStr(&u8g2, 2, 17, "OTA update");
}

void _messagePage(info_display_handle_t *data)
{
//    u8g2_DrawBox(&u8g2, 0, 26, data->perceptual, 6);
    u8g2_DrawFrame(&u8g2, 0, 10, 127, 58);

    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    u8g2_DrawStr(&u8g2, 2, 32, data->message);

//    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
//    u8g2_DrawStr(&u8g2, 2, 17, "OTA update");
}

void _joyCalibrationPage(info_display_handle_t *data)
{
    u8g2_DrawFrame(&u8g2, 0, 0, 126, 64);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_me);
    switch(data->step){
        case 0: // Center
            u8g2_DrawStr(&u8g2, 4, 12, "Click Center");
        break;
        case 1: // Max
            u8g2_DrawStr(&u8g2, 4, 12, "Click Lower Left");
        break;
        case 2: // test 
            u8g2_DrawStr(&u8g2, 4, 12, "Click to save");
        break;
        default:
            ESP_LOGW(TAG,"Invalid step! %d",data->step);
        break;
    }
    char message[22];
    sprintf(message,"X%d Y%d S%d",data->jx,data->jy,data->js);
    u8g2_DrawStr(&u8g2, 4, 22, message);

    // Draw simbol X position
    u8g2_SetDrawColor(&u8g2, 2);
    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_1x_t);
    u8g2_DrawGlyph(&u8g2,data->jx-4,data->jy-4,0x011B);
    u8g2_SetDrawColor(&u8g2, 1);

    int x = data->jx, y = data->jy;
    x = x>126?126:(x<2?2:x);
    y = y>62?62:(y<2?2:y);
    ESP_LOGD(TAG,"Cross %d %d",x,y);
    u8g2_DrawLine(&u8g2,x-2,y-2,x+2,y+2);
    u8g2_DrawLine(&u8g2,x+2,y-2,x-2,y+2);
}

/**
 * Running Tasks
 **/

void info_display_task(void *params)
{
    info_display_handle_t *data = (info_display_handle_t *)params;
    while (true)
    {
        if(data->page_timeout > 0 && data->return_page != DISPLAY_NO_PAGE ){
            uint64_t current = (esp_timer_get_time() / 1000ULL);
            if(current - lastPageTime > data->page_timeout){
                ESP_LOGD(TAG,"Return to previous page!");
                data->page = data->return_page;
                data->return_page = DISPLAY_NO_PAGE;
            }
        }
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
        case DISPLAY_MESSAGE_PAGE:
            _messagePage(data);
            break;
        case DISPLAY_JOY_CALIBRATION_PAGE:
            _joyCalibrationPage(data);
            break;
        default:
            ESP_LOGI(TAG, "No page to display go Home!");
            data->page = DISPLAY_MAIN_PAGE;
            break;
        }
//        vTaskMissedYield();
        u8g2_SendBuffer(&u8g2);
        vTaskDelay(500 / portTICK_PERIOD_MS); // 500ms refresh for display
    }
}

void request_status_task(void *params){
    while(1){
        uint64_t current = (esp_timer_get_time() / 1000ULL);
        if(current - info_display_handle.lastStatusUpdate > 500){
            printf("?\r");
            info_display_handle.lastStatusUpdate = current;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // 2Hz refresh for info request
    }
}