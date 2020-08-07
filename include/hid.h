#pragma once
#include "esp_event.h"
#include <esp_err.h>

// Declare an event base
ESP_EVENT_DECLARE_BASE(HID_EVENT);        // declaration of the timer events family

enum {                                       // declaration of the specific events under the OTA event family
    HID_EVENT_SEL,                     
    HID_EVENT_BEFORE,                      
    HID_EVENT_NEXT,                         
    HID_EVENT_INC,
    HID_EVENT_DEC,
    HID_EVENT_JOY_BUTTON,
    HID_EVENT_JOY_MOVE,
    HID_EVENT_JOY_NEED_CALIBRATION                           
};


typedef enum {
    HID_BUTTON_NONE = 0,
    HID_BUTTON_CLICK,
//    HID_BUTTON_DOUBLE_CLICK, // Not yet implemented
    HID_BUTTON_LONG_PRESSED,
} hid_button_status_t;


typedef struct {
    bool calibrated;

    hid_button_status_t sel;
    hid_button_status_t before;
    hid_button_status_t next;
    hid_button_status_t inc;
    hid_button_status_t dec;
    hid_button_status_t joy;

    uint16_t x,y,cx,cy,max_x,max_y;
    uint8_t joy_ss;
    int16_t dx,dy;
} hid_status_t;


esp_err_t initHID();

esp_err_t startJoytickHID();
esp_err_t stopJoytickHID();
esp_err_t saveCalibration();
