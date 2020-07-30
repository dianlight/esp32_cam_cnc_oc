#pragma once
#include "esp_event.h"

#define OTA_URL_SIZE 256

// Declare an event base
ESP_EVENT_DECLARE_BASE(OTA_EVENT);        // declaration of the timer events family

enum {                                       // declaration of the specific events under the OTA event family
    OTA_EVENT_STARTED,                       // raised when the ota start
    OTA_EVENT_PROGRESS,                      // raised when ota progress change
    OTA_EVENT_ERROR,                         // raised when ota progress go to error state
    OTA_EVENT_END                            // raised when the ota end
};

typedef struct {
    uint8_t command;
    uint16_t port;
    uint32_t size;
    char filemd5[256];
    char addr_str[128];
    uint32_t flashed;
} ota_file_info_t;

esp_err_t start_arduino_ota();
//void stop_arduino_ota();
