#pragma once
#include <esp_err.h>
#include "esp_event.h"

// Declare an event base
ESP_EVENT_DECLARE_BASE(SERIAL_EVENT);        

enum {                                      
    SERIAL_EVENT_LINE,                      
};

#define IN_SERIAL_BUFFER_SIZE 256

typedef struct {
    uint8_t line_buffer[IN_SERIAL_BUFFER_SIZE];
    uint16_t i_line;
} in_serial_buffer_t;


esp_err_t initialize_in_serial(void);
