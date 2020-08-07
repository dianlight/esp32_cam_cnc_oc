#include "jog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "infoDisplay.h"
#include "hid.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "joy";

static uint16_t max_x,max_y,max_d;

static bool joy_mode = false;

static void hid_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGD(TAG,"Joy Event!");
    hid_status_t *hid_status = (hid_status_t*)event_data;
    switch (event_id)
    {
        case HID_EVENT_JOY_BUTTON:
            ESP_LOGI(TAG,"JOY pressed! Setting home...");
            break;
        case HID_EVENT_JOY_MOVE:
            ESP_LOGI(TAG,"JOY move! %d %d %d %d",hid_status->x,hid_status->y,hid_status->dx,hid_status->dy);
            if(hid_status->x > max_x)max_x = hid_status->x;
            if(hid_status->y > max_y)max_y = hid_status->y;
            if( hid_status->x == hid_status->cx && hid_status->y == hid_status->cy){
                if(joy_mode){
                    ESP_LOGI(TAG,"Stop JOY");
                    putchar(0x85); // Joy stop command
                    joy_mode = false;
                }
            } else  {
                uint16_t dist = sqrt( pow((max_x/2)-hid_status->x,2)+pow((max_y/2)-hid_status->y,2) );
                float xi = hid_status->dx > 0?0.5:(hid_status->dx < 0?-0.5:0.0);
                float yi = hid_status->dy > 0?0.5:(hid_status->dy < 0?-0.5:0.0);
                ESP_LOGI(TAG,"JOY dist %d %.2f %.2f [%d,%d]",dist,xi,yi,hid_status->x,hid_status->y);
                if( dist > max_d)max_d = dist;
                uint8_t feed = max_d/dist*100;
                ESP_LOGI(TAG,"Computed -->$J=G91 G20 X%.2f Y%.2f F%d",xi,yi,feed);
                printf("$J=G91 G20 X%.2f Y%.2f F%d\r",xi,yi,feed);
                joy_mode = true;
            }
            break;
    }
}


bool startJog(void){

    if(info_display_handle.status != GRBL_IDLE && info_display_handle.status != GRBL_JOG) 
    {
        ESP_LOGW(TAG,"Wrong GRBL status for JOG");
        messageDisplay("Invalid Status!",5000);
        return false;
    } else {
        ESP_LOGD(TAG,"Request GRBL JOG mode");
        ESP_ERROR_CHECK(startJoytickHID());
        ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, HID_EVENT_JOY_BUTTON, &hid_event_handler, NULL));
        return true;
    }
}

bool stopJog(void){
    joy_mode = false;
    ESP_LOGI(TAG,"Stop GRBL JOG mode");
    ESP_ERROR_CHECK(stopJoytickHID());
    ESP_ERROR_CHECK(esp_event_handler_unregister(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(HID_EVENT, HID_EVENT_JOY_BUTTON, &hid_event_handler));
    return true;
}

