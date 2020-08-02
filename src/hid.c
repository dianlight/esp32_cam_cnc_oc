#include "hid.h"
#include "pinConfig.h"
#include <string.h>
#include <i2cdev.h>
#include <mcp23x17.h>
#include <ads111x.h>
#include <esp_log.h>

#define JOY_SENSIBILITY   200

static const char *TAG = "hid";

/* Event source periodic timer related definitions */
ESP_EVENT_DEFINE_BASE(HID_EVENT);

static hid_status_t hid_status;

int16_t readADSPin(i2c_dev_t *device, ads111x_mux_t mux)
{
    ESP_ERROR_CHECK(ads111x_set_input_mux(device, mux)); // positive = AIN0, negative = GND
    ads111x_set_comp_queue(device, ADS111X_COMP_QUEUE_DISABLED);
    ads111x_set_comp_latch(device, ADS111X_COMP_LATCH_DISABLED);
    ads111x_set_comp_polarity(device, ADS111X_COMP_POLARITY_LOW);
    ads111x_set_comp_mode(device, ADS111X_COMP_MODE_NORMAL);
    ads111x_set_data_rate(device, ADS111X_DATA_RATE_128);
    ads111x_set_mode(device, ADS111X_MODE_SINGLE_SHOT);
    ESP_ERROR_CHECK(ads111x_set_gain(device, ADS111X_GAIN_4V096));

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ads111x_start_conversion(device));
    bool busy = true;
    do
    {
        ESP_ERROR_CHECK(ads111x_is_busy(device, &busy));
    } while (busy);

    int16_t raw = 0;
    if (ads111x_get_value(device, &raw) == ESP_OK)
    {
        return raw;
    }
    else
    {
        return -1;
    }
}

void hid_task(void *pvParameters)
{
    mcp23x17_t dev;
    memset(&dev, 0, sizeof(mcp23x17_t));
    ESP_ERROR_CHECK(mcp23x17_init_desc(&dev, 0, MCP23X17_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup PORTA0 as input
    for (uint8_t p = 0; p < 8; p++)
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
        for (uint8_t p = 0; p < 8; p++)
        {
            if (mcp23x17_get_level(&dev, p, &val) == ESP_OK)
            {
                switch (p)
                {
                case MCP_PIN_BEFORE_BUTTON:
                    if (!val)
                    {
                        hid_status.before = hid_status.before == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_BEFORE, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.before = HID_BUTTON_NONE;
                    }
                    break;
                case MCP_PIN_NEXT_BUTTON:
                    if (!val)
                    {
                        hid_status.next = hid_status.next == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_NEXT, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.next = HID_BUTTON_NONE;
                    }
                    break;
                case MCP_PIN_INC_BUTTON:
                    if (!val)
                    {
                        hid_status.inc = hid_status.inc == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_INC, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.inc = HID_BUTTON_NONE;
                    }
                    break;
                case MCP_PIN_DEC_BUTTON:
                    if (!val)
                    {
                        hid_status.dec = hid_status.dec == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_DEC, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.dec = HID_BUTTON_NONE;
                    }
                    break;
                case MCP_PIN_SEL_BUTTON:
                    if (!val)
                    {
                        hid_status.sel = hid_status.sel == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_SEL, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.sel = HID_BUTTON_NONE;
                    }
                    break;
                case MCP_PIN_JOY_BUTTON:
                    if (!val)
                    {
                        hid_status.joy = hid_status.joy == HID_BUTTON_NONE ? HID_BUTTON_CLICK : HID_BUTTON_LONG_PRESSED;
                        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_BUTTON, &hid_status, sizeof(hid_status_t), 0));
                    }
                    else
                    {
                        hid_status.joy = HID_BUTTON_NONE;
                    }
                    break;
                default:
//                    ESP_LOGW(TAG, "Unmapped Pin %d -> %s", p, val ? "HIGH" : "LOW");
                    break;
                }
                vTaskMissedYield();
            }
            else
                ESP_LOGE(TAG, "Errore reading pin %d", p);
        }
        ESP_LOGD(TAG, "MPC DONE!");
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

        vTaskDelay(100 / portTICK_PERIOD_MS);

        int16_t raw = readADSPin(&device,ADS_PIN_X);
        if(raw >= 0){
            hid_status.dx = hid_status.x - raw;
            if(abs(hid_status.dx) > JOY_SENSIBILITY){
                hid_status.x = raw;
                ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_status, sizeof(hid_status_t), 0));
 //               printf("Raw ADC value: %d,", raw);
            }
        } else {
            ESP_LOGW(TAG, "Cannot read ADC value for X!");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);

        raw = readADSPin(&device,ADS_PIN_Y);
        if(raw >= 0){
            hid_status.dy = hid_status.y - raw;
            if(abs(hid_status.dy) > JOY_SENSIBILITY){
                hid_status.y = raw;
                ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_status, sizeof(hid_status_t), 0));
 //               printf("Raw ADC value: %d,", raw);
            }
        } else {
            ESP_LOGW(TAG, "Cannot read ADC value for Y!");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static TaskHandle_t hid_task_handle;

esp_err_t initHID()
{
//    esp_err_t rt = i2cdev_init();
//    xTaskCreate(hid_task, "hid_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
//    xTaskCreatePinnedToCore(hid_task, "hid_task", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL, APP_CPU_NUM);
//    return rt;
    if (xTaskCreatePinnedToCore(hid_task, "hid_task", configMINIMAL_STACK_SIZE * 8, NULL, 1, &hid_task_handle, APP_CPU_NUM) == pdPASS)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }

}

void stopHid()
{
    ESP_LOGD(TAG, "Suspend InfoDisplay task");
    if (hid_task_handle != NULL)
    {
        vTaskDelete(hid_task_handle);
        hid_task_handle = NULL;
        /*
        if (eTaskGetState(&info_display_handle.task) == eRunning)
            vTaskSuspend(&info_display_handle.task);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        */
    }
}



