#include "hal/u8g2_i2c_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include <string.h>
#include <i2cdev.h>
#include <u8g2.h>
#include "pinConfig.h"


#define I2C_FREQ_HZ 1000000

static const char *TAG = "u8g2_hal";

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