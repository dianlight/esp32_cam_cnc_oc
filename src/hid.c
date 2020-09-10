#include "hid.h"

#include <a8i2cGateway.h>
#include <ads111x.h>
#include <i2cdev.h>
#include <mcp23x17.h>
#include <string.h>

#include "pinConfig.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "hid";

/* Event source periodic timer related definitions */
ESP_EVENT_DEFINE_BASE(HID_EVENT);

static hid_status_t hid_status;

int16_t readADSPin(i2c_dev_t *device, ads111x_mux_t mux) {
  ESP_ERROR_CHECK(
      ads111x_set_input_mux(device, mux));  // positive = AIN0, negative = GND
  vTaskDelay(1 / portTICK_PERIOD_MS);       // 1ms config ok

  ESP_ERROR_CHECK(ads111x_start_conversion(device));  // 25us
  vTaskDelay(1 / portTICK_PERIOD_MS);                 // 1ms for request
  bool busy = true;
  do {
    ESP_ERROR_CHECK(ads111x_is_busy(device, &busy));
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms rate is for RATE_128
  } while (busy);

  int16_t raw = 0;
  if (ads111x_get_value(device, &raw) == ESP_OK) {
    return raw;
  } else {
    return -1;
  }
}

void hid_a8i2c_joy_task(void *pvParameters) {
  // A8I2CGW
  // JOY
  cmd_gpio_set_t sgpio;
  sgpio.pin = A8I2C_PIN_X;
  sgpio.mode = kPinModeInput;
  sgpio.type = kAnalog;
  sgpio.hz = 0;      // Single read
  setupGpio(sgpio);  // JoyX
  sgpio.pin = A8I2C_PIN_Y;
  setupGpio(sgpio);  // JoyY
  sgpio.pin = A8I2C_PIN_JOY_BUTTON;
  sgpio.mode = kPinModeInputPullup;
  sgpio.type = kDigital;
  setupGpio(sgpio);  // JoySw

  ESP_LOGD(TAG, "Start task  hid_a8i2c_joy_task");

  while (1) {
    int16_t raw = readLastGpio(A8I2C_PIN_X);
    if (raw >= 0) {
      raw = raw / hid_status.joy_ss;
      hid_status.dx = hid_status.x - raw;
      hid_status.x = raw;
    } else {
      ESP_LOGW(TAG, "Cannot read ADC value for X!");
    }

    //        vTaskDelay(125 / portTICK_PERIOD_MS); // 125? Why?

    raw = readLastGpio(A8I2C_PIN_Y);
    if (raw >= 0) {
      raw = raw / hid_status.joy_ss;
      hid_status.dy = hid_status.y - raw;
      hid_status.y = raw;
    } else {
      ESP_LOGW(TAG, "Cannot read ADC value for Y!");
    }

    ESP_LOGD(TAG, "X %d Y %d dx %d dy %d", hid_status.x, hid_status.y,
             hid_status.dx, hid_status.dy);
    if (hid_status.calibrated && hid_status.x > hid_status.max_x) {
      ESP_LOGW(TAG, "Invalid X calibration data!");
      hid_status.calibrated = false;
      hid_status.max_x = hid_status.x;
    }
    if (hid_status.calibrated && hid_status.y > hid_status.max_y) {
      ESP_LOGW(TAG, "Invalid Y calibration data!");
      hid_status.calibrated = false;
      hid_status.max_y = hid_status.y;
    }
    if (hid_status.cx == 0) {
      ESP_LOGW(TAG, "No center calibration, setting autocenter!");
      hid_status.cx = hid_status.x;
      hid_status.cy = hid_status.y;
      ESP_LOGI(TAG, "AutoSetting Joy Center to %d,%d", hid_status.cx,
               hid_status.cy);
    } else {
      ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_status,
                                     sizeof(hid_status_t), 0));
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void hid_joy_task(void *pvParameters) {
  i2c_dev_t device;
  memset(&device, 0, sizeof(i2c_dev_t));

  ESP_ERROR_CHECK(
      ads111x_init_desc(&device, ADS111X_ADDR_GND, 0, SDA_GPIO, SCL_GPIO));
  ESP_ERROR_CHECK(
      ads111x_set_data_rate(&device, ADS111X_DATA_RATE_8));  // 1 read 125ms
  ESP_ERROR_CHECK(ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT));
  ESP_ERROR_CHECK(ads111x_set_gain(&device, ADS111X_GAIN_4V096));
  ESP_ERROR_CHECK(ads111x_set_comp_queue(&device, ADS111X_COMP_QUEUE_DISABLED));
  ESP_ERROR_CHECK(ads111x_set_comp_latch(&device, ADS111X_COMP_LATCH_DISABLED));
  ESP_ERROR_CHECK(
      ads111x_set_comp_polarity(&device, ADS111X_COMP_POLARITY_LOW));
  ESP_ERROR_CHECK(ads111x_set_comp_mode(&device, ADS111X_COMP_MODE_NORMAL));

  ESP_LOGD(TAG, "Start task  hid_joy_task");

  while (1) {
    int16_t raw = readADSPin(&device, ADS_PIN_X);
    if (raw >= 0) {
      raw = raw / hid_status.joy_ss;
      hid_status.dx = hid_status.x - raw;
      hid_status.x = raw;
    } else {
      ESP_LOGW(TAG, "Cannot read ADC value for X!");
    }

    vTaskDelay(125 / portTICK_PERIOD_MS);  // 125? Why?

    raw = readADSPin(&device, ADS_PIN_Y);
    if (raw >= 0) {
      raw = raw / hid_status.joy_ss;
      hid_status.dy = hid_status.y - raw;
      hid_status.y = raw;
    } else {
      ESP_LOGW(TAG, "Cannot read ADC value for Y!");
    }

    ESP_LOGD(TAG, "X %d Y %d dx %d dy %d", hid_status.x, hid_status.y,
             hid_status.dx, hid_status.dy);
    if (hid_status.calibrated && hid_status.x > hid_status.max_x) {
      ESP_LOGW(TAG, "Invalid X calibration data!");
      hid_status.calibrated = false;
      hid_status.max_x = hid_status.x;
    }
    if (hid_status.calibrated && hid_status.y > hid_status.max_y) {
      ESP_LOGW(TAG, "Invalid Y calibration data!");
      hid_status.calibrated = false;
      hid_status.max_y = hid_status.y;
    }
    if (hid_status.cx == 0) {
      ESP_LOGW(TAG, "No center calibration, setting autocenter!");
      hid_status.cx = hid_status.x;
      hid_status.cy = hid_status.y;
      ESP_LOGI(TAG, "AutoSetting Joy Center to %d,%d", hid_status.cx,
               hid_status.cy);
    } else {
      ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_MOVE, &hid_status,
                                     sizeof(hid_status_t), 0));
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void hid_a8i2c_task(void *pvParameters) {
  // A8I2CGW
  // Buttons
  cmd_gpio_set_t sgpio;
  sgpio.hz = 0;  // Single read
  sgpio.mode = kPinModeInputPullup;
  sgpio.type = kDigital;
  sgpio.pin = A8I2C_PIN_JOY_BUTTON;
  setupGpio(sgpio);  // JoySw
  sgpio.pin = A8I2C_PIN_INC_BUTTON;
  setupGpio(sgpio);  // Inc
  sgpio.pin = A8I2C_PIN_DEC_BUTTON;
  setupGpio(sgpio);  // Dec
  sgpio.pin = A8I2C_PIN_NEXT_BUTTON;
  setupGpio(sgpio);  // Next
  sgpio.pin = A8I2C_PIN_BEFORE_BUTTON;
  setupGpio(sgpio);  // Prev
  sgpio.pin = A8I2C_PIN_SEL_BUTTON;
  setupGpio(sgpio);  // Sel

  while (1) {
    // A8I2CGW
    int16_t val = readLastGpio(A8I2C_PIN_BEFORE_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.before = hid_status.before == HID_BUTTON_NONE
                                ? HID_BUTTON_CLICK
                                : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_BEFORE, &hid_status,
                                       sizeof(hid_status_t), 0));
      } else {
        hid_status.before = HID_BUTTON_NONE;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read GPIO BB");
    }
    val = readLastGpio(A8I2C_PIN_NEXT_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.next = hid_status.next == HID_BUTTON_NONE
                              ? HID_BUTTON_CLICK
                              : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_NEXT, &hid_status,
                                       sizeof(hid_status_t), 0));
      } else {
        hid_status.next = HID_BUTTON_NONE;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read GPIO NB");
    }
    val = readLastGpio(A8I2C_PIN_INC_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.inc = hid_status.inc == HID_BUTTON_NONE
                             ? HID_BUTTON_CLICK
                             : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_INC, &hid_status,
                                       sizeof(hid_status_t), 0));
      } else {
        hid_status.inc = HID_BUTTON_NONE;
      }

    } else {
      ESP_LOGW(TAG, "Unable to read GPIO IB");
    }
    val = readLastGpio(A8I2C_PIN_DEC_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.dec = hid_status.dec == HID_BUTTON_NONE
                             ? HID_BUTTON_CLICK
                             : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_DEC, &hid_status,
                                       sizeof(hid_status_t), 0));
      } else {
        hid_status.dec = HID_BUTTON_NONE;
      }

    } else {
      ESP_LOGW(TAG, "Unable to read GPIO DB");
    }
    val = readLastGpio(A8I2C_PIN_SEL_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.sel = hid_status.sel == HID_BUTTON_NONE
                             ? HID_BUTTON_CLICK
                             : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_SEL, &hid_status,
                                       sizeof(hid_status_t), 0));
      } else {
        hid_status.sel = HID_BUTTON_NONE;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read GPIO SB");
    }
    val = readLastGpio(A8I2C_PIN_JOY_BUTTON);
    if (val >= 0) {
      if (!val) {
        hid_status.joy = hid_status.joy == HID_BUTTON_NONE
                             ? HID_BUTTON_CLICK
                             : HID_BUTTON_LONG_PRESSED;
        ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_BUTTON,
                                       &hid_status, sizeof(hid_status_t), 0));
      } else {
        hid_status.joy = HID_BUTTON_NONE;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read GPIO JB");
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void hid_task(void *pvParameters) {
  mcp23x17_t dev;
  memset(&dev, 0, sizeof(mcp23x17_t));
  ESP_ERROR_CHECK(
      mcp23x17_init_desc(&dev, 0, MCP23X17_ADDR_BASE, SDA_GPIO, SCL_GPIO));

  // Setup DOOR0 as input
  for (uint8_t p = 0; p < 8; p++) {
    mcp23x17_set_mode(&dev, p, MCP23X17_GPIO_INPUT);
  }
  mcp23x17_port_set_pullup(&dev, 0x00FF);

  while (1) {
    uint32_t val = 0;
    for (uint8_t p = 0; p < 8; p++) {
      if (mcp23x17_get_level(&dev, p, &val) == ESP_OK) {
        switch (p) {
          case MCP_PIN_BEFORE_BUTTON:
            if (!val) {
              hid_status.before = hid_status.before == HID_BUTTON_NONE
                                      ? HID_BUTTON_CLICK
                                      : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_BEFORE,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.before = HID_BUTTON_NONE;
            }
            break;
          case MCP_PIN_NEXT_BUTTON:
            if (!val) {
              hid_status.next = hid_status.next == HID_BUTTON_NONE
                                    ? HID_BUTTON_CLICK
                                    : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_NEXT,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.next = HID_BUTTON_NONE;
            }
            break;
          case MCP_PIN_INC_BUTTON:
            if (!val) {
              hid_status.inc = hid_status.inc == HID_BUTTON_NONE
                                   ? HID_BUTTON_CLICK
                                   : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_INC,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.inc = HID_BUTTON_NONE;
            }
            break;
          case MCP_PIN_DEC_BUTTON:
            if (!val) {
              hid_status.dec = hid_status.dec == HID_BUTTON_NONE
                                   ? HID_BUTTON_CLICK
                                   : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_DEC,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.dec = HID_BUTTON_NONE;
            }
            break;
          case MCP_PIN_SEL_BUTTON:
            if (!val) {
              hid_status.sel = hid_status.sel == HID_BUTTON_NONE
                                   ? HID_BUTTON_CLICK
                                   : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_SEL,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.sel = HID_BUTTON_NONE;
            }
            break;
          case MCP_PIN_JOY_BUTTON:
            if (!val) {
              hid_status.joy = hid_status.joy == HID_BUTTON_NONE
                                   ? HID_BUTTON_CLICK
                                   : HID_BUTTON_LONG_PRESSED;
              ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_BUTTON,
                                             &hid_status, sizeof(hid_status_t),
                                             0));
            } else {
              hid_status.joy = HID_BUTTON_NONE;
            }
            break;
          default:
            break;
        }
        vTaskMissedYield();
      } else
        ESP_LOGE(TAG, "Errore reading pin %d", p);
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

static TaskHandle_t hid_task_handle;

esp_err_t initHID() {
  // Calibration Data
  hid_status.calibrated = false;
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("hid_storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    // Read
    ESP_LOGD(TAG, "Reading calibration from NVS ... ");
    hid_status.calibrated = true;
    err = nvs_get_u16(my_handle, "joy_cx", &hid_status.cx);
    if (err == ESP_ERR_NVS_NOT_FOUND)
      hid_status.calibrated = false;
    else if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }
    err = nvs_get_u16(my_handle, "joy_cy", &hid_status.cy);
    if (err == ESP_ERR_NVS_NOT_FOUND)
      hid_status.calibrated = false;
    else if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }
    err = nvs_get_u16(my_handle, "joy_max_x", &hid_status.max_x);
    if (err == ESP_ERR_NVS_NOT_FOUND)
      hid_status.calibrated = false;
    else if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }
    err = nvs_get_u16(my_handle, "joy_max_y", &hid_status.max_y);
    if (err == ESP_ERR_NVS_NOT_FOUND)
      hid_status.calibrated = false;
    else if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }
    hid_status.joy_ss = 10;  // Default value
    err = nvs_get_u8(my_handle, "joy_sensibility", &hid_status.joy_ss);
    if (err == ESP_ERR_NVS_NOT_FOUND)
      hid_status.calibrated = false;
    else if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
    }

    ESP_LOGI(
        TAG,
        "NVS Joy calibration data center(%d,%d) max(%d %d) sensibility(%d)",
        hid_status.cx, hid_status.cy, hid_status.max_x, hid_status.max_y,
        hid_status.joy_ss);

    // Close
    nvs_close(my_handle);
  }
  if (!hid_status.calibrated)
    ESP_ERROR_CHECK(esp_event_post(HID_EVENT, HID_EVENT_JOY_NEED_CALIBRATION,
                                   &hid_status, sizeof(hid_status_t),
                                   portMAX_DELAY));
  //

//  if (xTaskCreatePinnedToCore(hid_task, "hid_task",
//                              configMINIMAL_STACK_SIZE * 8, NULL, 1,
//                              &hid_task_handle, APP_CPU_NUM) == pdPASS) {
  if (xTaskCreatePinnedToCore(hid_a8i2c_task, "hid_task",
                              configMINIMAL_STACK_SIZE * 8, NULL, 1,
                              &hid_task_handle, APP_CPU_NUM) == pdPASS) {
    return ESP_OK;
  } else {
    return ESP_FAIL;
  }
}

void stopHid() {
  ESP_LOGD(TAG, "Suspend InfoDisplay task");
  if (hid_task_handle != NULL) {
    vTaskDelete(hid_task_handle);
    hid_task_handle = NULL;
    /*
    if (eTaskGetState(&info_display_handle.task) == eRunning)
        vTaskSuspend(&info_display_handle.task);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    */
  }
}

static TaskHandle_t hid_joy_task_handle;

esp_err_t startJoytickHID() {
  ESP_LOGD(TAG, "Request Joy Task!");

  if (hid_joy_task_handle != NULL) {
    if (eTaskGetState(&hid_joy_task_handle) == eSuspended) {
      ESP_LOGD(TAG, "Resume Joy Task!");
      vTaskResume(&hid_joy_task_handle);
      return ESP_OK;
    } else {
      return ESP_FAIL;
    }
  } else {
    ESP_LOGD(TAG, "Start Joy Task!");
//    if (xTaskCreatePinnedToCore(hid_joy_task, "hid_joy_task",
//                                configMINIMAL_STACK_SIZE * 8, NULL, 1,
//                                &hid_joy_task_handle, APP_CPU_NUM) == pdPASS) {
    if (xTaskCreatePinnedToCore(hid_a8i2c_joy_task, "hid_a8i2c_joy_task",
                                configMINIMAL_STACK_SIZE * 8, NULL, 1,
                                &hid_joy_task_handle, APP_CPU_NUM) == pdPASS) {
      return ESP_OK;
    } else {
      return ESP_FAIL;
    }
  }
}

esp_err_t stopJoytickHID() {
  ESP_LOGD(TAG, "Suspend Joy Task!");
  if (hid_joy_task_handle != NULL) {
    if (eTaskGetState(&hid_joy_task_handle) == eRunning ||
        eTaskGetState(&hid_joy_task_handle) == eReady) {
      vTaskSuspend(&hid_joy_task_handle);
      return ESP_OK;
    } else {
      ESP_LOGW(TAG, "Joy Task Invalid state %d!",
               eTaskGetState(&hid_joy_task_handle));
      return ESP_FAIL;
    }
  }
  ESP_LOGW(TAG, "No handle for Joy Task!");
  return ESP_FAIL;
}

esp_err_t saveCalibration() {
  // Calibration Data
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("hid_storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    return err;
  } else {
    // Read
    ESP_LOGD(TAG, "Save calibration to NVS ... ");
    hid_status.calibrated = true;

    err = nvs_set_u16(my_handle, "joy_cx", hid_status.cx);
    if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) writing!\n", esp_err_to_name(err));
    }
    err = nvs_set_u16(my_handle, "joy_cy", hid_status.cy);
    if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) writing!\n", esp_err_to_name(err));
    }
    err = nvs_set_u16(my_handle, "joy_max_x", hid_status.max_x);
    if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) writing!\n", esp_err_to_name(err));
    }
    err = nvs_set_u16(my_handle, "joy_max_y", hid_status.max_y);
    if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) writing!\n", esp_err_to_name(err));
    }
    err = nvs_set_u8(my_handle, "joy_sensibility", hid_status.joy_ss);
    if (err != ESP_OK) {
      hid_status.calibrated = false;
      ESP_LOGW(TAG, "Error (%s) writing!\n", esp_err_to_name(err));
    }

    ESP_LOGD(TAG, "Committing updates in NVS ... ");
    err = nvs_commit(my_handle);

    // Close
    nvs_close(my_handle);
    return err;
  }
}

void commit_calibration(hid_status_t *newvalues) {
  hid_status.cx = newvalues->cx;
  hid_status.cy = newvalues->cy;
  hid_status.max_x = newvalues->max_x;
  hid_status.max_y = newvalues->max_y;
  hid_status.joy_ss = newvalues->joy_ss;
}
