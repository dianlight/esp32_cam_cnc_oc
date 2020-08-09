/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiDisplay class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_display.h"

#include <esp_log.h>
#include <u8g2.h>

#include <cstring>
#include <string>

#include "hal/u8g2_i2c_hal.h"

/* Event source periodic timer related definitions */
ESP_EVENT_DEFINE_BASE(GUI_EVENT);

namespace gui {

static const char *TAG = "gui_display";

GuiDisplay::GuiDisplay(gpio_num_t sda, gpio_num_t scl) : sda(sda), scl(scl) {
  /* i2cdev solution */
  i2c_dev_t device;
  memset(&device, 0, sizeof(i2c_dev_t));

  device.port = i2c_port_t::I2C_NUM_0;
  device.addr = 0x78;
  device.cfg.sda_io_num = sda;
  device.cfg.scl_io_num = scl;
  ESP_ERROR_CHECK(i2c_dev_create_mutex(&device));

  u8g2_Setup_ssd1309_i2c_128x64_noname2_f(
      &u8g2, U8G2_R0, u8g2_i2cdev_byte_cb,
      u8g2_gpio_and_delay_cb);  // init u8g2 structure

  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                            // sleep mode after this,

  u8g2_SetPowerSave(&u8g2, 0);  // wake up display

  // Regiter GUI events
  gui_event_handler = +[](void *handler_args, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    static_cast<GuiDisplay *>(handler_args)
        ->GuiEventHandler(handler_args, event_base, event_id, event_data);
  };
  ESP_ERROR_CHECK(esp_event_handler_register(GUI_EVENT, GUI_EVENT_CHANGE_PAGE,
                                             gui_event_handler,
                                             static_cast<void *>(this)));

  if (xTaskCreate(
          +[](void *instance) {
            static_cast<GuiDisplay *>(instance)->GuiDisplayTask(instance);
          },
          "gui_display_task", configMINIMAL_STACK_SIZE * 6,
          static_cast<void *>(this), 1, &gui_display_task_handle) == pdPASS) {
    ESP_LOGD(TAG, "Create InfoDisplay task");
  } else {
    ESP_LOGE(TAG, "Error creating task");
  }
}

GuiDisplay::~GuiDisplay() {
  if (gui_display_task_handle != nullptr) {
    vTaskDelete(gui_display_task_handle);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 500ms refresh for display
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      GUI_EVENT, GUI_EVENT_CHANGE_PAGE, gui_event_handler));
}

void GuiDisplay::GuiDisplayTask(void *params) {
  // info_display_handle_t *data = (info_display_handle_t *)params;
  while (true) {
    u8g2_ClearBuffer(&u8g2);
    if (current_gui_page != nullptr) {
      current_gui_page->renderPage(&u8g2);
    } else if (gui_pages.size() > 0) {
      current_gui_page = gui_pages.begin()->second;
      current_gui_page->enterPage();
      current_gui_page->renderPage(&u8g2);
    } else {
      ESP_LOGW(TAG, "No page to display!");
    }
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 500ms refresh for display
  }
}

void GuiDisplay::GuiEventHandler(void *handler_args,
                                 esp_event_base_t event_base, int32_t event_id,
                                 void *event_data) {
  switch (event_id) {
    case GUI_EVENT_CHANGE_PAGE: {
      const char *pagename = static_cast<const char *>(event_data);
      ESP_LOGD(TAG, "Request page switch to %s", pagename);
      DisplayGuiPage(std::string(pagename));
      break;
    }
    case GUI_EVENT_CLOSE_PAGE: {
      if (!DisplayGuiClosePage())
        ESP_LOGW(TAG, "Unable to close Page %s",
                 current_gui_page->GetName().c_str());
      break;
    }
    default: {
      ESP_LOGW(TAG, "Unknown Event_id %d", event_id);
      break;
    }
  }
}

}  // namespace gui
