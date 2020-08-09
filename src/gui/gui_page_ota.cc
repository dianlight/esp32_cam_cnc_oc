/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiPageOta class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_page_ota.h"

#include <esp_log.h>
#include <idf_arduino_ota.h>
#include <u8g2.h>

#include <cstring>
#include <string>

#include "gui/gui_display.h"

namespace gui {

static const char *TAG = "gui_page_ota";

GuiPageOta::GuiPageOta() {
  ota_event_handler = +[](void *handler_args, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    static_cast<GuiPageOta *>(handler_args)
        ->OtaEventHandler(handler_args, event_base, event_id, event_data);
  };
  ESP_ERROR_CHECK(esp_event_handler_register(OTA_EVENT, ESP_EVENT_ANY_ID,
                                             ota_event_handler,
                                             static_cast<void *>(this)));
}

GuiPageOta::~GuiPageOta() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      OTA_EVENT, ESP_EVENT_ANY_ID, ota_event_handler));
}

void GuiPageOta::renderPage(u8g2_t *u8g2) {
  u8g2_DrawBox(u8g2, 0, 26, perceptual, 6);
  u8g2_DrawFrame(u8g2, 0, 26, 100, 6);

  u8g2_SetFont(u8g2, u8g2_font_6x12_me);
  char charperceptual[5];
  snprintf(charperceptual, sizeof(charperceptual), "%d%%", perceptual);
  u8g2_DrawStr(u8g2, 102, 26 + 6, charperceptual);

  u8g2_SetFont(u8g2, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(u8g2, 2, 17, "OTA update");
}

void GuiPageOta::enterPage() {}
void GuiPageOta::exitPage() {}

void GuiPageOta::OtaEventHandler(void *handler_args,
                                 esp_event_base_t event_base, int32_t event_id,
                                 void *event_data) {
  ota_file_info_t *ota_file_info = static_cast<ota_file_info_t *>(event_data);
  uint8_t perceptual;
  switch (event_id) {
    case OTA_EVENT_STARTED: {
      ESP_LOGI(TAG, "OTA Started %p", ota_file_info);
      perceptual = 0;
      ESP_ERROR_CHECK_WITHOUT_ABORT(
          esp_event_post(GUI_EVENT, GUI_EVENT_CHANGE_PAGE, const_cast<void *>(static_cast<const void*>(GetName().c_str())),
                         GetName().length() + 1, portMAX_DELAY));
      break;
    }
    case OTA_EVENT_END: {
      ESP_LOGI(TAG, "OTA Stopped");
      break;
    }
    case OTA_EVENT_PROGRESS: {
      perceptual = ota_file_info->flashed * 100 / ota_file_info->size;
      ESP_LOGD(TAG, "OTA Running %d %d/%d", perceptual, ota_file_info->flashed,
               ota_file_info->size);
      break;
    }
    case OTA_EVENT_ERROR: {
      ESP_LOGE(TAG, "OTA Error");
      break;
    }
    default: {
      break;
    }
  }
}

}  // namespace gui
