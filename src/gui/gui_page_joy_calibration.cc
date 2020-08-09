/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiPageJoyCalibration class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_page_joy_calibration.h"

#include <esp_log.h>
#include <hid.h>
#include <u8g2.h>

#include <cstring>
#include <string>

#include "gui/gui_display.h"

namespace gui {

static const char *TAG = "gui_page_joy_calibration";

GuiPageJoyCalibration::GuiPageJoyCalibration() {
  hid_event_handler = +[](void *handler_args, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    static_cast<GuiPageJoyCalibration *>(handler_args)
        ->HidEventHandler(handler_args, event_base, event_id, event_data);
  };
  ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID,
                                             hid_event_handler,
                                             static_cast<void *>(this)));
}

GuiPageJoyCalibration::~GuiPageJoyCalibration() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      HID_EVENT, ESP_EVENT_ANY_ID, hid_event_handler));
}

void GuiPageJoyCalibration::renderPage(u8g2_t *u8g2) {
  u8g2_DrawFrame(u8g2, 0, 0, 126, 64);
  u8g2_SetFont(u8g2, u8g2_font_6x12_me);
  switch (step) {
    case 0:  // Center
      u8g2_DrawStr(u8g2, 4, 12, "Click Center");
      break;
    case 1:  // Max
      u8g2_DrawStr(u8g2, 4, 12, "Click Lower Left");
      break;
    case 2:  // test
      u8g2_DrawStr(u8g2, 4, 12, "Click to save");
      break;
    default:
      ESP_LOGW(TAG, "Invalid step! %d", step);
      break;
  }
  char message[22];
  snprintf(message, sizeof(message), "X%d Y%d S%d", jx, jy, js);
  u8g2_DrawStr(u8g2, 4, 22, message);

  // Draw simbol X position
  u8g2_SetDrawColor(u8g2, 2);
  u8g2_SetFont(u8g2, u8g2_font_open_iconic_all_1x_t);
  u8g2_DrawGlyph(u8g2, jx - 4, jy - 4, 0x011B);
  u8g2_SetDrawColor(u8g2, 1);

  int x = jx, y = jy;
  x = x > 126 ? 126 : (x < 2 ? 2 : x);
  y = y > 62 ? 62 : (y < 2 ? 2 : y);
  ESP_LOGD(TAG, "Cross %d %d", x, y);
  u8g2_DrawLine(u8g2, x - 2, y - 2, x + 2, y + 2);
  u8g2_DrawLine(u8g2, x + 2, y - 2, x - 2, y + 2);
}

void GuiPageJoyCalibration::enterPage() {
  ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID,
                                             hid_event_handler,
                                             static_cast<void *>(this)));
  ESP_ERROR_CHECK(startJoytickHID());
}
void GuiPageJoyCalibration::exitPage() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      HID_EVENT, ESP_EVENT_ANY_ID, hid_event_handler));
  ESP_ERROR_CHECK_WITHOUT_ABORT(stopJoytickHID());
}

void GuiPageJoyCalibration::HidEventHandler(void *handler_args,
                                            esp_event_base_t event_base,
                                            int32_t event_id,
                                            void *event_data) {
  hid_status_t *hid_status = static_cast<hid_status_t *>(event_data);

  switch (event_id) {
    case HID_EVENT_INC:
      ESP_LOGI(TAG, "INC pressed!");
      hid_status->joy_ss += 5;
      js = hid_status->joy_ss;
      step = 0;
      commit_calibration(hid_status);
      ESP_LOGD(TAG, "INC joy ss %d", hid_status->joy_ss);
      break;
    case HID_EVENT_DEC:
      ESP_LOGI(TAG, "DEC pressed!");
      hid_status->joy_ss -= 5;
      js = hid_status->joy_ss;
      step = 0;
      commit_calibration(hid_status);
      ESP_LOGD(TAG, "DEC joy ss %d", hid_status->joy_ss);
      break;
    case HID_EVENT_SEL:
    case HID_EVENT_JOY_BUTTON:
      ESP_LOGI(TAG, "JOY pressed or SEL pressed!");
      ESP_LOGD(TAG, "Exec Step %d", step);
      switch (step) {
        case 0:  // Center
          hid_status->cx = hid_status->x;
          hid_status->cy = hid_status->y;
          step++;
          commit_calibration(hid_status);
          break;
        case 1:  // Max
          hid_status->max_x = hid_status->x;
          hid_status->max_y = hid_status->y;
          step++;
          commit_calibration(hid_status);
          break;
        case 2:
          ESP_LOGD(TAG, "End wizard!");
          saveCalibration();
          ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post(
              GUI_EVENT, GUI_EVENT_CLOSE_PAGE, nullptr, 0, portMAX_DELAY));
          break;
      }
      break;
    case HID_EVENT_JOY_MOVE:
      ESP_LOGI(TAG, "JOY move! %d(%d) %d(%d)", hid_status->x, hid_status->dx,
               hid_status->y, hid_status->dy);
      if (step == 0) {
        jx = 128 - (hid_status->x * 128 / (UINT16_MAX / hid_status->joy_ss));
        jy = hid_status->y * 64 / (UINT16_MAX / hid_status->joy_ss);
        js = hid_status->joy_ss;
        ESP_LOGI(TAG, "Cursor Step0 at %d %d %d", jx, jy, js);
      } else if (step == 1) {
        jx = 128 - (hid_status->x * 128 / (hid_status->cx * 2));
        jy = hid_status->y * 64 / (hid_status->cy * 2);
        js = hid_status->joy_ss;
        ESP_LOGI(TAG, "Cursor Step1 at %d %d %d", jx, jy, js);
      } else {
        jx = 128 - (hid_status->x * 128 / hid_status->max_x);
        jy = hid_status->y * 64 / hid_status->max_y;
        js = hid_status->joy_ss;
        ESP_LOGI(TAG, "Cursor Step2 at %d %d %d", jx, jy, js);
      }
      break;
    case HID_EVENT_JOY_NEED_CALIBRATION:
      // TODO(dianlight) Fix multiple HID_EVENT_JOY_NEED_CALIBRATION throw
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_post(
          GUI_EVENT, GUI_EVENT_CHANGE_PAGE,
          const_cast<void *>(static_cast<const void *>(GetName().c_str())),
          GetName().length() + 1, portMAX_DELAY));
      break;
    default:
      break;
  }
}

}  // namespace gui
