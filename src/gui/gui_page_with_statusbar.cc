/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiPageWithStatusbar class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_page_with_statusbar.h"

#include <esp_log.h>
#include <hid.h>
#include <u8g2.h>

#include <cstring>
#include <string>

#include "gui/gui_display.h"

namespace gui {

static const char *TAG = "gui_page_with_statusbar";

GuiPageWithStatusbar::GuiPageWithStatusbar() {
  status_event_handler = +[](void *handler_args, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
    static_cast<GuiPageWithStatusbar *>(handler_args)
        ->StatusEventHandler(handler_args, event_base, event_id, event_data);
  };
}

GuiPageWithStatusbar::~GuiPageWithStatusbar() {}

void GuiPageWithStatusbar::renderPage(u8g2_t *u8g2) {
  uint64_t current = (esp_timer_get_time() / 1000ULL);
  if (current - last_mainPage_millis > 500) {
    last_mainPage_millis = current;
    blink = !blink;
  }

  // Icon Bar 16x16
  u8g2_SetFont(u8g2, u8g2_font_open_iconic_all_1x_t);
  for (uint8_t pi = 0; pi < 15; pi++) {
    switch (static_cast<Icon>(pi)) {
      case Icon::kStatus:
        if (!status_blink && blink)
          u8g2_DrawGlyph(u8g2, pi * 8, 8,
                         kIconBar[pi][static_cast<uint8_t>(status)]);
        break;
      case Icon::kWiFi:
        if (!wifi_blink && blink)
          u8g2_DrawGlyph(u8g2, pi * 8, 8, kIconBar[pi][wifi ? 0 : 1]);
        break;
      case Icon::kTcpSerial:
        u8g2_DrawGlyph(u8g2, pi * 8, 8, kIconBar[pi][tcp_serial ? 0 : 1]);
        break;
      case Icon::kBluetoothSerial:
        u8g2_DrawGlyph(u8g2, pi * 8, 8, kIconBar[pi][bluetooth_serial ? 0 : 1]);
        break;
      case Icon::kWebcam:
        u8g2_DrawGlyph(u8g2, pi * 8, 8, kIconBar[pi][webcam ? 0 : 1]);
        break;
      default:
        if (kIconBar[pi][0] != 0x0000)
          u8g2_DrawGlyph(u8g2, pi * 8, 8, kIconBar[pi][0]);
        break;
    }
  }
  // Separator Line
  u8g2_DrawLine(u8g2, 0, 9, 128, 9);
}

void GuiPageWithStatusbar::enterPage() {
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             status_event_handler,
                                             static_cast<void *>(this)));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP,
                                             status_event_handler,
                                             static_cast<void *>(this)));
  ESP_ERROR_CHECK(esp_event_handler_register(
      WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, status_event_handler,
      static_cast<void *>(this)));
  ESP_ERROR_CHECK(esp_event_handler_register(
      WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, status_event_handler,
      static_cast<void *>(this)));
}

void GuiPageWithStatusbar::exitPage() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, status_event_handler));
}

void GuiPageWithStatusbar::StatusEventHandler(void *handler_args,
                                              esp_event_base_t event_base,
                                              int32_t event_id,
                                              void *event_data) {
  //  hid_status_t *hid_status = static_cast<hid_status_t *>(event_data);
  // TODO(dianlight) Implement Events!
  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    wifi = true;
    wifi_blink = false;
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi = false;
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
    wifi = true;
    wifi_blink = true;
  }
}

}  // namespace gui
