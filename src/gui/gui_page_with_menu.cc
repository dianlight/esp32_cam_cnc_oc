/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiPageWithMenu class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_page_with_menu.h"

#include <esp_log.h>
#include <hid.h>
#include <u8g2.h>

#include <cstring>
#include <string>

#include "gui/gui_display.h"

namespace gui {

static const char *TAG = "gui_page_with_menu";

GuiPageWithMenu::GuiPageWithMenu() {
  hid_event_handler = +[](void *handler_args, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    static_cast<GuiPageWithMenu *>(handler_args)
        ->HidEventHandler(handler_args, event_base, event_id, event_data);
  };
}

GuiPageWithMenu::~GuiPageWithMenu() {
}

void GuiPageWithMenu::renderPage(u8g2_t *u8g2) {
  uint64_t current = (esp_timer_get_time() / 1000ULL);
  if (current - last_mainPage_millis > 500) {
    last_mainPage_millis = current;
    blink = !blink;
  }

  u8g2_SetFont(u8g2, u8g2_font_open_iconic_all_2x_t);
  for (int pi = 0; pi < 7; pi++) {
    if (MenuIcon[pi] == 0x00) {
      if (pi == selmenupos) selmenupos = (selmenupos + selmenu_dir) % 7;
      continue;
    }
    u8g2_DrawGlyph(u8g2, pi * 18, 64, MenuIcon[pi]);
    if (menupos == pi) {
      u8g2_SetDrawColor(u8g2, 2);
      u8g2_DrawBox(u8g2, pi * 18, 64 - 18, 18, 18);
      u8g2_SetDrawColor(u8g2, 1);
    }
    if (menupos != selmenupos && selmenupos == pi && blink) {
      u8g2_SetDrawColor(u8g2, 2);
      u8g2_DrawBox(u8g2, pi * 18, 64 - 18, 18, 18);
      u8g2_SetDrawColor(u8g2, 1);
    }
  }
}

void GuiPageWithMenu::enterPage() {
  ESP_ERROR_CHECK(esp_event_handler_register(HID_EVENT, ESP_EVENT_ANY_ID,
                                             hid_event_handler,
                                             static_cast<void *>(this)));
}
void GuiPageWithMenu::exitPage() {
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_unregister(
      HID_EVENT, ESP_EVENT_ANY_ID, hid_event_handler));
}

void GuiPageWithMenu::HidEventHandler(void *handler_args,
                                      esp_event_base_t event_base,
                                      int32_t event_id, void *event_data) {
//  hid_status_t *hid_status = static_cast<hid_status_t *>(event_data);

  switch (event_id) {
    case HID_EVENT_BEFORE:
      selmenu_dir = -1;
      if (selmenupos + selmenu_dir < 0)
        selmenupos = 6;
      else
        selmenupos = (selmenupos + selmenu_dir) % 7;
      break;
    case HID_EVENT_NEXT:
      selmenu_dir = 1;
      selmenupos = (selmenupos + selmenu_dir) % 7;
      break;
    case HID_EVENT_INC:
      ESP_LOGI(TAG, "INC pressed!");
      break;
    case HID_EVENT_DEC:
      ESP_LOGI(TAG, "DEC pressed!");
      break;
    case HID_EVENT_SEL:
      ESP_LOGD(TAG, "SEL pressed!");
      if (selmenupos != menupos) {
        menupos = selmenupos;
        selmenu_dir = 0;
      }
      break;
    default:
      break;
  }
}

}  // namespace gui
