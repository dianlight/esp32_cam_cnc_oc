#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiPageWithMenu virtual class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/

#include <esp_event.h>
#include <u8g2.h>

#include <string>
#include "gui/gui_page.h"

namespace gui {

class GuiPageWithMenu : public GuiPage {
 public:
  GuiPageWithMenu();
  ~GuiPageWithMenu();

  void renderPage(u8g2_t* u8g2);
  void enterPage();
  void exitPage();

 private:
  uint8_t menupos = 0, selmenupos = 0;
  int8_t selmenu_dir = 0;

  int64_t last_mainPage_millis = 0;
  bool blink = false;

  esp_event_handler_t hid_event_handler;

  enum class Menu { kCycle = 0, KJog = 1, kProbe = 2, kSSD = 4, kConfig = 6 };

  static constexpr uint16_t MenuIcon[7] = {
      0x00F3,  // Cycle start
      0x00E0,  // Jog
      0x0082,  // Probe
      0x0000,  //
      0x00AB,  // SD/File
      0x0000,  //
      0x0081,  // Config
  };

  void HidEventHandler(void* handler_args, esp_event_base_t event_base,
                       int32_t event_id, void* event_data);
};

}  // namespace gui
