#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiPageBoot virtual class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include <esp_event.h>

#include "gui/gui_page.h"

namespace gui {

class GuiPageOta : public GuiPage {
 public:
  GuiPageOta();
  ~GuiPageOta();

  void renderPage(u8g2_t* u8g2);
  void enterPage();
  void exitPage();

 private:
  uint8_t perceptual;
  esp_event_handler_t ota_event_handler;
  void OtaEventHandler(void* handler_args, esp_event_base_t event_base,
                       int32_t event_id, void* event_data);
};

}  // namespace gui
