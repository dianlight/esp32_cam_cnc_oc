#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiPageBoot class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/

#include "gui/gui_page.h"

namespace gui {

class GuiPageBoot : public GuiPage {
 public:
  GuiPageBoot();
  ~GuiPageBoot();

  void renderPage(u8g2_t *u8g2);
  void enterPage();
  void exitPage();

 private:
};

}  // namespace gui
