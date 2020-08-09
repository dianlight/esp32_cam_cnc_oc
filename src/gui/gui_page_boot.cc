/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This the GuiPageBoot class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include "gui/gui_page_boot.h"

#include <esp_log.h>
#include <u8g2.h>

#include <cstring>
#include <string>


namespace gui {

static const char *TAG = "gui_page_boot";

GuiPageBoot::GuiPageBoot() {}
GuiPageBoot::~GuiPageBoot() {}

void GuiPageBoot::renderPage(u8g2_t *u8g2) {
  u8g2_DrawBox(u8g2, 0, 26, 80, 6);
  u8g2_DrawFrame(u8g2, 0, 26, 100, 6);

  u8g2_SetFont(u8g2, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(u8g2, 2, 17, "GRBL v0.034");
}

void GuiPageBoot::enterPage() {}
void GuiPageBoot::exitPage() {}

}  // namespace gui
