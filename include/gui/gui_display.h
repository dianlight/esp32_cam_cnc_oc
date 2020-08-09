#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiDisplay class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include <i2cdev.h>
#include <esp_event.h>

#include <map>
#include <string>

#include "gui/gui_page.h"


// Declare an event base
ESP_EVENT_DECLARE_BASE(GUI_EVENT);          // declaration of the events family

enum {                                      // declaration of the specific events under the GUI event family
    GUI_EVENT_CHANGE_PAGE                   // raised for change gui page
};

namespace gui {

class GuiDisplay {
 public:
  GuiDisplay(gpio_num_t sda, gpio_num_t scl);
  ~GuiDisplay();

  void AddGuiPage(GuiPage *page) { gui_pages[page->GetName()] = page; }

  bool DisplayGuiPage(const std::string name) {
    if (gui_pages.count(name) > 0) {
      current_gui_page = gui_pages[name];
      return true;
    } else {
      return false;
    }
  }

  GuiPage *GetDisplayedGuiPage() { return current_gui_page; }

 private:
  gpio_num_t sda;
  gpio_num_t scl;
  u8g2_t u8g2;
  TaskHandle_t gui_display_task_handle;
  std::map<std::string, GuiPage *> gui_pages;
  GuiPage *current_gui_page;
  esp_event_handler_t gui_event_handler;

  void GuiEventHandler(void* handler_args, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);
  void GuiDisplayTask(void *params);
};

}  // namespace gui
// extern GuiDisplay gui_display;
