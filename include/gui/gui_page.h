#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiPage virtual class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/

#include <u8g2.h>

#include <string>

namespace gui {

class GuiPage {
 public:
  GuiPage() {}
  virtual ~GuiPage() {}

  virtual void renderPage(u8g2_t* u8g2);
  virtual void enterPage();
  virtual void exitPage();

  std::string GetName() { return className(__PRETTY_FUNCTION__); }

 private:
  // #define __CLASS_NAME__ className(__PRETTY_FUNCTION__)
  inline std::string className(const std::string& prettyFunction) {
    size_t colons = prettyFunction.find("::");
    if (colons == std::string::npos) return "::";
    size_t begin = prettyFunction.substr(0, colons).rfind(" ") + 1;
    size_t end = colons - begin;

    return prettyFunction.substr(begin, end);
  }
};

}  // namespace gui
