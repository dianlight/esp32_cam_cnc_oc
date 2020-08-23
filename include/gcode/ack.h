#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the a ack message response.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include <stdio.h>

#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "gcode/core.h"

namespace gcode {

class AckParser : public GCodeParser {
 public:
  OkParser(char *ack) : GCodeParser(ack) {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (std::strcmp(str, name) == 0)
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) {
    GCodeFactory::wait_ack = false;
    return new Ack(str);
  }
};

class Ack : public GCodeResponseData {
 public:
  explicit Ack(char *str) : GCodeResponseData(str) {}
  //  ~GCodeResponseData();
};

// Automatic initialization and register
AckParser grbl_ok_parser("ok");

}  // namespace gcode
