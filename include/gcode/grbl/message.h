#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the message.
 * [MSG:] : Indicates a non-queried feedback message.
 *
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

class MessageParser : public GCodeParser {
 public:
  MessageParser() : GCodeParser("[") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (strstr(str, "[MSG:") == str && str[sizeof(str) - 2] == ']')
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) { return new Message(str); }
};

class Message : public GCodeResponseData {
 public:
  explicit Message(char *str) : GCodeResponseData(str) { parsingMessage(str); }
  //  ~GCodeResponseData();

  char *getMessage() { return message; }

 private:
  char message[255];

  // Parsing GRBL  message
  void parsingMessage(char *data) { sscanf(data, "[MSG:%255[^]]", message); }
};

// Automatic initialization and register
MessageParser grbl_message_parser();

}  // namespace gcode
