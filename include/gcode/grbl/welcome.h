#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the welcome message.
 * 
 * Grbl X.Xx ['$' for help] : Welcome message indicates initialization.
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

class WelcomeParser : public GCodeParser {
 public:
  WelcomeParser() : GCodeParser("W") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (str[0] == 'G' && str[sizeof(str) - 2] == ']')
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) { return new Welcome(str); }
};

class Welcome : public GCodeResponseData {
 public:
  typedef struct {
    uint8_t major;  // Current Version 1
    uint8_t minor;  // Current Version 1
    uint8_t patch;  // Current Version f
  } status_handle_t;

  explicit Welcome(char *str) : GCodeResponseData(str) {
    parsingWelcomeMessage(str);
  }
  //  ~GCodeResponseData();

  status_handle_t getStatus() { return status_data; }

 private:
  status_handle_t status_data;
  /*
    bool nextParam(uint8_t *cursor, char *line, char *name, char args[3][10]) {
      for (int y = 0; y < 3; y++) memset(&args[y], 0x00, sizeof(args[y]));
      char pstr[255];
      int r = sscanf(&line[*cursor], "%255[^|>]", pstr);
      sscanf(pstr, "%10[^:]:%10[^,],%10[^,],%10[^,]", name, args[0], args[1],
             args[2]);
      (*cursor) += strlen(pstr) + 1;
      return r != -1;
    }
  */
  // Parsing GRBL status message <...>
  void parsingWelcomeMessage(char *data) {
    sscanf(data, "Grbl %c.%c%c [", &status_data.major, &status_data.minor,
           &status_data.patch);
  }
};

// Automatic initialization and register
WelcomeParser grbl_welcome_parser();

}  // namespace gcode
