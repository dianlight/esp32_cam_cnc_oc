#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the a state $I message command and response.
 *
 * [VER:] : Indicates build info and string from a $I user query.
 * [VER:1.1d.20161014:]
 * [OPT:,15,128]
 * [VER:1.1h.20190830:TEST]
 * [OPT:V,15,128]
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

class InfoCommand : public GCodeCommand {
 public:
  InfoCommand() : GCodeCommand("$I") {}
}

class InfoParser : public GCodeParser {
 public:
  InfoParser() : GCodeParser("$I") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (strstr(str, "[VER:") == str || strstr(str, "[OPT:") == str) {
      return true;
    } else {
      return false;
    }
  }

  GCodeResponseData *make(char *str) { return new Info(str); }
};

class Info : public GCodeResponseData {
 public:
  typedef struct {
    float version;   // Current Version 1.1
    char patch;      // Current Version f
    uint32_t build;  // 20190830
    char info[64];

    bool V;       //  Variable spindle enabled
    bool N;       //  Line numbers enabled
    bool M;       //  Mist coolant enabled
    bool C;       //  CoreXY enabled
    bool P;       //  Parking motion enabled
    bool Z;       //  Homing force origin enabled
    bool H;       //  Homing single axis enabled
    bool T;       //  Two limit switches on axis enabled
    bool A;       //  Allow feed rate overrides in probe cycles
    bool star;    //  * Restore all EEPROM disabled
    bool dollar;  //  $ Restore EEPROM $ settings disabled
    bool sharp;   //  # Restore EEPROM parameter data disabled
    bool I;       //  Build info write user string disabled
    bool E;       //  Force sync upon EEPROM write disabled
    bool W;       //  Force sync upon work coordinate offset change disabled
    bool L;  //  Homing init lock sets Grbl into an alarm state upon power up

    uint16_t blockBufferSize;
    uint16_t rxBufferSize;
  } info_handle_t;

  explicit Info(char *str) : GCodeResponseData(str) { parsingInfoMessage(str); }

  info_handle_t getStatus() { return info_data; }

 private:
  info_handle_t info_data;

  // Parsing GRBL info message <...>
  void parsingInfoMessage(char *data) {
    if (strstr(data, "[VER:") == str) {
      sscanf(data, "[VER:%f%c.%" PRIu16 ":%63[^]]", &info_data.version,
             &info_data.patch, &info_data.build, info_data.info);
    } else if (strstr(data, "[OPT:") == str) {
      char opt[25];
      sscanf(data, "[%*[^,],%" PRIu16 ",%" PRIu16, &info_data.blockBufferSize,
             &info_data.rxBufferSize);
      sscanf(data, "[OPT:%25[^,]", opt);

      for (int c = 0; c < strlen(opt); c++) {
        switch (opt[c]) {
          case 'V':  //  Variable spindle enabled
            info_data.V = true;
            break;
          case 'N':  //  Line numbers enabled
            info_data.N = true;
            break;
          case 'M':  //  Mist coolant enabled
            info_data.M = true;
            break;
          case 'C':  //  CoreXY enabled
            info_data.C = true;
            break;
          case 'P':  //  Parking motion enabled
            info_data.P = true;
            break;
          case 'Z':  //  Homing force origin enabled
            info_data.Z = true;
            break;
          case 'H':  //  Homing single axis enabled
            info_data.H = true;
            break;
          case 'T':  //  Two limit switches on axis enabled
            info_data.T = true;
            break;
          case 'A':  //  Allow feed rate overrides in probe cycles
            info_data.A = true;
            break;
          case '*':  //  * Restore all EEPROM disabled
            info_data.star = true;
            break;
          case '$':  //  $ Restore EEPROM $ settings disabled
            info_data.dollar = true;
            break;
          case '#':  //  # Restore EEPROM parameter data disabled
            info_data.sharp = true;
            break;
          case 'I':  //  Build info write user string disabled
            info_data.I = true;
            break;
          case 'E':  //  Force sync upon EEPROM write disabled
            info_data.E = true;
            break;
          case 'W':  //  Force sync upon work coordinate offset change disabled
            info_data.W = true;
            break;
          case 'L':  //  Homing init lock sets Grbl into an alarm state upon power up
            info_data.L = true;
            break;
        }
      }
    }
  }
}

// Automatic initialization and register
InfoParser grbl_info_parser();
InfoCommand grbl_info_command();
}  // namespace gcode
