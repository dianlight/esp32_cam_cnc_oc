#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the startup message.
 * >G54G20:ok
 * >G54G20:error:X
 * >:error:7 --> EPROM Read Error
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

class StartupParser : public GCodeParser {
 public:
  StartupParser() : GCodeParser(">") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (strstr(str, ">G54G20:") == str || strstr(str, ">:error:7") == str)
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) { return new Startup(str); }
};

class Startup : public GCodeResponseData {
 public:
  typedef enum {
    kOK = 'o',
    kEpromError = '7',
    // From Alarm
    kHardLimit = '1',
    kGcodeLimit = '2',
    kLostStep = '3',
    kProbeFailInitialState = '4',
    kProbeFail = '5',
    kHomeFail = '6',
    //    kHomeFailDoorOpen = '7',
    kHomeFailClearLimit = '8',
    kHomeFailLimitSearch = '9'
  } alarm_code_t;

  explicit Startup(char *str) : GCodeResponseData(str) {
    parsingStartupMessage(str);
  }
  //  ~GCodeResponseData();

  alarm_code_t getStatus() { return status_data; }

 private:
  alarm_code_t status_data;

  // Parsing GRBL status message <...>
  void parsingStatusMessage(char *data) {
    if (strstr(str, ">:error:7") == str)
      status_data = kEpromError;
    else if (strstr(str, ">G54G20:ok") == str)
      status_data = kOK;
    else
      sscanf(data, ">G54G20:error:%c",
             reinterpret_cast<uint8_t *>(&status_data));
  }
};

// Automatic initialization and register
StartupParser grbl_startup_parser();

}  // namespace gcode
