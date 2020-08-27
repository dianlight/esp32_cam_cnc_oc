#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the alarm message.
 * ALARM:x : Indicates an alarm has been thrown. Grbl is now in an alarm state.
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

class AlarmParser : public GCodeParser {
 public:
  WelcomeParser() : GCodeParser("A") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (strstr(str,"ALARM:") == str) 
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) { return new Alert(str); }
};

class Alarm : public GCodeResponseData {
 public:
  typedef enum {
    kHardLimit = '1',
    kGcodeLimit = '2',
    kLostStep = '3',
    kProbeFailInitialState = '4',
    kProbeFail = '5',
    kHomeFail = '6',
    kHomeFailDoorOpen = '7',
    kHomeFailClearLimit = '8',
    kHomeFailLimitSearch = '9'
  } alarm_code_t;

  explicit Welcome(char *str) : GCodeResponseData(str) {
    parsingWelcomeMessage(str);
  }
  //  ~GCodeResponseData();

  alarm_code_t getStatus() { return status_data; }

 private:
  alarm_code_t status_data;
 
  // Parsing GRBL status message <...>
  void parsingWelcomeMessage(char *data) {
    sscanf(data, "ALARM:%" PRIu8, reinterpret_cast<uint8_t *>(&status_data));
  }
};

// Automatic initialization and register
AlarmParser grbl_alarm_parser();

}  // namespace gcode
