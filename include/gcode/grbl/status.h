#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the a status message response.
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

class StatusCommand : publie GCodeCommand {
 public:
  StatusCommand() : GCodeCommand("?") {}
}

class StatusParser : public GCodeParser {
 public:
  StatusParser() : GCodeParser("?") {}
  //  ~StatusParser() {}

  bool isLineParsable(char *str) {
    if (str[0] == '<' && str[sizeof(str) - 2] == '>')
      return true;
    else
      return false;
  }

  GCodeResponseData *make(char *str) { return new Status(str); }
};

class Status : public GCodeResponseData {
 public:
  typedef struct {
    enum {
      GRBL_UNKNOWN,
      GRBL_SLEEP,
      GRBL_IDLE,
      GRBL_RUN,
      GRBL_HOLD,
      GRBL_JOG,
      GRBL_ALARM,
      GRBL_DOOR,
      GRBL_CHECK,
      GRBL_HOME,
    } status;
    float x, y, z;                   // Current Position         MPos:
    float xco, yco, zco;             // Work Coordinate Offset   WCO:
    uint16_t bf, bfmax;              // Buffer State             Bf:
    uint32_t ln;                     // Line Number              Ln:
    uint16_t fr;                     // Feed Rate                F: o FS:
    uint16_t speed;                  // Spindle Speed            FS:
    bool limitXYZ[3];                // Pin State                Pn:XYZ
    bool limitP;                     // Pin State                Pn:P
    bool pinDoor;                    // Pin State                Pn:D
    bool pinHold;                    // Pin State                Pn:H
    bool pinReset;                   // Pin State                Pn:R
    bool pinStart;                   // Pin State                Pn:S
    uint8_t ofeed, orapids, ospeed;  // Override value     Ov:
    enum {                           // Accessory state          A:
      SPINDLE_OFF,
      SPINDLE_CW,
      SPINDLE_CCW
    } spindle;
    bool flood;  // Accessory state          A:
    bool mist;   // Accessory state          A:
  } status_handle_t;

  explicit Status(char *str) : GCodeResponseData(str) {
    parsingStatusMessage(str);
  }
  //  ~GCodeResponseData();

  status_handle_t getStatus() { return status_data; }

 private:
  status_handle_t status_data;

  bool nextParam(uint8_t *cursor, char *line, char *name, char args[3][10]) {
    for (int y = 0; y < 3; y++) memset(&args[y], 0x00, sizeof(args[y]));
    char pstr[255];
    int r = sscanf(&line[*cursor], "%255[^|>]", pstr);
    sscanf(pstr, "%10[^:]:%10[^,],%10[^,],%10[^,]", name, args[0], args[1],
           args[2]);
    (*cursor) += strlen(pstr) + 1;
    return r != -1;
  }

  // Parsing GRBL status message <...>
  void parsingStatusMessage(char *data) {
    uint8_t cursor = 0;
    char name[10];
    char argv[3][10] = {0};
    while (nextParam(&cursor, data + 1, name, argv)) {
      //            ESP_LOGD(TAG,"Param %d %s \n\tArgs:",cursor,name);
      //            for(int v=0;v < 3 && argv[v][0] != 0x00;v++){
      //                ESP_LOGD(TAG,"[%s]",argv[v]);
      //            }

      // Status
      if (strcmp(name, "Idle") == 0) {
        status_data.status = status_handle_t::GRBL_IDLE;
      } else if (strcmp(name, "Run") == 0) {
        status_data.status = status_handle_t::GRBL_RUN;
      } else if (strcmp(name, "Hold") == 0) {
        status_data.status = status_handle_t::GRBL_HOLD;
        //        if (argv[0][0] == '0') {  // Hold completed
        //          status_data.status_blink = false;
        //        } else {  // Hold in progress
        //          status_data.status_blink = true;
        //       }
      } else if (strcmp(name, "Home") == 0) {
        status_data.status = status_handle_t::GRBL_HOME;
      } else if (strcmp(name, "Jog") == 0) {
        status_data.status = status_handle_t::GRBL_JOG;
      } else if (strcmp(name, "Alarm") == 0) {
        status_data.status = status_handle_t::GRBL_ALARM;
      } else if (strcmp(name, "Door") == 0) {
        status_data.status = status_handle_t::GRBL_HOLD;
        //        if (argv[0][0] == '0') {  // Door completed
        //          status_data.status_blink = false;
        //        } else {  // Door in progress 1,2,3 (@see
        //                  //
        //                  https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface#interacting-with-grbls-systems)
        //          status_data.status_blink = true;
        //        }
      } else if (strcmp(name, "Check") == 0) {
        status_data.status = status_handle_t::GRBL_CHECK;
      } else if (strcmp(name, "Sleep") == 0) {
        status_data.status = status_handle_t::GRBL_SLEEP;
      } else if (strcmp(name, "MPos") == 0) {  // MPos - Current Position
        status_data.x = atof(argv[0]);
        status_data.y = atof(argv[1]);
        status_data.z = atof(argv[2]);
      } else if (strcmp(name, "WCO") == 0) {  // WCO - Work Coordinate Offset
        status_data.xco = atof(argv[0]);
        status_data.yco = atof(argv[1]);
        status_data.zco = atof(argv[2]);
      } else if (strcmp(name, "Bf") == 0) {  // Bf - Buffer State
        status_data.bf = atoi(argv[0]);
        status_data.bfmax = atoi(argv[1]);
      } else if (strcmp(name, "Ln") == 0) {  // Ln - Line number
        status_data.ln = atoi(argv[0]);
      } else if (strcmp(name, "F") == 0) {  // F FS - Current Feed and Speed
        status_data.fr = atoi(argv[0]);
      } else if (strcmp(name, "FS") == 0) {
        status_data.fr = atoi(argv[0]);
        status_data.speed = atoi(argv[1]);
      } else if (strcmp(name, "Pn") == 0) {  // Pn - Input Pin State
        status_data.limitXYZ[0] = false;     // Pin State Pn:XYZ
        status_data.limitXYZ[1] = false;     // Pin State Pn:XYZ
        status_data.limitXYZ[2] = false;     // Pin State Pn:XYZ
        status_data.limitP = false;          // Pin State                Pn:P
        status_data.pinDoor = false;         // Pin State                Pn:D
        status_data.pinHold = false;         // Pin State                Pn:H
        status_data.pinReset = false;        // Pin State                Pn:R
        status_data.pinStart = false;        // Pin State                Pn:S

        for (int c = 0; c < strlen(argv[0]); c++) {
          switch (argv[0][c]) {
            case 'X':  // Pin State                Pn:XYZ
              status_data.limitXYZ[0] = true;
              break;
            case 'Y':  // Pin State                Pn:XYZ
              status_data.limitXYZ[1] = true;
              break;
            case 'Z':  // Pin State                Pn:XYZ
              status_data.limitXYZ[2] = true;
              break;
            case 'P':  // Pin State                Pn:P
              status_data.limitP = true;
              break;
            case 'D':
              status_data.pinDoor = true;
              break;
            case 'H':
              status_data.pinHold = true;
              break;
            case 'R':
              status_data.pinReset = true;
              break;
            case 'S':
              status_data.pinStart = true;
              break;
          }
        }
      } else if (strcmp(name, "Ov") == 0) {  // Ov - Override Values
        status_data.ofeed = atoi(argv[0]);
        status_data.orapids = atoi(argv[1]);
        status_data.ospeed = atoi(argv[2]);
      } else if (strcmp(name, "A") == 0) {  // A - Accessory State
        status_data.spindle = status_handle_t::SPINDLE_OFF;
        status_data.flood = false;
        status_data.mist = false;
        for (int c = 0; c < strlen(argv[0]); c++) {
          switch (argv[0][c]) {
            case 'S':  // S indicates spindle is enabled in the CW direction.
                       // This does not appear with C.
              status_data.spindle = status_handle_t::SPINDLE_CW;
              break;
            case 'C':  // C indicates spindle is enabled in the CCW direction.
                       // This does not appear with S.
              status_data.spindle = status_handle_t::SPINDLE_CCW;
              break;
            case 'F':  // F indicates flood coolant is enabled.
              status_data.flood = true;
              break;
            case 'M':  // M indicates mist coolant is enabled.
              status_data.mist = true;
              break;
          }
        }
      }
    }
  }
};

// Automatic initialization and register
StatusParser grbl_status_parser();
StatusCommand grbl_status_command();

}  // namespace gcode
