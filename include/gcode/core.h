#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the a generic message gcode.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/
#include <freertos/task.h>

#include <cstdarg>
#include <map>
#include <string>
#include <vector>

namespace gcode {

class GCodeFactory {
 public:
  typedef struct {
    std::string name;
    GCodeCommand *command;
    GCodeParser *parser;
  } gcode_spec_t;

  static GCodeResponseData *parseLine(char *str) {
    for (std::map<std::string, gcode_spec_t>::iterator it = line_types.begin();
         it != line_types.end(); ++it) {
      if ((it->second.parser)->isLineParsable(str))
        return (it->second.parser)->make(str);
    }
    // Generic GCodeResponseData
    return new GCodeResponseData(str);
  }

  static void sendCommand(std::string name, ...) {
    GCodeCommand *command = line_types[name].command;
    if (command == nullptr) {
      command = line_types[name].command = new GCodeCommand(name.c_str());
    }
    va_list args;
    va_start(args, name);
    printf("%s\r", command->compositeCommand(args).c_str());
    va_end(args);
  }

  static void sendCommandAndWait(std::string name, ...) {
    GCodeCommand *command = line_types[name].command;
    if (command == nullptr) {
      command = line_types[name].command = new GCodeCommand(name.c_str());
    }
    va_list args;
    va_start(args, name);
    printf("%s\r", command->compositeCommand(args).c_str());
    va_end(args);
    wait_ack = true;
    while (!wait_ack) vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  static void registerGCodeResponseParser(std::string name,
                                          GCodeParser *parser) {
    line_types[name].parser = parser;
  }

  static void registerGCodeCommand(std::string name, GCodeCommand *command) {
    line_types[name].command = command;
  }

  static void unregisterGCode(std::string name) { line_types.erase(name); }

  static bool wait_ack;

 private:
  static std::map<std::string, gcode_spec_t> line_types;
};

class GCodeParser {
 public:
  explicit GCodeParser(std::string name): name(name) {
    GCodeFactory::registerGCodeResponseParser(name, this);
  }
  //  ~GCodeParser() { GCodeFactory::unregisterLineParser(this); }

  virtual bool isLineParsable(char *str) = 0;
  virtual GCodeResponseData *make(char *str) = 0;
 private:
  std:string name;
};

class GCodeResponseData {
 public:
  explicit GCodeResponseData(char *str) : lineData(str) {}
  ~GCodeResponseData();

 private:
  char *lineData;
};

class GCodeCommand {
 public:
  explicit GCodeCommand(const char *str) : lineCommand(str) {}
  ~GCodeCommand();

  std::string compositeCommand(va_list argv) { return lineCommand; }

 private:
  const char *lineCommand;
};

}  // namespace gcode
