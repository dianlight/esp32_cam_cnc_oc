#pragma once
/**
 * @file
 *
 * @copyright Copyright (c) 2020 Lucio Tarantino. All Rights Reserved.
 *
 * @license This file is distributed under the MIT License. See LICENSE.TXT for
 *details.
 *
 * @brief This header file defines the GuiPageWithStatusbar virtual class.
 *
 * @author lucio.tarantino@gmail.com (Lucio Tarantino).
 *
 * @project ESP32_CAM_CNC_OC
 *
 **/

#include <esp_event.h>
#include <u8g2.h>

#include <string>

#include "gui/gui_page.h"

namespace gui {

class GuiPageWithStatusbar : public GuiPage {
 public:
  GuiPageWithStatusbar();
  ~GuiPageWithStatusbar();

  void renderPage(u8g2_t* u8g2);
  void enterPage();
  void exitPage();

 private:
  //  uint8_t menupos = 0, selmenupos = 0;
  //  int8_t selmenu_dir = 0;
  bool status_blink, wifi_blink;
  bool wifi, tcp_serial, bluetooth_serial, webcam;

  int64_t last_mainPage_millis = 0;
  bool blink = false;

  esp_event_handler_t status_event_handler;

  enum class Status : uint8_t {
    kUnknown,
    kSleep,
    kIdle,
    kRun,
    kHold,
    kJog,
    kHome,
    kCheck,
    kDoor,
    kAlarm
  };

  Status status;

  enum class Icon : uint8_t {
    kStatus = 0,
    kWebcam = 7,
    kTcpSerial = 12,
    kBluetoothSerial = 13,
    kWiFi = 14
  };

  // IconBar On/Op1 | Op2 | Op2 | Op3 | Op4 | Op5 | Op6 | Op7 | Op8 | Op9 | Op10
  // | Off  Icon
  static constexpr uint16_t kIconBar[15][11] = {
      {0x00BC, 0x00DF, 0x00EE, 0x00E9, 0x00D2, 0x00E5, 0x0059, 0x0134, 0x00CB,
       0x0118, 0x0000},  // GRBL_UNKNOWN | GRBL_SLEEP | GRBL_IDLE | GRBL_RUN |
                         // GRBL_HOLD | GRBL_JOG | GRBL_HOME | GRBL_CHECK |
                         // GRBL_DOOR | GRBL_ALARM |
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0114, 0x0000},  // Webcam Streaming
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x0000, 0x0000},  //
      {0x00DE, 0x0000},  // TCP/IP (Serial)
      {0x005E, 0x0000},  // Blutooth (Serial)
      {0x00F7, 0x0000},  // WiFi
  };

  void StatusEventHandler(void* handler_args, esp_event_base_t event_base,
                          int32_t event_id, void* event_data);
};

}  // namespace gui
