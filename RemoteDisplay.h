/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include "RemoteDisplayWidgets.h"
#include "RemoteMenu.h"
#include <stdint.h>
#include <ILI9341_t3n.h> // Hardware-specific library

class RemoteDisplay {
public:
  RemoteDisplay();

  void initPins();
  void begin();
  void stopDisplay();
  void updateScreen();
  bool busy();

  void setTeensyBatteryVoltage(float v) {
    teensy_battery_.setVoltage(v);
  }

  void setBus1BatteryVoltage(float v) {
    bus1_battery_.setVoltage(v);
  }

  void setBus3BatteryVoltage(float v) {
    bus3_battery_.setVoltage(v);
  }

  void setRadioStatus(const char* msg) {
    radio_status_.setStatus(msg);
  }

  void setRadioStatus(const String& msg) {
    radio_status_.setStatus(msg);
  }

  void setRadioStatusColor(uint16_t color) {
    radio_status_.setColor(color);
  }

  void setSW1Label(const char* v) {
    sw1_.setLabel(v);
  }
  void setSW2Label(const char* v) {
    sw2_.setLabel(v);
  }
  void setSW3Label(const char* v) {
    sw3_.setLabel(v);
  }
  void setSW4Label(const char* v) {
    sw4_.setLabel(v);
  }
  void setSW5Label(const char* v) {
    sw5_.setLabel(v);
  }
  void setSW1Active(bool v) {
    sw1_.activate(v);
  }
  void setSW2Active(bool v) {
    sw2_.activate(v);
  }
  void setSW3Active(bool v) {
    sw3_.activate(v);
  }
  void setSW4Active(bool v) {
    sw4_.activate(v);
  }
  void setSW5Active(bool v) {
    sw5_.activate(v);
  }
  void setMenuActive(bool v) {
    menu_.activate(v);
  }
#if 0
  void setX1(float v) {
    x1_.setValue(v);
  }
  void setY1(float v) {
    y1_.setValue(v);
  }
#endif
  MenuController& controller() {
    return menuController_;
  }
private:
  void drawUi();
  bool dirty() const {
    for(auto widget : widgets_) {
      if (widget->dirty()) return true;
    }
    return false;
  }

  BatteryWidget teensy_battery_;
  BatteryWidget bus1_battery_;
  BatteryWidget bus3_battery_;
  StatusWidget radio_status_;
  ButtonWidget sw1_;
  ButtonWidget sw2_;
  ButtonWidget sw3_;
  ButtonWidget sw4_;
  ButtonWidget sw5_;
#if 0
  HBarWidget x1_;
  VBarWidget y1_;
#endif
  ButtonWidget menu_;
  MenuController menuController_;
  Widget* widgets_[11] = {
    &teensy_battery_,
    &bus1_battery_,
    &bus3_battery_,
    &radio_status_,
    &sw1_,
    &sw2_,
    &sw3_,
    &sw4_,
    &sw5_,
#if 0
    &x1_,
    &y1_,
#endif
    &menu_,
    &menuController_
  };
};

extern RemoteDisplay remoteDisplay;
