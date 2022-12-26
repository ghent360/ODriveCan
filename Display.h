
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <math.h>

class Display {
public:
  void initDisplay();
  void stopDisplay();
  void updateScreen();

  void setTeensyBatteryVoltage(float v) {
    if (fabsf(v - teensy_battery_voltage_) < 0.01) {
      teensy_battery_voltage_ = v;
      dirty_ = true;
    }
  }

  void setBus1BatteryVoltage(float v) {
    if (fabsf(v - bus1_battery_voltage_) < 0.01) {
      bus1_battery_voltage_ = v;
      dirty_ = true;
    }
  }

  void setBus3BatteryVoltage(float v) {
    if (fabsf(v - bus3_battery_voltage_) < 0.01) {
      bus3_battery_voltage_ = v;
      dirty_ = true;
    }
  }

  void setCanStatus(const char* msg) {
    if (can_status_ != msg) {
      can_status_ = msg;
      dirty_ = true;
    }
  }
private:
  void drawUi();
  void drawCanStatus();
  
  float teensy_battery_voltage_;
  float bus1_battery_voltage_;
  float bus3_battery_voltage_;
  const char* can_status_;
  bool dirty_;
};
