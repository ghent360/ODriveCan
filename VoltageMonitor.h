/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

// Monitor the battery voltage for the Teensy 4.1 board.
// If voltage gets bellow critical point switch off as many
// peripherals as we can and switch the Teensy to low power mode.
class VoltageMonitor {
public:
  void initVoltageMonitor();
  float readBatteryVoltage();
  void lowPowerMode();
private:
  float cvtMult_;
};