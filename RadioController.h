/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "Fixed.hpp"
#include "globals.h"
#include "Swithes.h"
#include "ValueWithChangeDetection.hpp"

// The Radio.* code implements generic radio receiver class. This class contains
// the OpenDog specific protocol handling and logic.
class RadioController {
public:
  RadioController();

  void processRxData(const uint8_t *rxData, uint8_t len);

  void setReady(bool);

  void setMotorState(bool v);
  void setWalkState(bool v) {
    walk_engaged_ = v;
  }

  void setB1Voltage(float v) {
    b1_voltage_ = v;
  }

  void setB2Voltage(float v) {
    b2_voltage_ = v;
  }

  void setRXVoltage(float v) {
    rx_voltage_ = v;
  }

  void reportAxisError(uint16_t axisCanId, uint32_t error);
  void reportMotorError(uint16_t axisCanId, uint64_t error);
  void reportEncoderError(uint16_t axisCanId, uint32_t error);
  void reportAxisIq(uint16_t axisCanId, float iqSetpoint);
private:
  void setNextTxPacket();

  using BatteryVoltage6S = Fixed<uint8_t, 8, 5, 18>;
  using BatteryVoltage2S = Fixed<uint8_t, 8, 6, 6>;

  bool ready_;
  bool motors_engaged_;
  bool walk_engaged_;
  ValueWithChangeDetection<uint32_t> axis_errors_[numAxes];
  ValueWithChangeDetection<uint64_t> axis_motor_errors_[numAxes];
  ValueWithChangeDetection<uint32_t> axis_encoder_errors_[numAxes];
  BatteryVoltage6S b1_voltage_;
  BatteryVoltage6S b2_voltage_;
  BatteryVoltage2S rx_voltage_;
  SW3POS sw1_;
  SW3POS sw2_;
  SW2POS sw5_;
  float axis_iq_desired_[numAxes];
};

extern RadioController radioController;
