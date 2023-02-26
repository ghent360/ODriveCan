/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "globals.h"

// The Radio.* code implements generic radio receiver class. This class contains
// the OpenDog specific protocol handling and logic.
class RadioController {
public:
  RadioController();

  void processRxData(const uint8_t *rxData, uint8_t len);

  void setReady(bool);
  void reportAxisError(uint16_t axisCanId, uint32_t error);
  void reportMotorError(uint16_t axisCanId, uint64_t error);
  void reportEncoderError(uint16_t axisCanId, uint32_t error);
private:
  bool ready_;
  uint32_t axis_errors_[numAxes];
  uint64_t axis_motor_errors_[numAxes];
  uint32_t axis_encoder_errors_[numAxes];
};

extern RadioController radioController;
