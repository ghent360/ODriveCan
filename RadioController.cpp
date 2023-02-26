/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */


#include "RadioController.h"
#include <string.h>

RadioController::RadioController()
  : ready_(false) {
}

void RadioController::setReady(bool value) {
  if (ready_ != value && value) {
    memset(axis_errors_, 0, sizeof(axis_errors_));
    memset(axis_motor_errors_, 0, sizeof(axis_motor_errors_));
    memset(axis_encoder_errors_, 0, sizeof(axis_encoder_errors_));
  }
  ready_ = value;
}

void RadioController::reportAxisError(uint16_t axisCanId, uint32_t error) {
  int8_t axis_idx = (int8_t)getJointByAxisId(axisCanId);
  if (axis_idx >= 0 && axis_idx < numAxes) {
    axis_errors_[axis_idx] = error;
  }
}

void RadioController::reportMotorError(uint16_t axisCanId, uint64_t error) {
  int8_t axis_idx = (int8_t)getJointByAxisId(axisCanId);
  if (axis_idx >= 0 && axis_idx < numAxes) {
    axis_motor_errors_[axis_idx] = error;
  }
}

void RadioController::reportEncoderError(uint16_t axisCanId, uint32_t error) {
  int8_t axis_idx = (int8_t)getJointByAxisId(axisCanId);
  if (axis_idx >= 0 && axis_idx < numAxes) {
    axis_encoder_errors_[axis_idx] = error;
  }
}

void RadioController::processRxData(const uint8_t *rxData, uint8_t len) {
  //TODO: implement
}

RadioController radioController;
