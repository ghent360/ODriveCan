/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */


#include "RadioController.h"
#include "RemoteProtocol.h"
#include "RobotDefinition.h"
#include <string.h>

RadioController::RadioController()
  : ready_(false) {
}

void RadioController::setReady(bool value) {
  if (ready_ != value && value) {
    for(uint8_t idx = 0; idx < numAxes; idx++) {
      axis_errors_[idx] = 0;
      axis_errors_[idx].changeReset();
      axis_motor_errors_[idx] = 0;
      axis_motor_errors_[idx].changeReset();
      axis_encoder_errors_[idx] = 0;
      axis_encoder_errors_[idx].changeReset();
    }
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
  const struct TxDataPacket *tx_packet =
    reinterpret_cast<const struct TxDataPacket*>(rxData);
  if (len >= sizeof(TxDataPacket) && ready_) {
    SW3POS sw1, sw2;
    SW2POS sw5;
    sw1 = SW3POS(tx_packet->state1.sw1);
    sw2 = SW3POS(tx_packet->state1.sw2);
    sw5 = SW2POS(tx_packet->state2.sw5);
    if (sw5 != sw5_) {
      if (sw5 == SW2_ON) {
        robotBody.setAllAxesActive();
      } else {
        robotBody.setAllAxesIdle();
      }
      sw5_ = sw5;
    }
    if (sw2 != sw2_) {
      if (!walk_engaged_) {
        switch(sw2) {
          case SW3_OFF: break; // not implemented
          case SW3_MID: break;
          case SW3_ON:
            if (motors_engaged_) {
              robotBody.parkLegs();
            }
            break;
        }
      }
      sw2_ = sw2;
    }
    if (sw1 != sw1_) {
      if (motors_engaged_) {
        switch (sw1) {
          case SW3_OFF:
            if (sw2_ == SW3_MID && !walk_engaged_) {
              robotBody.startWalking();
            }
            break;
          case SW3_MID:
            if (sw2_ == SW3_MID && walk_engaged_) {
              robotBody.stopWalking();
            }
            break;
          case SW3_ON:
            if (sw2_ == SW3_MID && !walk_engaged_) {
              robotBody.resetAll();
            }
            break;
        }
      }
      sw1_ = sw1;
    }
    if (tx_packet->cmd != CMD_NOOP) {
      switch(tx_packet->cmd) {
        case CMD_CLEAR_ERRORS:
          for(auto& axis: axes) {
            axis.ClearErrors();
          }
          break;
        case CMD_SET_GAINS:
          robotBody.modifyAxesGains();
          break;
        default:
          break;
      }
    }
  }
  setNextTxPacket();
}

void RadioController::setNextTxPacket() {
  struct RxPacket data;
  uint8_t len = sizeof(RxPacketHdr);
  data.hdr.state.reserved = 0;
  data.hdr.state.control = motors_engaged_;
  data.hdr.state.walk = walk_engaged_;
  data.hdr.errors.a1error = axis_errors_[0] != 0;
  data.hdr.errors.a2error = axis_errors_[1] != 0;
  data.hdr.errors.a3error = axis_errors_[2] != 0;
  data.hdr.errors.a4error = axis_errors_[3] != 0;
  data.hdr.errors.a5error = axis_errors_[4] != 0;
  data.hdr.errors.a6error = axis_errors_[5] != 0;
  data.hdr.errors.a7error = axis_errors_[6] != 0;
  data.hdr.errors.a8error = axis_errors_[7] != 0;
  data.hdr.errors.a9error = axis_errors_[8] != 0;
  data.hdr.errors.a10error = axis_errors_[9] != 0;
  data.hdr.errors.a11error = axis_errors_[10] != 0;
  data.hdr.errors.a12error = axis_errors_[11] != 0;
  data.hdr.errors.reserved = 0;
  data.hdr.b1_voltage = *b1_voltage_.to_bytes();
  data.hdr.b2_voltage = *b2_voltage_.to_bytes();
  data.hdr.rx_voltage = *rx_voltage_.to_bytes();
  data.hdr.ext_type = EXT_TYPE_NONE;
  for(uint8_t idx = 0; idx < numAxes; idx++) {
    if (axis_errors_[idx] != 0 && axis_errors_[idx].changed()) {
      len += sizeof(RxAxisError);
      data.hdr.ext_type = EXT_AXIS_ERROR;
      data.ext.axis_err.axis_id = idx;
      data.ext.axis_err.axis_error.error = axis_errors_[idx];
      data.ext.axis_err.motor_error.error = axis_motor_errors_[idx];
      data.ext.axis_err.encoder_error.error = axis_encoder_errors_[idx];
      axis_errors_[idx].changeReset();
      break;
    }
  }
  radio.writeTxData((const uint8_t*)&data, len);
}

RadioController radioController;
