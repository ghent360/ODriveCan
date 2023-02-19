/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <stdint.h>
#include "can_helpers.hpp"

namespace odrive {

// ODrive uses CAN2.0. Message data payload is max 8 bytes long.
typedef uint8_t CanMsgData[8];

enum ODriveCanCMD {
  MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
  MSG_ODRIVE_HEARTBEAT,
  MSG_ODRIVE_ESTOP,
  MSG_GET_MOTOR_ERROR,
  MSG_GET_ENCODER_ERROR,
  MSG_GET_SENSORLESS_ERROR,
  MSG_SET_AXIS_NODE_ID,
  MSG_SET_AXIS_REQUESTED_STATE,
  MSG_SET_AXIS_STARTUP_CONFIG,
  MSG_GET_ENCODER_ESTIMATES,
  MSG_GET_ENCODER_COUNT,
  MSG_SET_CONTROLLER_MODES,
  MSG_SET_INPUT_POS,
  MSG_SET_INPUT_VEL,
  MSG_SET_INPUT_TORQUE,
  MSG_SET_LIMITS,
  MSG_START_ANTICOGGING,
  MSG_SET_TRAJ_VEL_LIMIT,
  MSG_SET_TRAJ_ACCEL_LIMITS,
  MSG_SET_TRAJ_INERTIA,
  MSG_GET_IQ,
  MSG_GET_SENSORLESS_ESTIMATES,
  MSG_RESET_ODRIVE,
  MSG_GET_VBUS_VOLTAGE,
  MSG_CLEAR_ERRORS,
  MSG_SET_LINEAR_COUNT,
  MSG_SET_POS_GAIN,
  MSG_SET_VEL_GAINS,
  MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
};

enum AxisState {
  AXIS_STATE_UNDEFINED                         = 0,
  AXIS_STATE_IDLE                              = 1,
  AXIS_STATE_STARTUP_SEQUENCE                  = 2,
  AXIS_STATE_FULL_CALIBRATION_SEQUENCE         = 3,
  AXIS_STATE_MOTOR_CALIBRATION                 = 4,
  AXIS_STATE_ENCODER_INDEX_SEARCH              = 6,
  AXIS_STATE_ENCODER_OFFSET_CALIBRATION        = 7,
  AXIS_STATE_CLOSED_LOOP_CONTROL               = 8,
  AXIS_STATE_LOCKIN_SPIN                       = 9,
  AXIS_STATE_ENCODER_DIR_FIND                  = 10,
  AXIS_STATE_HOMING                            = 11,
  AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
  AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION    = 13,
};

enum ControlMode {
  CONTROL_MODE_VOLTAGE_CONTROL     = 0,
  CONTROL_MODE_TORQUE_CONTROL      = 1,
  CONTROL_MODE_VELOCITY_CONTROL    = 2,
  CONTROL_MODE_POSITION_CONTROL    = 3,
};

enum InputMode {
  INPUT_MODE_INACTIVE              = 0,
  INPUT_MODE_PASSTHROUGH           = 1,
  INPUT_MODE_VEL_RAMP              = 2,
  INPUT_MODE_POS_FILTER            = 3,
  INPUT_MODE_MIX_CHANNELS          = 4,
  INPUT_MODE_TRAP_TRAJ             = 5,
  INPUT_MODE_TORQUE_RAMP           = 6,
  INPUT_MODE_MIRROR                = 7,
  INPUT_MODE_TUNING                = 8,
};

class ODriveAxis;

// The following structures represent several CAN messages 
// that we can receive from the ODrive board, each structure
// has public data members that receive the decoded data from
// the message and a callback to be invoked when the message
// is decoded.

// This is the hearthbeat message, that is send every 100ms 
// by default for each axis. In addition to the message data
// the structure is enhanced with 'alive' flag and 'unreachable'
// callback to be called when communication with the axis is
// lost.
struct Heartbeat {
  bool     alive;
  uint32_t error;
  uint8_t  state; // AxisState enum above
  // Bit 0 - controller error flag
  // Bit 7 - trajectory complete flag
  uint8_t  ctrl_state;

  enum Error {
    ERROR_NONE                        = 0x00000000,
    ERROR_INVALID_STATE               = 0x00000001,
    ERROR_MOTOR_FAILED                = 0x00000040,
    ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080,
    ERROR_ENCODER_FAILED              = 0x00000100,
    ERROR_CONTROLLER_FAILED           = 0x00000200,
    ERROR_WATCHDOG_TIMER_EXPIRED      = 0x00000800,
    ERROR_MIN_ENDSTOP_PRESSED         = 0x00001000,
    ERROR_MAX_ENDSTOP_PRESSED         = 0x00002000,
    ERROR_ESTOP_REQUESTED             = 0x00004000,
    ERROR_HOMING_WITHOUT_ENDSTOP      = 0x00020000,
    ERROR_OVER_TEMP                   = 0x00040000,
    ERROR_UNKNOWN_POSITION            = 0x00080000,
  };

  typedef void (*Callback)(
    ODriveAxis& axis, Heartbeat &old, Heartbeat &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis,
    uint8_t cmdId,
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_ODRIVE_HEARTBEAT && dataLen == 8) {
      Heartbeat newHb;
      newHb.error = can_getSignal<uint32_t>(msg, 0, 32, true);
      newHb.state = can_getSignal<uint8_t>(msg, 32, 8, true);
      //newHb.motor_flags = can_getSignal<uint8_t>(msg, 40, 8, true);
      //newHb.encoder_flags = can_getSignal<uint8_t>(msg, 48, 8, true);
      newHb.ctrl_state = can_getSignal<uint8_t>(msg, 56, 8, true);
      newHb.alive = true;
      newHb.alive_hb_ = true;
      if (cb_) {
        cb_(axis, *this, newHb);
      }
      alive = newHb.alive;
      alive_hb_ = newHb.alive_hb_;
      error = newHb.error;
      state = newHb.state;
      ctrl_state = newHb.ctrl_state;
      return true;
    }
    return false;
  }

  typedef void (*UnreachableCallback)(ODriveAxis& axis);
  void SetUnreachableCallback(const UnreachableCallback cb) {
    unreachable_cb_ = cb;
  }
  
  // Must be called every 150ms or so to check if we have received a
  // heartbeat CAN message from this axis. Heartbeat messages are sent
  // every 100ms by default.
  void PeriodicCheck(ODriveAxis& axis) {
    if (!alive_hb_) {
      if (alive) {
        if (unreachable_cb_) {
          unreachable_cb_(axis);
        }
      }
      alive = false;
    }
    alive_hb_ = false;
  }
private:
  bool     alive_hb_;
  Callback cb_;
  UnreachableCallback unreachable_cb_;
};

// The encoder estimates message is sent every 10ms by default.
struct EncoderEstimate {
  float pos;
  float vel;

  typedef void (*Callback)(
    ODriveAxis& axis, EncoderEstimate &old, EncoderEstimate &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }
   
  bool Parse(
    ODriveAxis& axis,
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_ENCODER_ESTIMATES && dataLen == 8) {
      EncoderEstimate newEst;
      newEst.pos = can_getSignal<float>(msg, 0, 32, true);
      newEst.vel = can_getSignal<float>(msg, 32, 32, true);
      if (cb_) {
        cb_(axis, *this, newEst);
      }
      pos = newEst.pos;
      vel = newEst.vel;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct EncoderCount {
  int32_t shadowCount;
  int32_t countInCPR;

  typedef void (*Callback)(
    ODriveAxis& axis, EncoderCount &old, EncoderCount &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }
   
  bool Parse(
    ODriveAxis& axis,
    uint8_t cmdId,
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_ENCODER_COUNT && dataLen == 8) {
      EncoderCount newEcnt;
      newEcnt.shadowCount = can_getSignal<int32_t>(msg, 0, 32, true);
      newEcnt.countInCPR = can_getSignal<int32_t>(msg, 32, 32, true);
      if (cb_) {
        cb_(axis, *this, newEcnt);
      }
      shadowCount = newEcnt.shadowCount;
      countInCPR = newEcnt.countInCPR;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct IqValues {
  float iqSetpoint;
  float iqMeasured;

  typedef void (*Callback)(ODriveAxis& axis, IqValues &old, IqValues &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_IQ && dataLen == 8) {
      IqValues newIq;
      newIq.iqSetpoint = can_getSignal<float>(msg, 0, 32, true);
      newIq.iqMeasured = can_getSignal<float>(msg, 32, 32, true);
      if (cb_) {
        cb_(axis, *this, newIq);
      }
      iqSetpoint = newIq.iqSetpoint;
      iqMeasured = newIq.iqMeasured;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct SensorlessEstimates {
  float pos;
  float vel;

  typedef void (*Callback)(
    ODriveAxis& axis, SensorlessEstimates &old, SensorlessEstimates &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_SENSORLESS_ESTIMATES && dataLen == 8) {
      SensorlessEstimates newEst;
      newEst.pos = can_getSignal<float>(msg, 0, 32, true);
      newEst.vel = can_getSignal<float>(msg, 32, 32, true);
      if (cb_) {
        cb_(axis, *this, newEst);
      }
      pos = newEst.pos;
      vel = newEst.vel;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct MotorError {
  uint64_t err;

  enum Error {
    ERROR_NONE                            = 0x00000000,
    ERROR_PHASE_RESISTANCE_OUT_OF_RANGE   = 0x00000001,
    ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE   = 0x00000002,
    ERROR_DRV_FAULT                       = 0x00000008,
    ERROR_CONTROL_DEADLINE_MISSED         = 0x00000010,
    ERROR_MODULATION_MAGNITUDE            = 0x00000080,
    ERROR_CURRENT_SENSE_SATURATION        = 0x00000400,
    ERROR_CURRENT_LIMIT_VIOLATION         = 0x00001000,
    ERROR_MODULATION_IS_NAN               = 0x00010000,
    ERROR_MOTOR_THERMISTOR_OVER_TEMP      = 0x00020000,
    ERROR_FET_THERMISTOR_OVER_TEMP        = 0x00040000,
    ERROR_TIMER_UPDATE_MISSED             = 0x00080000,
    ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
    ERROR_CONTROLLER_FAILED               = 0x00200000,
    ERROR_I_BUS_OUT_OF_RANGE              = 0x00400000,
    ERROR_BRAKE_RESISTOR_DISARMED         = 0x00800000,
    ERROR_SYSTEM_LEVEL                    = 0x01000000,
    ERROR_BAD_TIMING                      = 0x02000000,
    ERROR_UNKNOWN_PHASE_ESTIMATE          = 0x04000000,
    ERROR_UNKNOWN_PHASE_VEL               = 0x08000000,
    ERROR_UNKNOWN_TORQUE                  = 0x10000000,
    ERROR_UNKNOWN_CURRENT_COMMAND         = 0x20000000,
    ERROR_UNKNOWN_CURRENT_MEASUREMENT     = 0x40000000,
    ERROR_UNKNOWN_VBUS_VOLTAGE            = 0x80000000,
    ERROR_UNKNOWN_VOLTAGE_COMMAND         = 0x100000000,
    ERROR_UNKNOWN_GAINS                   = 0x200000000,
    ERROR_CONTROLLER_INITIALIZING         = 0x400000000,
    ERROR_UNBALANCED_PHASES               = 0x800000000,
  };

  typedef void (*Callback)(
    ODriveAxis& axis, MotorError &old, MotorError &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_MOTOR_ERROR && dataLen == 8) {
      MotorError newErr;
      newErr.err = can_getSignal<uint64_t>(msg, 0, 64, true);
      if (cb_) {
        cb_(axis, *this, newErr);
      }
      err = newErr.err;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct EncoderError {
  uint32_t err;

  enum Error {
    ERROR_NONE                       = 0x00000000,
    ERROR_UNSTABLE_GAIN              = 0x00000001,
    ERROR_CPR_POLEPAIRS_MISMATCH     = 0x00000002,
    ERROR_NO_RESPONSE                = 0x00000004,
    ERROR_UNSUPPORTED_ENCODER_MODE   = 0x00000008,
    ERROR_ILLEGAL_HALL_STATE         = 0x00000010,
    ERROR_INDEX_NOT_FOUND_YET        = 0x00000020,
    ERROR_ABS_SPI_TIMEOUT            = 0x00000040,
    ERROR_ABS_SPI_COM_FAIL           = 0x00000080,
    ERROR_ABS_SPI_NOT_READY          = 0x00000100,
    ERROR_HALL_NOT_CALIBRATED_YET    = 0x00000200,
  };

  typedef void (*Callback)(
    ODriveAxis& axis, EncoderError &old, EncoderError &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_ENCODER_ERROR && dataLen >= 4) {
      EncoderError newErr;
      newErr.err = can_getSignal<uint32_t>(msg, 0, 32, true);
      if (cb_) {
        cb_(axis, *this, newErr);
      }
      err = newErr.err;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct SensorlessError {
  uint32_t err;

  enum Error {
    ERROR_NONE                        = 0x00000000,
    ERROR_UNSTABLE_GAIN               = 0x00000001,
    ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x00000002,
  };
  
  typedef void (*Callback)(
    ODriveAxis& axis, SensorlessError &old, SensorlessError &newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_SENSORLESS_ERROR && dataLen >= 4) {
      SensorlessError newErr;
      newErr.err = can_getSignal<uint32_t>(msg, 0, 32, true);
      if (cb_) {
        cb_(axis, *this, newErr);
      }
      err = newErr.err;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

struct VbusVoltage {
  float val;

  typedef void (*Callback)(
    ODriveAxis& axis, VbusVoltage &old, VbusVoltage& newVal);
  void SetCallback(const Callback cb) {
    cb_ = cb;
  }

  bool Parse(
    ODriveAxis& axis, 
    uint8_t cmdId, 
    uint8_t dataLen,
    const CanMsgData& msg) {
    if (cmdId == MSG_GET_VBUS_VOLTAGE && dataLen >= 4) {
      VbusVoltage newVal;
      newVal.val = can_getSignal<float>(msg, 0, 32, true);
      if (cb_) {
        cb_(axis, *this, newVal);
      }
      val = newVal.val;
      return true;
    }
    return false;
  }
private:
  Callback cb_;
};

// This is the main class for communication with the ODrive. The actual
// sending and receiving of CAN messages is not handled here. Note that
// ODrive uses CAN2.0 with 11 bit identifiers by default.
class ODriveAxis {
public:
  // We use the following callback to send CAN messages. One has to provide
  // a function that interfaces with their CAN hardware.
  typedef void (*const CanSendMsg)(uint32_t msgId, uint8_t len, uint8_t* buf);

  // Each ODrive axis has assigned ID, referred to as node_id. These IDs must
  // be known/set before the CAN communication would work.
  uint32_t            node_id;

  // The following list is structures from the messages we care to decode.
  // You can skip on some of the messages to save code/data, for example
  // The Sensorless messages are not handled (commented out).
  Heartbeat           hb;
  EncoderEstimate     enc_est;
  //EncoderCount        enc_count;
  EncoderError        enc_err;
  //IqValues            iq;
  //SensorlessEstimates sens_est;
  //SensorlessError     sens_err;
  MotorError          mot_err;
  VbusVoltage         vbus;

  ODriveAxis(uint32_t nodeId, CanSendMsg canSend) 
    : node_id(nodeId), can_send_(canSend) {}

  // You must call the Parse method on all messages you wish to handle form
  // the list above. The code does not look pretty, but the compiler
  // optimizes it very well.
  bool Parse(uint8_t cmdId, uint8_t dataLen, const CanMsgData& msg) {
    if (hb.Parse(*this, cmdId, dataLen, msg)) return true;
    if (enc_est.Parse(*this, cmdId, dataLen, msg)) return true;
    //if (enc_count.Parse(*this, cmdId, dataLen, msg)) return true;
    if (enc_err.Parse(*this, cmdId, dataLen, msg)) return true;
    //if (iq.Parse(*this, cmdId, dataLen, msg)) return true;
    //if (sens_est.Parse(*this, cmdId, dataLen, msg)) return true;
    //if (sens_err.Parse(*this, cmdId, dataLen, msg)) return true;
    if (mot_err.Parse(*this, cmdId, dataLen, msg)) return true;
    if (vbus.Parse(*this, cmdId, dataLen, msg)) return true;
    return false;
  }

  // The following methods communicate with the ODrive axis. Some methods
  // send data/commands to the ODrive and some request data. You can get
  // notified when the data arrives, by setting a callback on the messages
  // above.

  void EStop() {
    SendCmd(MSG_ODRIVE_ESTOP);
  }

  void RequestMotorError() {
    SendCmd(MSG_GET_MOTOR_ERROR);
  }

  void RequestEncoderError() {
    SendCmd(MSG_GET_ENCODER_ERROR);
  }

  void RequestSensorlessError() {
    SendCmd(MSG_GET_SENSORLESS_ERROR);
  }

  void SetState(AxisState newState) {
    uint8_t buf[4] = {0};
    can_setSignal<uint32_t>(buf, newState, 0, 32, true);
    SendCmd(MSG_SET_AXIS_REQUESTED_STATE, 4, buf);
  }

  // Encoder estimates are sent every 10ms by default.
  void RequestEncoderEstimates() {
    SendCmd(MSG_GET_ENCODER_ESTIMATES);
  }

  void RequestEncoderCount() {
    SendCmd(MSG_GET_ENCODER_COUNT);
  }

  void SetControlerModes(ControlMode crtlMode, InputMode inputMode) {
    uint8_t buf[8] = {0};
    can_setSignal<int32_t>(buf, crtlMode, 0, 32, true);
    can_setSignal<int32_t>(buf, inputMode, 32, 32, true);
    SendCmd(MSG_SET_CONTROLLER_MODES, 8, buf);
  }

  void SetInputPos(float inputPos, float velFf = 0, float torqueFf = 0) {
    uint8_t buf[8] = {0};
    can_setSignal<float>(buf, inputPos, 0, 32, true);
    can_setSignal<int16_t>(buf, velFf, 32, 16, true, 0.001, 0);
    can_setSignal<int16_t>(buf, torqueFf, 48, 16, true, 0.001, 0);
    SendCmd(MSG_SET_INPUT_POS, 8, buf);
  }

  void SetInputVel(float velocity, float torqueFf = 0) {
    uint8_t buf[8] = {0};
    can_setSignal<float>(buf, velocity, 0, 32, true);
    can_setSignal<float>(buf, torqueFf, 32, 32, true);
    SendCmd(MSG_SET_INPUT_VEL, 8, buf);
  }

  void SetInputTorque(float torque) {
    uint8_t buf[4] = {0};
    can_setSignal<float>(buf, torque, 0, 32, true);
    SendCmd(MSG_SET_INPUT_TORQUE, 4, buf);
  }

  void SetLimits(float velLimit, float currentLimit) {
    uint8_t buf[8] = {0};
    can_setSignal<float>(buf, velLimit, 0, 32, true);
    can_setSignal<float>(buf, currentLimit, 32, 32, true);
    SendCmd(MSG_SET_LIMITS, 8, buf);
  }

  void StartAnticogging() {
    SendCmd(MSG_START_ANTICOGGING);
  }

  void SetTrajVelLimit(float velLimit) {
    uint8_t buf[4] = {0};
    can_setSignal<float>(buf, velLimit, 0, 32, true);
    SendCmd(MSG_SET_TRAJ_VEL_LIMIT, 4, buf);
  }

  void SetTrajAccelLimit(float accelLimit, float decelLimit) {
    uint8_t buf[8] = {0};
    can_setSignal<float>(buf, accelLimit, 0, 32, true);
    can_setSignal<float>(buf, decelLimit, 32, 32, true);
    SendCmd(MSG_SET_TRAJ_ACCEL_LIMITS, 8, buf);
  }

  void SetTrajInertia(float inertia) {
    uint8_t buf[4] = {0};
    can_setSignal<float>(buf, inertia, 0, 32, true);
    SendCmd(MSG_SET_TRAJ_INERTIA, 4, buf);
  }

  void RequestIq() {
    SendCmd(MSG_GET_IQ);
  }
  
  void RequestSensorlessEstimates() {
    SendCmd(MSG_GET_SENSORLESS_ESTIMATES);
  }
  
  void ResetODrive() {
    SendCmd(MSG_RESET_ODRIVE);
  }
  
  void ClearErrors() {
    SendCmd(MSG_CLEAR_ERRORS);
  }
  
  void RequestVbusVoltage() {
    SendCmd(MSG_GET_VBUS_VOLTAGE);
  }

  void SetLinearCount(int32_t linearCount) {
    uint8_t buf[4] = {0};
    can_setSignal<int32_t>(buf, linearCount, 0, 32, true);
    SendCmd(MSG_SET_LINEAR_COUNT, 4, buf);
  }

  void SetPosGain(float posGain) {
    uint8_t buf[4] = {0};
    can_setSignal<float>(buf, posGain, 0, 32, true);
    SendCmd(MSG_SET_POS_GAIN, 4, buf);
  }

  void SetVelGains(float velGain, float velIntegratorGain) {
    uint8_t buf[8] = {0};
    can_setSignal<float>(buf, velGain, 0, 32, true);
    can_setSignal<float>(buf, velIntegratorGain, 32, 32, true);
    SendCmd(MSG_SET_VEL_GAINS, 8, buf);
  }

private:

  void SendCmd(uint8_t cmdId, uint8_t len = 0, uint8_t *buf = NULL) {
    uint32_t canId = (node_id << 5) | (cmdId & 0x1f);
    if (can_send_) {
      can_send_(canId, len, buf);
    }
  }

  CanSendMsg can_send_;
};

} // namespace odrive