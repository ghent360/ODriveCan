/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "RadioController.h"
#include "RobotDefinition.h"
#include "globals.h"
#include "Kinematics.h"
#include "JointDriver.h"
#include "TaskIds.h"

#define MOVE_PREP
#define MOVE_WALK
#define MOVE_END
//#define DEBUG_PREP
//#define DEBUG_WALK
//#define DEBUG_END

using odrive::AxisState;
using odrive::CanInterface;
using odrive::ODriveAxis;

// We communicate with 6 ODrive boards, each board has 2 axes. Each axis
// has to be setup separately. If you have boards that are connected to a
// different CAN bus, you would need separate CAN callback for each bus.
// For example SendCmdCh1.
//
// It is required to have unique node_id even if some axis are on a
// separate CAN bus. Otherwise the parsing code has to be modified to 
// process each CAN bus separately.
//
// On my setup the axes have even node_ids starting with 1. These have to
// be configured using the odrivetool.
ODriveAxis axes[numAxes] = {
  [FRONT_RIGHT_SHIN] = ODriveAxis( 8, CanInterface::sendCmdCh3),
  [FRONT_LEFT_SHIN] = ODriveAxis( 7, CanInterface::sendCmdCh1),
  [BACK_RIGHT_SHIN] = ODriveAxis( 6, CanInterface::sendCmdCh3),
  [BACK_LEFT_SHIN] = ODriveAxis( 5, CanInterface::sendCmdCh1),
  [FRONT_RIGHT_TIE] = ODriveAxis( 4, CanInterface::sendCmdCh3),
  [FRONT_LEFT_TIE] = ODriveAxis( 3, CanInterface::sendCmdCh1),
  [BACK_RIGHT_TIE] = ODriveAxis(12, CanInterface::sendCmdCh3),
  [BACK_LEFT_TIE] = ODriveAxis(11, CanInterface::sendCmdCh1),
  [FRONT_RIGHT_HIP] = ODriveAxis( 2, CanInterface::sendCmdCh3),
  [FRONT_LEFT_HIP] = ODriveAxis( 1, CanInterface::sendCmdCh1),
  [BACK_RIGHT_HIP] = ODriveAxis( 10, CanInterface::sendCmdCh3),
  [BACK_LEFT_HIP] = ODriveAxis( 9, CanInterface::sendCmdCh1)
};

// The canInterface instance depends on the axes object.
odrive::CanInterface canInterface(axes, numAxes);

DogLegJoint getJointByAxisId(uint16_t axisId) {
  for(uint8_t idx = 0; idx <= numAxes; idx++) {
    if (axes[idx].node_id == axisId) {
      return DogLegJoint(idx);
    }
  }
  return DogLegJoint(-1);
}

const AxisClass jointClass[numAxes] = {
  [FRONT_RIGHT_SHIN] = AxisClass::CLASS_SHIN,
  [FRONT_LEFT_SHIN] = AxisClass::CLASS_SHIN,
  [BACK_RIGHT_SHIN] = AxisClass::CLASS_SHIN,
  [BACK_LEFT_SHIN] = AxisClass::CLASS_SHIN,
  [FRONT_RIGHT_TIE] = AxisClass::CLASS_TIE,
  [FRONT_LEFT_TIE] = AxisClass::CLASS_TIE,
  [BACK_RIGHT_TIE] = AxisClass::CLASS_TIE,
  [BACK_LEFT_TIE] = AxisClass::CLASS_TIE,
  [FRONT_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [FRONT_LEFT_HIP] = AxisClass::CLASS_HIP,
  [BACK_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [BACK_LEFT_HIP] = AxisClass::CLASS_HIP
};

#define STRINGIFY(x) #x

const char* axisName[numAxes] = {
  [FRONT_RIGHT_SHIN] = STRINGIFY(FRONT_RIGHT_SHIN),
  [FRONT_LEFT_SHIN] = STRINGIFY(FRONT_LEFT_SHIN),
  [BACK_RIGHT_SHIN] = STRINGIFY(BACK_RIGHT_SHIN),
  [BACK_LEFT_SHIN] = STRINGIFY(BACK_LEFT_SHIN),
  [FRONT_RIGHT_TIE] = STRINGIFY(FRONT_RIGHT_TIE),
  [FRONT_LEFT_TIE] = STRINGIFY(FRONT_LEFT_TIE),
  [BACK_RIGHT_TIE] = STRINGIFY(BACK_RIGHT_TIE),
  [BACK_LEFT_TIE] = STRINGIFY(BACK_LEFT_TIE),
  [FRONT_RIGHT_HIP] = STRINGIFY(FRONT_RIGHT_HIP),
  [FRONT_LEFT_HIP] = STRINGIFY(FRONT_LEFT_HIP),
  [BACK_RIGHT_HIP] = STRINGIFY(BACK_RIGHT_HIP),
  [BACK_LEFT_HIP] = STRINGIFY(BACK_LEFT_HIP)
};

const char* getLegName(DogLeg leg) {
  switch(leg) {
  case FRONT_LEFT: return STRINGIFY(FRONT_LEFT);
  case FRONT_RIGHT: return STRINGIFY(FRONT_RIGHT);
  case BACK_LEFT: return STRINGIFY(BACK_LEFT);
  case BACK_RIGHT: return STRINGIFY(BACK_RIGHT);
  }
  return "";
}

/*
Straight down pos (shin, tie, hip):
Front Right:  2.519,  0.713, -0.444
Front Left : -2.116, -0.98,   0.299
Back Right : -2.639, -0.84,  -0.286
Back Left  :  2.05,   0.855,  0.277
*/
const float jointOffsets[numAxes] = {
  [FRONT_RIGHT_SHIN] = 2.519,
  [FRONT_LEFT_SHIN] = -2.116,
  [BACK_RIGHT_SHIN] = -2.639,
  [BACK_LEFT_SHIN] = 2.05,
  [FRONT_RIGHT_TIE] = 0.713,
  [FRONT_LEFT_TIE] = -0.98,
  [BACK_RIGHT_TIE] = -0.84,
  [BACK_LEFT_TIE] = 0.855,
  [FRONT_RIGHT_HIP] = -0.444,
  [FRONT_LEFT_HIP] = 0.299,
  [BACK_RIGHT_HIP] = -0.286,
  [BACK_LEFT_HIP] = 0.277
};

const float parkPosition[numAxes] = {
  [FRONT_RIGHT_SHIN] = 2.391,
  [FRONT_LEFT_SHIN] = -2.336,
  [BACK_RIGHT_SHIN] = -2.323,
  [BACK_LEFT_SHIN] = 2.37,
  [FRONT_RIGHT_TIE] = -0.996,
  [FRONT_LEFT_TIE] = 0.935,
  [BACK_RIGHT_TIE] = 0.985,
  [BACK_LEFT_TIE] = -0.98,
  [FRONT_RIGHT_HIP] = 0,
  [FRONT_LEFT_HIP] = 0,
  [BACK_RIGHT_HIP] = 0,
  [BACK_LEFT_HIP] = 0
};

static constexpr DogLegJoint legToAxis[numLegs][3] = {
  [FRONT_LEFT] = {FRONT_LEFT_HIP, FRONT_LEFT_TIE, FRONT_LEFT_SHIN},
  [FRONT_RIGHT] = {FRONT_RIGHT_HIP, FRONT_RIGHT_TIE, FRONT_RIGHT_SHIN},
  [BACK_LEFT] = {BACK_LEFT_HIP, BACK_LEFT_TIE, BACK_LEFT_SHIN},
  [BACK_RIGHT] = {BACK_RIGHT_HIP, BACK_RIGHT_TIE, BACK_RIGHT_SHIN},
};

RobotLeg::RobotLeg(DogLeg legId)
  : leg_id_(legId),
    x_(0),
    y_(107),
    z_(-350),
    hip_axis_(legToAxis[legId][0]),
    tie_axis_(legToAxis[legId][1]),
    shin_axis_(legToAxis[legId][2]),
    positive_shin_angle_((legId == BACK_RIGHT) || (legId == FRONT_RIGHT)),
    reverse_x_((legId == BACK_LEFT) || (legId == FRONT_LEFT)) {
}

bool RobotLeg::startMove() {
  float ha, ta, sa;
  inverseKinematics_new(
    reverse_x_ ? -x_ : x_,
    y_,
    z_,
    positive_shin_angle_,
    ha,
    ta,
    sa);
  if (!std::isnan(sa) && !std::isnan(ta) && !std::isnan(ha)) {
    driveJoints(hip_axis_, -ha * radToPos);
    driveJoints(tie_axis_, ta * radToPos);
    driveJoints(shin_axis_, sa * radToPos);
    return true;
  }
  return false;
}

bool RobotLeg::allAxesActive() const {
  return
    axes[hip_axis_].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL &&
    axes[tie_axis_].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL &&
    axes[shin_axis_].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
}

void RobotLeg::calcPosFromAxis(float &x, float &y, float &z) const {
  float ha, ta, sa;
  ha = -(getJointPos(hip_axis_) * posToRad);
  ta = getJointPos(tie_axis_) * posToRad;
  sa = getJointPos(shin_axis_) * posToRad;
  forwardKinematics(ha, ta, sa, x, y, z);
  if (reverse_x_) {
    x = -x;
  }
}

void RobotLeg::calcVelFromAxis(float &vx, float &vy, float &vz) const {
  float ha, ta, sa;
  ha = -(getJointPos(hip_axis_) * posToRad);
  ta = getJointPos(tie_axis_) * posToRad;
  sa = getJointPos(shin_axis_) * posToRad;

  float wh, wt, ws;
  wh = -(getJointVel(hip_axis_) * posToRad);
  wt = getJointVel(tie_axis_) * posToRad;
  ws = getJointVel(shin_axis_) * posToRad;

  forwardVelocities(ha, ta, sa, wh, wt, ws, vx, vy, vz);
  if (reverse_x_) {
    vx = -vx;
  }
}

void RobotLeg::calcAccFromAxis(float &ax, float &ay, float &az) const {
  float ha, ta, sa;
  ha = -(getJointPos(hip_axis_) * posToRad);
  ta = getJointPos(tie_axis_) * posToRad;
  sa = getJointPos(shin_axis_) * posToRad;

  float wh, wt, ws;
  wh = -(getJointVel(hip_axis_) * posToRad);
  wt = getJointVel(tie_axis_) * posToRad;
  ws = getJointVel(shin_axis_) * posToRad;

  float ah, at, as;
  ah = -getJointTorque(hip_axis_);
  at = getJointTorque(tie_axis_);
  as = getJointTorque(shin_axis_);

  forwardAcceleration(ha, ta, sa, wh, wt, ws, ah, at, as, ax, ay, az);
  if (reverse_x_) {
    ax = -ax;
  }
}

void RobotLeg::calcStandingAccFromAxis(float &ax, float &ay, float &az) const {
  float ha, ta, sa;
  ha = -(getJointPos(hip_axis_) * posToRad);
  ta = getJointPos(tie_axis_) * posToRad;
  sa = getJointPos(shin_axis_) * posToRad;

  float ah, at, as;
  ah = -getJointTorque(hip_axis_);
  at = getJointTorque(tie_axis_);
  as = getJointTorque(shin_axis_);

  forwardStandingAcceleration(ha, ta, sa, ah, at, as, ax, ay, az);
  if (reverse_x_) {
    ax = -ax;
  }
}

float RobotLeg::getPosError() const {
  float x, y, z;
  calcPosFromAxis(x, y, z);
  x -= x_;
  y -= y_;
  z -= z_;
  return (x * x) + (y * y) + (z * z);
}

RobotBody::RobotBody()
  : axes_active_(false),
    state_(STATE_IDLE),
    prep_step_height_(15),
    prep_step_duration_(500),
    walk_step_duration_(1500),
    legs_ {
      [FRONT_LEFT] = RobotLeg(FRONT_LEFT),
      [FRONT_RIGHT] = RobotLeg(FRONT_RIGHT),
      [BACK_LEFT] = RobotLeg(BACK_LEFT),
      [BACK_RIGHT] = RobotLeg(BACK_RIGHT),
  } {
  updateReferencePos(FRONT_LEFT);
  updateReferencePos(FRONT_RIGHT);
  updateReferencePos(BACK_LEFT);
  updateReferencePos(BACK_RIGHT);
}

void RobotBody::parkLegs() {
  if (!isActiveAndIdle()) return;
  for (int idx=0; idx<numAxes; idx++) {
    if (axes[idx].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      driveJoints(static_cast<DogLegJoint>(idx), parkPosition[idx]);
    } 
  }
  // Wait 1 sec for the axes to complete the move.
  scheduleRecalculateLogPosition(1000);
}

void RobotBody::modifyAxesGains() {
  constexpr float posGainShin = 35.0f;
  constexpr float posGainHips = 40.0f;
  constexpr float posGainTie = 35.0f;
  constexpr float velGain = 0.1f;
  constexpr float integrator = 0.2f;
  float posGain = 35.0f;
  for (int idx=0; idx<numAxes; idx++) {
    switch(jointClass[idx]) {
      case CLASS_HIP:
        posGain = posGainHips;
        break;
      case CLASS_SHIN:
        posGain = posGainShin;
        break;
      case CLASS_TIE:
        posGain = posGainTie;
        break;
    }
    axes[idx].SetPosGain(posGain);
    axes[idx].SetVelGains(velGain, integrator);
  }
}

void RobotBody::setAllAxesActive() {
  for(auto& axis: axes) {
    axis.SetLimits(8.0f, 15.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  radioController.setMotorState(true);
  axes_active_ = true;
  scheduleRecalculateLogPosition(100);
  taskManager.removeById(StateThreeReportStandingAccl);
  taskManager.addBack(taskManager.newPeriodicTask(
    StateThreeReportStandingAccl,
    500,
    [](TaskNode*, uint32_t) {
      float v;
      v = robotBody.getLegAccZ(FRONT_LEFT);
      Serial.print("FL ");
      Serial.print(v);
      v = robotBody.getLegAccZ(FRONT_RIGHT);
      Serial.print(" FR ");
      Serial.print(v);
      v = robotBody.getLegAccZ(BACK_LEFT);
      Serial.print(" BL ");
      Serial.print(v);
      v = robotBody.getLegAccZ(BACK_RIGHT);
      Serial.print(" BR ");
      Serial.println(v);
    }));
}

void RobotBody::setAllAxesIdle() {
  taskManager.removeById(StateThreeReportStandingAccl);
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
  radioController.setMotorState(false);
  axes_active_ = false;
}

void RobotBody::scheduleRecalculateLogPosition(uint32_t delay_ms) {
  // remove previous task instance
  taskManager.removeById(RobotBodyRecalcLegPos);

  // Recalculate position in 500ms
  taskManager.addBack(
    taskManager.newSimpleTask(
      RobotBodyRecalcLegPos, delay_ms, [](TaskNode*, uint32_t) {
      robotBody.recalculateLegPositions();
    }));
}

void RobotBody::init() {
  state_ = STATE_IDLE;
  axes_active_ =
    legs_[FRONT_LEFT].allAxesActive() &&
    legs_[FRONT_RIGHT].allAxesActive() &&
    legs_[BACK_LEFT].allAxesActive() &&
    legs_[BACK_RIGHT].allAxesActive();
  radioController.setMotorState(axes_active_);
  if (axes_active_) {
    scheduleRecalculateLogPosition(250);
  }
}

bool RobotBody::planStepStart(DogLeg legId) {
  float gait_x, gait_z;
  walk_trajectory_.gaitPos(gait_plan_.leg_offset_[legId], gait_x, gait_z);
  gait_x -= leg_reference_pos_[legId][0];
  if (gait_x == 0) return false;
  prep_trajectory_.setStepParams(prep_step_height_, gait_x, 0);
  prep_step_half_size_ = gait_x / 2;
  prep_step_duration_ = fabsf(gait_x) * 10;
  if (prep_step_duration_ < 20)
    prep_step_duration_ = 20;
#ifdef DEBUG_PREP
  Serial.print(getLegName(legId));
  Serial.print(" pos X: ");
  Serial.print(leg_reference_pos_[legId][0]);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2]);
  Serial.print("Prep ");
  Serial.print(getLegName(legId));
  Serial.print(" step: ");
  Serial.println(gait_x);
#endif
  return true;
}

void RobotBody::moveStepStart(DogLeg legId, float t) {
  float gait_x, gait_z;
  prep_trajectory_.swingState2D(t, gait_x, gait_z);
#ifdef DEBUG_PREP
  Serial.print(getLegName(legId));
  Serial.print(" prep X: ");
  Serial.print(leg_reference_pos_[legId][0] + gait_x + prep_step_half_size_);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2] + gait_z);
#endif
#ifdef MOVE_PREP
  legs_[legId].setPos(
    leg_reference_pos_[legId][0] + gait_x + prep_step_half_size_,
    leg_reference_pos_[legId][1],
    leg_reference_pos_[legId][2] + gait_z
  );
#endif
}

void RobotBody::finishStepStart(DogLeg legId) {
  float gait_x, gait_z;
  walk_trajectory_.gaitPos(gait_plan_.leg_offset_[legId], gait_x, gait_z);
#ifdef DEBUG_PREP
  Serial.print(getLegName(legId));
  Serial.print(" prep end. Adjusting X: ");
  Serial.print(gait_x);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2] + gait_z);
#endif
  leg_reference_pos_[legId][0] = 0;
#ifdef MOVE_PREP
  legs_[legId].setPos(
    leg_reference_pos_[legId][0] + gait_x,
    leg_reference_pos_[legId][1],
    leg_reference_pos_[legId][2] + gait_z
  );
#endif
}

void RobotBody::moveLegWalk(DogLeg legId, float t) {
  float gait_x, gait_z;
  t += gait_plan_.leg_offset_[legId];
  if (t > 1) t -= 1;
  walk_trajectory_.gaitPos(t, gait_x, gait_z);
#ifdef DEBUG_WALK
  Serial.print(getLegName(legId));
  Serial.print(" X: ");
  Serial.print(leg_reference_pos_[legId][0] + gait_x);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2] + gait_z);
#endif
#ifdef MOVE_WALK
  legs_[legId].setPos(
    leg_reference_pos_[legId][0] + gait_x,
    leg_reference_pos_[legId][1],
    leg_reference_pos_[legId][2] + gait_z
  );
#endif
}

void RobotBody::moveWalk(float t) {
  for(uint8_t idx = 0; idx < numberOfLegs; idx++) {
    moveLegWalk(DogLeg(idx), t);
  }
}

bool RobotBody::planStepEnd(DogLeg legId) {
  float gait_x;
  gait_x = legs_[legId].getPosX() - leg_reference_pos_[legId][0];
  if (gait_x == 0) return false;
  prep_trajectory_.setStepParams(prep_step_height_, gait_x, 0);
  prep_step_half_size_ = gait_x / 2;
  prep_step_duration_ = fabsf(gait_x) * 10;
  if (prep_step_duration_ < 20)
    prep_step_duration_ = 20;
#ifdef DEBUG_END
  Serial.print(getLegName(legId));
  Serial.print(" pos X: ");
  Serial.print(legs_[legId].getPosX());
  Serial.print(" Z: ");
  Serial.println(legs_[legId].getPosZ());
  Serial.print("Prep ");
  Serial.print(getLegName(legId));
  Serial.print(" step: ");
  Serial.println(gait_x);
#endif
  return true;
}

void RobotBody::moveStepEnd(DogLeg legId, float t) {
  float gait_x, gait_z;
  prep_trajectory_.swingState2D(1.0f-t, gait_x, gait_z);
#ifdef DEBUG_END
  Serial.print(getLegName(legId));
  Serial.print(" end X: ");
  Serial.print(leg_reference_pos_[legId][0] + gait_x + prep_step_half_size_);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2] + gait_z);
#endif
#ifdef MOVE_END
  legs_[legId].setPos(
    leg_reference_pos_[legId][0] + gait_x + prep_step_half_size_,
    leg_reference_pos_[legId][1],
    leg_reference_pos_[legId][2] + gait_z
  );
#endif
}

void RobotBody::finishStepEnd(DogLeg legId) {
#ifdef DEBUG_END
  Serial.print(getLegName(legId));
  Serial.print(" finish end. Adjusting X: ");
  Serial.print(leg_reference_pos_[legId][0]);
  Serial.print(" Z: ");
  Serial.println(leg_reference_pos_[legId][2]);
#endif
#ifdef MOVE_END
  legs_[legId].setPos(
    leg_reference_pos_[legId][0],
    leg_reference_pos_[legId][1],
    leg_reference_pos_[legId][2]
  );
#endif
}

void RobotBody::startWalking() {
  if (!isActiveAndIdle()) return;
  setCreepGaitPlan();
#if 0
  // Level all 4 legs on the Z axis
  int16_t body_height = 0;
  for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
    body_height += leg_reference_pos_[idx][2];
  }
  body_height += numberOfLegs / 2;
  body_height /= numberOfLegs;
  for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
    leg_reference_pos_[idx][2] = body_height;
  }
#endif
  state_ = STATE_PREPARE_FRONT_LEFT;
  step_start_ = millis();
  if (!planStepStart(FRONT_LEFT)) {
    step_start_ -= prep_step_duration_;
  }
  radioController.setWalkState(true);
  taskManager.addBack(
    taskManager.newPeriodicTask(
      RobotBodyStateExecutor, 5, [](TaskNode*, uint32_t now){
        robotBody.runState(now);
      }));
}

void RobotBody::stopWalking() {
  state_ = STATE_FINALIZE_FRONT_LEFT;
  step_start_ = millis();
  if (!planStepEnd(FRONT_LEFT)) {
    step_start_ -= prep_step_duration_;
  }
}

void RobotBody::runState(uint32_t now) {
  if (state_ == STATE_IDLE) {
    radioController.setWalkState(false);
    taskManager.removeById(RobotBodyStateExecutor);
    return;
  }
  uint32_t elapsed = now - step_start_;
  float t;
  switch(state_) {
  case STATE_PREPARE_FRONT_LEFT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepStart(FRONT_LEFT);
      state_ = STATE_PREPARE_BACK_RIGHT;
      step_start_ = now;
      if (!planStepStart(BACK_RIGHT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepStart(FRONT_LEFT, t);
    }
  }
  break;
  case STATE_PREPARE_BACK_RIGHT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepStart(BACK_RIGHT);
      state_ = STATE_PREPARE_FRONT_RIGHT;
      step_start_ = now;
      if (!planStepStart(FRONT_RIGHT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepStart(BACK_RIGHT, t);
    }
  }
  break;
  case STATE_PREPARE_FRONT_RIGHT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepStart(FRONT_RIGHT);
      state_ = STATE_PREPARE_BACK_LEFT;
      step_start_ = now;
      if (!planStepStart(BACK_LEFT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepStart(FRONT_RIGHT, t);
    }
  }
  break;
  case STATE_PREPARE_BACK_LEFT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepStart(BACK_LEFT);
      state_ = STATE_EXECUTE_GAIT;
      step_start_ = now;
    } else {
      moveStepStart(BACK_LEFT, t);
    }
  }
  break;
  case STATE_EXECUTE_GAIT: {
    t = float(elapsed) / walk_step_duration_;
    if (t > 1) {
      t = 0;
      step_start_ = now;
    }
    moveWalk(t);
  }
  break;
  case STATE_FINALIZE_FRONT_LEFT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepEnd(FRONT_LEFT);
      state_ = STATE_FINALIZE_BACK_RIGHT;
      step_start_ = now;
      if (!planStepEnd(BACK_RIGHT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepEnd(FRONT_LEFT, t);
    }
  }
  break;
  case STATE_FINALIZE_BACK_RIGHT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepEnd(BACK_RIGHT);
      state_ = STATE_FINALIZE_FRONT_RIGHT;
      step_start_ = now;
      if (!planStepEnd(FRONT_RIGHT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepEnd(BACK_RIGHT, t);
    }
  }
  break;
  case STATE_FINALIZE_FRONT_RIGHT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepEnd(FRONT_RIGHT);
      state_ = STATE_FINALIZE_BACK_LEFT;
      step_start_ = now;
      if (!planStepEnd(BACK_LEFT)) {
        step_start_ -= prep_step_duration_;
      }
    } else {
      moveStepEnd(FRONT_RIGHT, t);
    }
  }
  break;
  case STATE_FINALIZE_BACK_LEFT: {
    t = float(elapsed) / prep_step_duration_;
    if (t > 1) {
      finishStepEnd(BACK_LEFT);
      state_ = STATE_IDLE;
      radioController.setWalkState(false);
      taskManager.removeById(RobotBodyStateExecutor);
    } else {
      moveStepEnd(BACK_LEFT, t);
    }
  }
  break;
  default:
    state_ = STATE_IDLE;
    radioController.setWalkState(false);
    taskManager.removeById(RobotBodyStateExecutor);
    break;
  }
}

void RobotBody::printPositions() const {
  for(uint8_t idx = 0; idx < numberOfLegs; idx++) {
    Serial.println(getLegName(DogLeg(idx)));
    Serial.print("  Pos X: ");
    Serial.print(legs_[idx].getPosX());
    Serial.print(" Y: ");
    Serial.print(legs_[idx].getPosY());
    Serial.print(" Z: ");
    Serial.println(legs_[idx].getPosZ());
    Serial.print("  Ref X: ");
    Serial.print(leg_reference_pos_[idx][0]);
    Serial.print(" Y: ");
    Serial.print(leg_reference_pos_[idx][1]);
    Serial.print(" Z: ");
    Serial.println(leg_reference_pos_[idx][2]);
  }
}

RobotBody robotBody;
