/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "RobotDefinition.h"
#include "globals.h"
#include "Kinematics.h"
#include "JointDriver.h"
#include "TaskIds.h"

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
  [BACK_RIGHT_HIP] = -0,
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
    x_(20),
    y_(107),
    z_(-330),
    hip_axis_(legToAxis[legId][0]),
    tie_axis_(legToAxis[legId][1]),
    shin_axis_(legToAxis[legId][2]),
    positive_shin_angle_((legId == BACK_RIGHT) || (legId == FRONT_RIGHT)),
    reverse_x_((legId == BACK_LEFT) || (legId == FRONT_LEFT)) {
}

bool RobotLeg::startMove() {
  float ha, ta, sa;
  inverseKinematics(
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

void RobotLeg::calcPosFromAxis(float &x, float &y, float &z) const {
  float ha, ta, sa;
  ha = (axes[hip_axis_].enc_est.pos - jointOffsets[hip_axis_]) * posToRad;
  ta = (axes[tie_axis_].enc_est.pos - jointOffsets[tie_axis_]) * posToRad;
  sa = (axes[shin_axis_].enc_est.pos - jointOffsets[shin_axis_]) * posToRad;
  forwardKinematics(ha, ta, sa, x, y, z);
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
  : legs_ {
    [FRONT_LEFT] = RobotLeg(FRONT_LEFT),
    [FRONT_RIGHT] = RobotLeg(FRONT_RIGHT),
    [BACK_LEFT] = RobotLeg(BACK_LEFT),
    [BACK_RIGHT] = RobotLeg(BACK_RIGHT),
  } {}

void RobotBody::parkLegs() {
  for (int idx=0; idx<numAxes; idx++) {
    if (axes[idx].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      driveJoints(static_cast<DogLegJoint>(idx), parkPosition[idx]);
    } 
  }
  // Wait 1 sec for the axes to complete the move.
  scheduleRecalculateLogPosition(1000);
}

void RobotBody::modifyAxisGains() {
  constexpr float posGainShin = 20.0f;
  constexpr float posGainHips = 60.0f;
  constexpr float posGainTie = 20.0f;
  constexpr float velGain = 0.1f;
  constexpr float integrator = 0.2f;
  float posGain = 20.0f;
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

void RobotBody::setAxisActive() {
  for(auto& axis: axes) {
    axis.SetLimits(20.0f, 10.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  scheduleRecalculateLogPosition(100);
}

void RobotBody::setAxisIdle() {
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
}

void RobotBody::scheduleRecalculateLogPosition(uint32_t delay_ms) {
  // remove previous task instance
  tm.remove(tm.findById(RebotBodyRecalsLegPos), true);

  // Recalculate position in 500ms
  tm.newSimpleTask(RebotBodyRecalsLegPos, delay_ms, [](TaskNode*, uint32_t) {
    robotBody.recalculateLegPositions();
  });
}

RobotBody robotBody;
