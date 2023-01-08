/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "RobotDefinition.h"
#include "globals.h"
#include "Kinematics.h"
#include "JointDriver.h"

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

Leg::Leg(DogLeg legId)
  : leg_id_(legId),
    x_(20),
    y_(107),
    z_(-330),
    hip_axis_(legToAxis[legId][0]),
    tie_axis_(legToAxis[legId][1]),
    shin_axis_(legToAxis[legId][2]),
    posShinAngle_((legId == BACK_RIGHT) || (legId == FRONT_RIGHT)),
    reverseX_((legId == BACK_LEFT) || (legId == FRONT_LEFT)) {
}

bool Leg::startMove() {
  float ha, ta, sa;
  inverseKinematics(
    reverseX_ ? -x_ : x_,
    y_,
    z_,
    posShinAngle_,
    ha,
    ta,
    sa);
  if (!isnan(sa) && !isnan(ta) && !isnan(ha)) {
    driveJoints(hip_axis_, -ha * radToPos);
    driveJoints(tie_axis_, ta * radToPos);
    driveJoints(shin_axis_, sa * radToPos);
    return true;
  }
  return false;
}

float Leg::getPosError() const {
  // Not implemented yet.
  return 0;
}

RobotBody::RobotBody()
  : legs_ {
    [FRONT_LEFT] = Leg(FRONT_LEFT),
    [FRONT_RIGHT] = Leg(FRONT_RIGHT),
    [BACK_LEFT] = Leg(BACK_LEFT),
    [BACK_RIGHT] = Leg(BACK_RIGHT),
  } {}

void RobotBody::parkLegs() {
  for (int idx=0; idx<numAxes; idx++) {
    if (axes[idx].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      driveJoints(static_cast<DogLegJoint>(idx), parkPosition[idx]);
    } 
  }
}

