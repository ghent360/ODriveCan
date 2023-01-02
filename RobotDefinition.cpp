/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "RobotDefinition.h"
#include "globals.h"

using odrive::ODriveAxis;
using odrive::CanInterface;

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
  [FRONT_RIGHT_KNEE] = ODriveAxis( 8, CanInterface::sendCmdCh3),
  [FRONT_LEFT_KNEE] = ODriveAxis( 7, CanInterface::sendCmdCh1),
  [BACK_RIGHT_KNEE] = ODriveAxis( 6, CanInterface::sendCmdCh3),
  [BACK_LEFT_KNEE] = ODriveAxis( 5, CanInterface::sendCmdCh1),
  [FRONT_RIGHT_SHOULDER] = ODriveAxis( 4, CanInterface::sendCmdCh3),
  [FRONT_LEFT_SHOULDER] = ODriveAxis( 3, CanInterface::sendCmdCh1),
  [BACK_RIGHT_SHOULDER] = ODriveAxis(12, CanInterface::sendCmdCh3),
  [BACK_LEFT_SHOULDER] = ODriveAxis(11, CanInterface::sendCmdCh1),
  [FRONT_RIGHT_HIP] = ODriveAxis( 2, CanInterface::sendCmdCh3),
  [FRONT_LEFT_HIP] = ODriveAxis( 1, CanInterface::sendCmdCh1),
  [BACK_RIGHT_HIP] = ODriveAxis( 10, CanInterface::sendCmdCh3),
  [BACK_LEFT_HIP] = ODriveAxis( 9, CanInterface::sendCmdCh1)
};

DogLegJoint getJointByAxisId(uint16_t axisId) {
  for(uint8_t idx = 0; idx <= numAxes; idx++) {
    if (axes[idx].node_id == axisId) {
      return DogLegJoint(idx);
    }
  }
  return DogLegJoint(-1);
}

const AxisClass jointClass[numAxes] = {
  [FRONT_RIGHT_KNEE] = AxisClass::CLASS_KNEE,
  [FRONT_LEFT_KNEE] = AxisClass::CLASS_KNEE,
  [BACK_RIGHT_KNEE] = AxisClass::CLASS_KNEE,
  [BACK_LEFT_KNEE] = AxisClass::CLASS_KNEE,
  [FRONT_RIGHT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [FRONT_LEFT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [BACK_RIGHT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [BACK_LEFT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [FRONT_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [FRONT_LEFT_HIP] = AxisClass::CLASS_HIP,
  [BACK_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [BACK_LEFT_HIP] = AxisClass::CLASS_HIP
};

#define STRINGIFY(x) #x

const char* axisName[numAxes] = {
  [FRONT_RIGHT_KNEE] = STRINGIFY(FRONT_RIGHT_KNEE),
  [FRONT_LEFT_KNEE] = STRINGIFY(FRONT_LEFT_KNEE),
  [BACK_RIGHT_KNEE] = STRINGIFY(BACK_RIGHT_KNEE),
  [BACK_LEFT_KNEE] = STRINGIFY(BACK_LEFT_KNEE),
  [FRONT_RIGHT_SHOULDER] = STRINGIFY(FRONT_RIGHT_SHOULDER),
  [FRONT_LEFT_SHOULDER] = STRINGIFY(FRONT_LEFT_SHOULDER),
  [BACK_RIGHT_SHOULDER] = STRINGIFY(BACK_RIGHT_SHOULDER),
  [BACK_LEFT_SHOULDER] = STRINGIFY(BACK_LEFT_SHOULDER),
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
  [FRONT_RIGHT_KNEE] = 2.519,
  [FRONT_LEFT_KNEE] = -2.116,
  [BACK_RIGHT_KNEE] = -2.639,
  [BACK_LEFT_KNEE] = 2.05,
  [FRONT_RIGHT_SHOULDER] = 0.713,
  [FRONT_LEFT_SHOULDER] = -0.98,
  [BACK_RIGHT_SHOULDER] = -0.84,
  [BACK_LEFT_SHOULDER] = 0.855,
  [FRONT_RIGHT_HIP] = -0.444,
  [FRONT_LEFT_HIP] = 0.299,
  [BACK_RIGHT_HIP] = -0.286,
  [BACK_LEFT_HIP] = 0.277
};

odrive::CanInterface canInterface(axes, numAxes);