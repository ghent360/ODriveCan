/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include "JointDriver.h"
#include "ODriveCan.hpp"
#include "globals.h"

using odrive::AxisState;

static float constrainJointPos(DogLegJoint joint, float pos) {
    switch(jointClass[joint]) {
    case AxisClass::CLASS_SHIN:
        return constrain(pos, -3.1, 3.1);
        break;
    case AxisClass::CLASS_TIE:
        return constrain(pos, -2.4, 2.4);
        break;
    case AxisClass::CLASS_HIP:
        return constrain(pos, -0.6, 0.8);
    }
    return pos;
}

void driveJoints(DogLegJoint joint, float pos) {
    // takes into account the original setup offsets for motor 
    // positions, and also turns around directions so they are
    // consistent, also constrains the motion limits for each joint

    pos = constrainJointPos(joint, pos);
    switch (joint) {
    case BACK_RIGHT_SHIN:
    case FRONT_RIGHT_SHIN:
    case FRONT_RIGHT_HIP:
    case FRONT_LEFT_SHIN:
    case BACK_LEFT_SHIN:
    case BACK_LEFT_HIP:
        pos = -pos;
        break;
    default:
        break;
    }
    if (axes[joint].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      axes[joint].SetInputPos(pos + jointOffsets[joint]);
    }
}

float getJoinPos(DogLegJoint joint) {
    float pos = axes[joint].enc_est.pos - jointOffsets[joint];
    switch (joint) {
    case BACK_RIGHT_SHIN:
    case FRONT_RIGHT_SHIN:
    case FRONT_RIGHT_HIP:
    case FRONT_LEFT_SHIN:
    case BACK_LEFT_SHIN:
    case BACK_LEFT_HIP:
        pos = -pos;
        break;
    default:
        break;
    }
    return pos;
}