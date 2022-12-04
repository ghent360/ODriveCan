/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>
#include "ODriveCan.hpp"

enum DogLeg {
    FRONT_LEFT = 1,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

// AXIS class definitions:
enum AxisClass {
  CLASS_KNEE = 0x1,
  CLASS_HIP  = 0x2,
  CLASS_SHOULDER = 0x3
};

enum DogLegJoint {
    FRONT_RIGHT_KNEE,
    FRONT_LEFT_KNEE,
    BACK_RIGHT_KNEE,
    BACK_LEFT_KNEE,
    FRONT_RIGHT_SHOULDER,
    FRONT_LEFT_SHOULDER,
    BACK_RIGHT_SHOULDER,
    BACK_LEFT_SHOULDER,
    FRONT_RIGHT_HIP,
    FRONT_LEFT_HIP,
    BACK_RIGHT_HIP,
    BACK_LEFT_HIP
};

constexpr int numAxes = 12;
extern const float jointOffsets[numAxes]; // Index with DogLegJoint value
extern const AxisClass jointClass[numAxes]; // Index with DogLegJoint value
extern odrive::ODriveAxis axes[numAxes]; // Index with DogLegJoint value (defined in CanInterface.cpp)
extern const char* axisName[numAxes]; // Index with DogLegJoint value

const char* getLegName(DogLeg leg);
void driveJoints(DogLegJoint joint, float pos);

void kinematicsInterp(
    DogLeg leg,
    float xIn,
    float yIn,
    float zIn,
    float roll,
    float pitch,
    float yawIn,
    bool interOn,
    int dur);

void kinematics(
    DogLeg leg,
    float x,
    float y,
    float z,
    float roll,
    float pitch,
    float yaw);

extern uint32_t currentMillis;
extern uint32_t previousInterpMillis;

#ifndef ARDUINO
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
