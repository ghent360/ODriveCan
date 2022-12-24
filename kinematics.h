/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include "RobotDefinition.h"

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
