/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once
#include "RobotDefinition.h"

void driveJoints(DogLegJoint joint, float pos);
float getJointPos(DogLegJoint joint);
float getJointVel(DogLegJoint joint);
float getJointTorque(DogLegJoint joint);
float getJointCurrent(DogLegJoint joint);

#ifndef ARDUINO
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
