/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include "RobotDefinition.h"
#include "ODriveCan.hpp"
#include "CanInterface.h"

constexpr int numAxes = 12;
extern const float jointOffsets[numAxes]; // Index with DogLegJoint value
extern const AxisClass jointClass[numAxes]; // Index with DogLegJoint value
extern odrive::ODriveAxis axes[numAxes]; // Index with DogLegJoint value (defined in CanInterface.cpp)
extern const char* axisName[numAxes]; // Index with DogLegJoint value

const char* getLegName(DogLeg leg);
extern odrive::CanInterface canInterface;