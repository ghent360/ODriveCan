/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include "RobotDefinition.h"
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "VoltageMonitor.h"

// various global data structures used by several files

// My robot uses 12 axes.
constexpr int numAxes = 12;

// LiPO cell voltage levels
constexpr float cellWarnVoltage = 3.5;
constexpr float cellMinVoltage = 3.35;

// Static offset for each joint at its resting position.
extern const float jointOffsets[numAxes]; // Index with DogLegJoint value

// An AxisClass for each of the axes.
extern const AxisClass jointClass[numAxes]; // Index with DogLegJoint value
// The actual ODriveAxis object for each motor axis.
extern odrive::ODriveAxis axes[numAxes]; // Index with DogLegJoint value
// A name for each axis.
extern const char* axisName[numAxes]; // Index with DogLegJoint value

// A name for each leg.
const char* getLegName(DogLeg leg);

// Our CanInterface instance.
extern odrive::CanInterface canInterface;

// The Teensy battery monitor instance.
extern VoltageMonitor voltageMonitor;
