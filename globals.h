/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include "RobotDefinition.h"
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "VoltageMonitor.h"
#include "Display.h"
#include "Radio.h"

// various global data structures used by several files

// My robot uses 12 axes and 4 legs
constexpr int numAxes = 12;
constexpr int numLegs = 4;

// LiPO cell voltage levels
constexpr float cellMaxVoltage = 4.2;
constexpr float cellWarnVoltage = 3.5;
constexpr float cellMinVoltage = 3.35;

// Static offset for each joint at its resting position.
extern const float jointOffsets[numAxes]; // Index with DogLegJoint value
// Parking position values for each axis - include the offsets.
extern const float parkPosition[numAxes]; // Index with DogLegJoint value

// An AxisClass for each of the axes.
extern const AxisClass jointClass[numAxes]; // Index with DogLegJoint value
// The actual ODriveAxis object for each motor axis.
extern odrive::ODriveAxis axes[numAxes]; // Index with DogLegJoint value
// A name for each axis.
extern const char* axisName[numAxes]; // Index with DogLegJoint value
// The legs for the robot
extern Leg legs[numLegs]; // Index with DogLeg value

// A name for each leg.
const char* getLegName(DogLeg leg);

DogLegJoint getJointByAxisId(uint16_t axisId);

// Our CanInterface instance.
extern odrive::CanInterface canInterface;

// The Teensy battery monitor instance.
extern VoltageMonitor voltageMonitor;

// The Display instance.
extern Display display;

// The Radio instance.
extern Radio radio;