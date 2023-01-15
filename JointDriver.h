/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
*/
#pragma once
#include "RobotDefinition.h"

void driveJoints(DogLegJoint joint, float pos);

#ifndef ARDUINO
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
