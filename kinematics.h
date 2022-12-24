/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>
#include "ODriveCan.hpp"

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
extern odrive::ODriveAxis axes[numAxes]; // Index with DogLegJoint value (defined in CanInterface.cpp)
