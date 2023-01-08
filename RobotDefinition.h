/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 */
#pragma once

// The ODrive controller uses 'revolution' as positioning system
// Where position 1 means one full revolution of the motor.
// We also have 10:1 gearbox on each motor.
//
// These constants are to convert joint angles in radians to
// motor position coordinates and vice versa.
constexpr float posToRad = 0.62831853071f; // 2 * pi / 10
constexpr float radToPos = 1.59154943092f; // 10 / (2 * pi)

enum DogLeg {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

// AXIS class definitions:
enum AxisClass {
  CLASS_SHIN,
  CLASS_HIP,
  CLASS_TIE
};

enum DogLegJoint {
    FRONT_RIGHT_SHIN,
    FRONT_LEFT_SHIN,
    BACK_RIGHT_SHIN,
    BACK_LEFT_SHIN,
    FRONT_RIGHT_TIE,
    FRONT_LEFT_TIE,
    BACK_RIGHT_TIE,
    BACK_LEFT_TIE,
    FRONT_RIGHT_HIP,
    FRONT_LEFT_HIP,
    BACK_RIGHT_HIP,
    BACK_LEFT_HIP
};

class Leg {
public:
  Leg(DogLeg legId);

  bool setPos(int16_t x, int16_t y, int16_t z, bool start = true) {
    x_ = x;
    y_ = y;
    z_ = z;
    if (start) {
      return startMove();
    }
    return false;
  }

  bool resetPos(bool start = true) {
    return setPos(20, 107, -330, start);
  }

  bool startMove();

  int16_t getPosX() const { return x_; }
  int16_t getPosY() const { return x_; }
  int16_t getPosZ() const { return x_; }
  float getPosError() const;
private:
  const uint8_t leg_id_;
  // Position accuracy is not that great that we allow sub 1mm position.
  // Leg position in the leg reference frame in mm. z_ should be negative.
  int16_t x_;
  int16_t y_;
  int16_t z_;

  const DogLegJoint hip_axis_;
  const DogLegJoint tie_axis_;
  const DogLegJoint shin_axis_;
  const bool posShinAngle_;
  const bool reverseX_;
};

class RobotBody {
public:
  RobotBody();

  bool setPos(DogLeg legId, int16_t x, int16_t y, int16_t z) {
    return legs_[legId].setPos(x, y, z, true);
  }

  void resetAll() {
    for (auto& leg: legs_) {
      leg.resetPos(true);
    }
  }

  void parkLegs();
private:
  Leg legs_[numLegs];
};
