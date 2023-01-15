/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <stdint.h>
#include <cmath>
#include "Vector.hpp"

using Vector3f = Vector<float, 3>;

// The ODrive controller uses 'revolution' as positioning system
// Where position 1 means one full revolution of the motor.
// We also have 10:1 gearbox on each motor.
//
// These constants are to convert joint angles in radians to
// motor position coordinates and vice versa.
constexpr float posToRad = 0.62831853071f; // 2 * pi / 10
constexpr float radToPos = 1.59154943092f; // 10 / (2 * pi)

// We use these enums as indexes in arrays, so it is important that
// they start from 0.
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

class RobotLeg {
public:
  RobotLeg(DogLeg legId);

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
  int16_t getPosY() const { return y_; }
  int16_t getPosZ() const { return z_; }

  float getPosError() const;

  void calcPosFromAxis() {
    float x, y, z;
    calcPosFromAxis(x, y, z);
    x_ = std::roundf(x);
    y_ = std::roundf(y);
    z_ = std::roundf(z);
  }

  bool incrementX(int16_t v, bool start = true) {
    return setPos(x_ + v, y_, z_, start);
  }
  bool incrementY(int16_t v, bool start = true) {
    return setPos(x_, y_ + v, z_, start);
  }
  bool incrementZ(int16_t v, bool start = true) {
    return setPos(x_, y_, z_ + v, start);
  }
private:
  void calcPosFromAxis(float &x, float &y, float &z) const;

  const uint8_t leg_id_;
  // Position accuracy is not that great that we allow sub 1mm position.
  // Leg position in the leg reference frame in mm. z_ should be negative.
  int16_t x_;
  int16_t y_;
  int16_t z_;

  const DogLegJoint hip_axis_;
  const DogLegJoint tie_axis_;
  const DogLegJoint shin_axis_;
  const bool positive_shin_angle_;
  const bool reverse_x_;
};

class RobotBody {
public:
  // leg reference center distances
  static constexpr float length = 531.72;
  static constexpr float width = 118.28;
  RobotBody();

  bool setPos(
    DogLeg legId, int16_t x, int16_t y, int16_t z, bool start = true) {
    return legs_[legId].setPos(x, y, z, start);
  }

  void resetAll() {
    for (auto& leg: legs_) {
      leg.resetPos();
    }
  }

  void startAll() {
    for (auto& leg: legs_) {
      leg.startMove();
    }
  }

  int16_t getPosX(DogLeg legId) const {
    return legs_[legId].getPosX();
  }
  int16_t getPosY(DogLeg legId) const {
    return legs_[legId].getPosY();
  }
  int16_t getPosZ(DogLeg legId) const {
    return legs_[legId].getPosZ();
  }
  float getPosError(DogLeg legId) const {
    return legs_[legId].getPosError();
  }

  void parkLegs();
  void recalculateLegPositions() {
    for(auto& leg : legs_) {
      leg.calcPosFromAxis();
    }
  }

  bool incrementX(DogLeg legId, int16_t v, bool start = true) {
    return legs_[legId].incrementX(v, start);
  }
  bool incrementY(DogLeg legId, int16_t v, bool start = true) {
    return legs_[legId].incrementY(v, start);
  }
  bool incrementZ(DogLeg legId, int16_t v, bool start = true) {
    return legs_[legId].incrementZ(v, start);
  }

  void incrementAllX(DogLeg legId, int16_t v, bool start = true) {
    for(auto& leg : legs_) {
      leg.incrementX(v, start);
    }
  }
  void incrementAllY(DogLeg legId, int16_t v, bool start = true) {
    for(auto& leg : legs_) {
      leg.incrementY(v, start);
    }
  }
  void incrementAllZ(DogLeg legId, int16_t v, bool start = true) {
    for(auto& leg : legs_) {
      leg.incrementZ(v, start);
    }
  }
  static void setAxisIdle();
  static void setAxisActive();
  static void modifyAxisGains();
private:
  static void scheduleRecalculateLogPosition(uint32_t delay_ms);

  RobotLeg legs_[4];
};
