/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <stdint.h>
#include <cmath>
#include "StepTrajectory.h"

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
    return setPos(0, 107, -350, start);
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

  bool allAxesActive() const;

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
  static constexpr uint8_t numberOfLegs = 4;
  // leg reference center distances
  static constexpr float length = 531.72;
  static constexpr float halfLength = length / 2;
  static constexpr float width = 118.28;
  static constexpr float halfWidth = width / 2;
  RobotBody();

  void init();

  bool setPos(
    DogLeg legId, int16_t x, int16_t y, int16_t z, bool start = true) {
    auto result = legs_[legId].setPos(x, y, z, start && isActiveAndIdle());
    leg_reference_pos_[legId][0] = x;
    leg_reference_pos_[legId][1] = y;
    leg_reference_pos_[legId][2] = z;
    return result;
  }

  void resetLeg(DogLeg legId, bool start = true) {
    legs_[legId].resetPos(start && isActiveAndIdle());
    updateReferencePos(legId);
  }

  void resetAll(bool start = true) {
    for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
      resetLeg(DogLeg(idx), start && isActiveAndIdle());
    }
  }

  bool startLeg(DogLeg legId) {
    if (!isActiveAndIdle()) return false;
    return legs_[legId].startMove();
  }

  void startAll() {
    if (!isActiveAndIdle()) return;
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

  int16_t getReferencePosX(DogLeg legId) const {
    return leg_reference_pos_[legId][0];
  }
  int16_t getReferencePosY(DogLeg legId) const {
    return leg_reference_pos_[legId][1];
  }
  int16_t getReferencePosZ(DogLeg legId) const {
    return leg_reference_pos_[legId][2];
  }

  void setReferencePos(DogLeg legId, int16_t x, int16_t y, int16_t z) {
    leg_reference_pos_[legId][0] = x;
    leg_reference_pos_[legId][1] = y;
    leg_reference_pos_[legId][2] = z;
  }

  float getPosError(DogLeg legId) const {
    return legs_[legId].getPosError();
  }

  bool incrementX(DogLeg legId, int16_t v, bool start = true) {
    auto result = legs_[legId].incrementX(v, start && isActiveAndIdle());
    leg_reference_pos_[legId][0] = legs_[legId].getPosX();
    return result;
  }
  bool incrementY(DogLeg legId, int16_t v, bool start = true) {
    auto result = legs_[legId].incrementY(v, start && isActiveAndIdle());
    leg_reference_pos_[legId][1] = legs_[legId].getPosY();
    return result;
  }
  bool incrementZ(DogLeg legId, int16_t v, bool start = true) {
    auto result = legs_[legId].incrementZ(v, start && isActiveAndIdle());
    leg_reference_pos_[legId][2] = legs_[legId].getPosZ();
    return result;
  }

  void incrementAllX(DogLeg legId, int16_t v, bool start = true) {
    for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
      legs_[idx].incrementX(v, start && isActiveAndIdle());
      leg_reference_pos_[idx][0] = legs_[idx].getPosX();
    }
  }
  void incrementAllY(DogLeg legId, int16_t v, bool start = true) {
    for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
      legs_[idx].incrementY(v, start && isActiveAndIdle());
      leg_reference_pos_[idx][1] = legs_[idx].getPosY();
    }
  }
  void incrementAllZ(DogLeg legId, int16_t v, bool start = true) {
    for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
      legs_[idx].incrementZ(v, start && isActiveAndIdle());
      leg_reference_pos_[idx][2] = legs_[idx].getPosZ();
    }
  }

  void startWalking();
  void stopWalking();
  void parkLegs();

  void printPositions() const;
  void setAllAxesIdle();
  void setAllAxesActive();
  static void modifyAxesGains();

  static float legToBodyX(DogLeg legId, float v) {
    float offset = 0;
    switch (legId) {
      case FRONT_LEFT:
      case FRONT_RIGHT:
        offset = halfLength;
        break;
      case BACK_LEFT:
      case BACK_RIGHT:
        offset = -halfLength;
        break;
    }
    return v + offset;
  }
  static float legToBodyY(DogLeg legId, float v) {
    float offset = 0;
    switch (legId) {
      case FRONT_LEFT:
      case BACK_LEFT:
        offset = -halfWidth;
        break;
      case FRONT_RIGHT:
      case BACK_RIGHT:
        offset = halfWidth;
        break;
    }
    return v + offset;
  }
  static float legToBodyZ(DogLeg legId, float v) {
    return v;
  }
  static float bodyToLegX(DogLeg legId, float v) {
    float offset = 0;
    switch (legId) {
      case FRONT_LEFT:
      case FRONT_RIGHT:
        offset = halfLength;
        break;
      case BACK_LEFT:
      case BACK_RIGHT:
        offset = -halfLength;
        break;
    }
    return v - offset;
  }
  static float bodyToLegY(DogLeg legId, float v) {
    float offset = 0;
    switch (legId) {
      case FRONT_LEFT:
      case BACK_LEFT:
        offset = -halfWidth;
        break;
      case FRONT_RIGHT:
      case BACK_RIGHT:
        offset = halfWidth;
        break;
    }
    return v - offset;
  }
  static float bodyToLegZ(DogLeg legId, float v) {
    return v;
  }
private:
  enum RobotState {
    STATE_IDLE,
    STATE_PREPARE_FRONT_LEFT,
    STATE_PREPARE_BACK_RIGHT,
    STATE_PREPARE_FRONT_RIGHT,
    STATE_PREPARE_BACK_LEFT,
    STATE_EXECUTE_GAIT,
    STATE_FINALIZE_FRONT_LEFT,
    STATE_FINALIZE_BACK_RIGHT,
    STATE_FINALIZE_FRONT_RIGHT,
    STATE_FINALIZE_BACK_LEFT
  };

  struct GaitPlan {
    float leg_offset_[numberOfLegs];
  };

  bool isActiveAndIdle() const {
    return axes_active_ && state_ == STATE_IDLE;
  }

  void updateReferencePos(DogLeg legId) {
    leg_reference_pos_[legId][0] = legs_[legId].getPosX();
    leg_reference_pos_[legId][1] = legs_[legId].getPosY();
    leg_reference_pos_[legId][2] = legs_[legId].getPosZ();
  }

  void recalculateLegPositions() {
    for (uint8_t idx = 0; idx < numberOfLegs; idx++) {
      legs_[idx].calcPosFromAxis();
      updateReferencePos(DogLeg(idx));
    }
  }

  void setCreepGaitPlan() {
    gait_plan_.leg_offset_[FRONT_LEFT] = 0;
    gait_plan_.leg_offset_[BACK_RIGHT] = 0.25f;
    gait_plan_.leg_offset_[FRONT_RIGHT] = 0.5f;
    gait_plan_.leg_offset_[BACK_LEFT] = 0.75f;
    walk_trajectory_.setStepOffset(0.75f);
  }

  void setTrotGaitPlan() {
    gait_plan_.leg_offset_[FRONT_LEFT] = 0;
    gait_plan_.leg_offset_[BACK_RIGHT] = 0;
    gait_plan_.leg_offset_[FRONT_RIGHT] = 0.5f;
    gait_plan_.leg_offset_[BACK_LEFT] = 0.5f;
    walk_trajectory_.setStepOffset(0.5f);
  }

  void planStepStart(DogLeg legId);
  void moveStepStart(DogLeg legId, float t);
  void finishStepStart(DogLeg legId);
  void moveLegWalk(DogLeg legId, float t);
  void moveWalk(float t);
  void planStepEnd(DogLeg legId);
  void moveStepEnd(DogLeg legId, float t);
  void finishStepEnd(DogLeg legId);
  void runState(uint32_t now);

  static void scheduleRecalculateLogPosition(uint32_t delay_ms);

  bool axes_active_;
  RobotState state_;
  uint32_t step_start_;
  float prep_step_half_size_;
  int32_t prep_step_height_;
  uint32_t prep_step_duration_;
  uint32_t walk_step_duration_;
  StepTrajectory prep_trajectory_;
  StepTrajectory walk_trajectory_;
  GaitPlan gait_plan_;
  int16_t leg_reference_pos_[numberOfLegs][3];
  RobotLeg legs_[numberOfLegs];
};
