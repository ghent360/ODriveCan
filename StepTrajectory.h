/*
 * Copyright (c) 2023 ghent360@iqury.us. See LICENSE file for details.
 */

#pragma once

//#define USE_BEZIER_CURVE

#ifdef USE_BEZIER_CURVE
#include "BezierCurve.hpp"
#endif

class StepTrajectory {
public:
  StepTrajectory();

  void reset();
  void gaitPos(float t, float& x, float& y);

  float getStepHeight() const { return step_height_; }
  float getStepLength() const { return step_length_; }
  float getStanceDepth() const { return stance_depth_; }
  float getStepOffset() const { return step_offset_; }

  void setStepParams(float height, float length, float depth) {
    if (height >= 0) {
      step_height_ = height;
    }
    if (length >= 0) {
      step_length_ = length;
    }
    if (depth >= 0) {
      stance_depth_ = depth;
    }
    initStepCurve();
  }

  void setStepOffset(float offset) {
    if (offset > 0 && offset < 1) {
      step_offset_ = offset;
    }
  }
private:
  void initStepCurve();
  void stanceState2D(float t, float &x, float &y);
  void swingState2D(float t, float &x, float &y);

  float step_height_;
  float step_length_;
  float stance_depth_;
  float step_offset_;
#if 0
  float velocity_ = 1;
  float Tsw_ = 0.25;
#endif

#ifdef USE_BEZIER_CURVE
  BezierCurve2D<12, float> swing_curve_;
#endif
};
