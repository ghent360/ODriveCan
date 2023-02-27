/*
 * Copyright (c) 2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "StepTrajectory.h"
#include "BezierCurve.hpp"
#include <math.h>

//#define USE_BEZIER_CURVE

#ifdef USE_BEZIER_CURVE
static BezierCurve2D<12, float> swingCurve;
#endif

StepTrajectory::StepTrajectory() {
  reset();
}

void StepTrajectory::reset() {
  step_height_ = 20;
  step_length_ = 50;
  stance_depth_ = 5;
  step_offset_ = 0.75f;
#if 0
  velocity_ = 1;
  Tsw_ = 0.25;
#endif
  initStepCurve();
}

void StepTrajectory::stanceState2D(float t, float &x, float &y) {
  x = step_length_ / 2 - step_length_ * t;
  y = -stance_depth_ * sinf(float(M_PI) * t);
}

void StepTrajectory::swingState2D(float t, float &x, float &y) {
#ifdef USE_BEZIER_CURVE
  swing_curve_.calculate(t, x, y);
#else
  float lhalf = step_length_ / 2;
  float lhalf2 = lhalf * lhalf;
  x = step_length_ * t - lhalf;
  float d = 1.0f - x * x / lhalf2;
  if (d < 0) d = 0;
  if (d > 1) d = 1;
  y = step_height_ * sqrtf(d);
#endif
}


void StepTrajectory::gaitPos(float t, float& x, float& y) {
  if ( t <= step_offset_) {
    stanceState2D(t / step_offset_, x, y);
  } else {
    t = (t - step_offset_) / (1 - step_offset_);
    swingState2D(t, x, y);
  }
}

void StepTrajectory::initStepCurve() {
#ifdef USE_BEZIER_CURVE
  const float halfStepLength = step_length_ / 2;

#if 0 // use velocity based P1 and P10
  swingCurve.setPoint(0, -halfStepLength, 0);
  swingCurve.setPoint(1, -halfStepLength - velocity_ / (12*Tsw_), 0);
  swingCurve.setPoint(2, -(1.76*halfStepLength), step_height_ * 0.9);
  swingCurve.setPoint(3, -(1.76*halfStepLength), step_height_ * 0.9);
  swingCurve.setPoint(4, -(1.76*halfStepLength), step_height_ * 0.9);
  swingCurve.setPoint(5, 0, step_height_ * 0.9);
  swingCurve.setPoint(6, 0, step_height_ * 0.9);
  swingCurve.setPoint(7, 0, step_height_ * 1.36);
  swingCurve.setPoint(8, (1.76*halfStepLength), step_height_ * 1.36);
  swingCurve.setPoint(9, (1.76*halfStepLength), step_height_ * 1.36);
  swingCurve.setPoint(10, halfStepLength + velocity_ / (12*Tsw_), 0);
  swingCurve.setPoint(11, halfStepLength, 0);
#else
  swingCurve.setPoint(0, -halfStepLength, 0);
  swingCurve.setPoint(1, -(1.4f*halfStepLength), 0);
  swingCurve.setPoint(2, -(1.5f*halfStepLength), step_height_ * 0.9f);
  swingCurve.setPoint(3, -(1.5f*halfStepLength), step_height_ * 0.9f);
  swingCurve.setPoint(4, -(1.5f*halfStepLength), step_height_ * 0.9f);
  swingCurve.setPoint(5, 0, step_height_ * 0.9f);
  swingCurve.setPoint(6, 0, step_height_ * 0.9f);
  swingCurve.setPoint(7, 0, step_height_ * 1.36f);
  swingCurve.setPoint(8, (1.5f*halfStepLength), step_height_ * 1.36f);
  swingCurve.setPoint(9, (1.5f*halfStepLength), step_height_ * 1.36f);
  swingCurve.setPoint(10, (1.4f*halfStepLength), 0);
  swingCurve.setPoint(11, halfStepLength, 0);
#endif
#endif // USE_BEZIER_CURVE
}
