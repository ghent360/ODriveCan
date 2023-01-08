/*
 * Copyright (c) 2023 ghent360. See LICENSE file for details.
 */

#include "stepTrajectory.h"
#include "bezierCurve.hpp"
#include <math.h>

#define USE_BEZIER_CURVE

static BezierCurve<12, float> swingCurve;

static constexpr float stepHeight = 50;
static constexpr float stepLength = 150;
static constexpr float stanceDepth = 10;

static void stanceState(float t, float &x, float &y) {
    x = stepLength / 2 - stepLength * t;
    y = -stanceDepth * sinf(float(M_PI) * t);
}


#ifdef USE_BEZIER_CURVE
static void swingStateBezier(float t, float &x, float &y) {
    swingCurve.calculate(t, x, y);
}
#else
static void swingStateEllipse(float t, float &x, float &y) {
    float lhalf = stepLength / 2;
    float lhalf2 = lhalf * lhalf;
    x = -stepLength / 2 + stepLength * t;
    y = stepHeight * sqrtf(1 - x * x / lhalf2);
}
#endif

void gaitPos(float t, float& x, float& y) {
    constexpr float stepOffset = 0.75f;
    if ( t <= stepOffset) {
        stanceState(t / stepOffset, x, y);
    } else {
        t = (t - stepOffset) / (1 - stepOffset);
#ifdef USE_BEZIER_CURVE
        swingStateBezier(t, x, y);
#else
        swingStateEllipse(t, x, y);
#endif
    }
}

#if 0
static constexpr float velocity = 1;
static constexpr float Tsw = 0.25;
#endif

void initStepCurve(float xref, float yref) {
    float halfStepLength = stepLength / 2;

#if 0 // use velocity based P1 and P10
    swingCurve.setPoint(0, -halfStepLength + xref, yref);
    swingCurve.setPoint(1, -halfStepLength + xref - velocity / (12*Tsw), yref);
    swingCurve.setPoint(2, -(1.76*halfStepLength) + xref, yref + stepHeight * 0.9);
    swingCurve.setPoint(3, -(1.76*halfStepLength) + xref, yref + stepHeight * 0.9);
    swingCurve.setPoint(4, -(1.76*halfStepLength) + xref, yref + stepHeight * 0.9);
    swingCurve.setPoint(5, xref, yref + stepHeight * 0.9);
    swingCurve.setPoint(6, xref, yref + stepHeight * 0.9);
    swingCurve.setPoint(7, xref, yref + stepHeight * 1.36);
    swingCurve.setPoint(8, (1.76*halfStepLength) + xref, yref + stepHeight * 1.36);
    swingCurve.setPoint(9, (1.76*halfStepLength) + xref, yref + stepHeight * 1.36);
    swingCurve.setPoint(10, halfStepLength + xref + velocity / (12*Tsw), yref);
    swingCurve.setPoint(11, halfStepLength + xref, yref);
#else
    swingCurve.setPoint(0, -halfStepLength + xref, yref);
    swingCurve.setPoint(1, -(1.4f*halfStepLength) + xref, yref);
    swingCurve.setPoint(2, -(1.5f*halfStepLength) + xref, yref + stepHeight * 0.9f);
    swingCurve.setPoint(3, -(1.5f*halfStepLength) + xref, yref + stepHeight * 0.9f);
    swingCurve.setPoint(4, -(1.5f*halfStepLength) + xref, yref + stepHeight * 0.9f);
    swingCurve.setPoint(5, xref, yref + stepHeight * 0.9f);
    swingCurve.setPoint(6, xref, yref + stepHeight * 0.9f);
    swingCurve.setPoint(7, xref, yref + stepHeight * 1.36f);
    swingCurve.setPoint(8, (1.5f*halfStepLength) + xref, yref + stepHeight * 1.36f);
    swingCurve.setPoint(9, (1.5f*halfStepLength) + xref, yref + stepHeight * 1.36f);
    swingCurve.setPoint(10, (1.4f*halfStepLength) + xref, yref);
    swingCurve.setPoint(11, halfStepLength + xref, yref);
#endif
}
