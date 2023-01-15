/*
 * Copyright (c) 2022-2023 ghent360. See LICENSE file for details.
*/

#include "Kinematics.h"
#include <stdint.h>
#include <math.h>

// Size in millimeters. Angles in rad.
constexpr float tieLength = 199.36;
constexpr float shinLength = 205;
constexpr float hipLength = 107.36;
constexpr float tieLength2 = tieLength * tieLength;
constexpr float shinLength2 = shinLength * shinLength;
constexpr float hipLength2 = hipLength * hipLength;
constexpr float lenDiff = shinLength - tieLength;
constexpr float lenDiff2 = lenDiff * lenDiff;
//constexpr float epsilon = 0.0005f;

static void inverseKinematics2d(
  float x, float z, bool posShinAngle, float &tieAngle, float &shinAngle) {
    float r2 = x * x + z * z;
    // check the destination is reachable
    // if (r2 > (tieLength2 + shinLength2)) return;
    // if (r2 < lenDiff2) return;
    float phi = acosf((tieLength2 + shinLength2 - r2) / (2 * tieLength * shinLength));
    float xsi2 = atan2f(x, z);
    if (x > 0) xsi2 -= float(M_PI);
    else xsi2 += float(M_PI);
    float r = sqrt(r2);
    float xsi1 = acosf((r2 + tieLength2 - shinLength2) / (2 * r * tieLength));
    if (posShinAngle) {
        shinAngle = float(M_PI) - phi;
        tieAngle = xsi2 - xsi1;
    } else {
        shinAngle = -(float(M_PI) - phi);
        tieAngle = xsi2 + xsi1;
    }
}

void inverseKinematics(
    float x, float y, float z, bool posShinAngle, float &h, float &t, float &s) {
    float hyp2 = z * z + y * y;
    float hyp = sqrtf(hyp2);
    float alpha = acosf(fabs(y) / hyp);
    float beta = acosf(hipLength / hyp);
    if (y > 0.0f) {
        h = alpha - beta;
    } else {
        h = float(M_PI) - alpha - beta;
    }
    float yprime = -sqrtf(hyp2 - hipLength2);
    inverseKinematics2d(-x, yprime, posShinAngle, t, s);
}

void forwardKinematics(
    float h, float t, float s, float &x, float &y, float &z) {
    float ch = cosf(h);
    float sh = sinf(h);
    float ct = cosf(t);
    float st = sinf(t);
    float cs = cosf(s);
    float ss = sinf(s);
    y = -(tieLength * ct * sh - hipLength * ch +
          tieLength * ct * cs * sh - shinLength * sh * st * ss);
    z = shinLength * ch * st * ss - tieLength * ch * ct -
        shinLength * ch * ct * cs - hipLength * sh;
    x = tieLength * st + shinLength * ct * ss + shinLength * cs * st;
}

#if 0
// A more forgiving asin and acos functions when the argument is slightly larger 
// than 1 or smaller than -1
static float clamp(float v) {
    if (v > 1.0f && v < (1 + epsilon)) {
        v = 1.0f;
    } else if (v < -1.0f && v > (-1 - epsilon)) {
        v = -1.0f;
    }
    return v;
}

static float asinR(float v) {
    return asinf(clamp(v));
}

static float acosR(float v) {
    return acosf(clamp(v));
}

void inverseKinematics3(
    float x, float y, float z, bool sol, float &hipAngle, float &tieAngle, float &shinAngle) {
    float z2 = z * z;
    float y2 = y * y;
    float z2y2 = z2 + y2;
    float d = 2 * sqrtf(z2 - z2y2 * (hipLength2 - y2) / hipLength2);
    float a = (2 * z2y2) / hipLength2;
    float z1a;
    float z1b;
    z1a = (-2 * z + d) / a;
    z1b = (-2 * z - d) / a;
    float th1a = asinR(z1a / hipLength);
    float th1b = asinR(z1b / hipLength);
    if (fabsf(z1a) < fabsf(z1b)) {
        hipAngle = th1a;
    } else {
        hipAngle = th1b;
    }
    float yprime = -sqrtf(y*y + z*z - hipLength2);
    inverseKinematics(x, yprime, sol, tieAngle, shinAngle);
}
#endif
