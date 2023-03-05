/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
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
    float r = sqrtf(r2);
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

void inverseKinematics_new(
    float x, float y, float z, bool posShinAngle, float &h, float &t, float &s) {
    float hyp2 = y * y + z * z - hipLength2;
    float hyp = sqrtf(hyp2);
    h = -(float(M_PI) - atan2f(-z, y) - atan2f(hyp, -hipLength));
    float d = (hyp2 + x * x - tieLength2 - shinLength2) /
        (2 * tieLength * shinLength);
    float q = sqrtf(1 - d * d);
    if (posShinAngle) {
        s = atan2f(-q, d);
    } else {
        s = atan2f(q, d);
    }
    t = atan2f(x, hyp) -
        atan2f(shinLength * sinf(s), tieLength + shinLength * cosf(s));
}

void forwardKinematics(
    float h, float t, float s, float &x, float &y, float &z) {
    float ch = cosf(h);
    float sh = sinf(h);
    float ct = cosf(t);
    float st = sinf(t);
    float cs = cosf(s);
    float ss = sinf(s);
    //y = -(tieLength * ct * sh - hipLength * ch +
    //      tieLength * ct * cs * sh - shinLength * sh * st * ss);
    y = hipLength * ch + shinLength * sh * st * ss - tieLength * ct * sh * (1 + cs);
    //z = shinLength * ch * st * ss - tieLength * ch * ct -
    //    shinLength * ch * ct * cs - hipLength * sh;
    z = shinLength * ch * (st * ss - ct * cs) - tieLength * ch * ct - hipLength * sh;
    //z = -shinLength * ch * cosf(t + s) - tieLength * ch * ct - hipLength * sh;
    //x = tieLength * st + shinLength * ct * ss + shinLength * cs * st;
    x = tieLength * st + shinLength * (ct * ss + cs * st);
    //x = tieLength * st + shinLength * sinf(t + s);
}

/*
z = - shinLength * ch * cos(t + s) - tieLength * ch * ct - hipLength * sh

calculator input:
- S * cos(func_h(t)) * cos(func_t(t) + func_s(t)) - T * cos(func_h(t)) * cos(func_t(t)) - H * sin(func_h(t))

First derivative:
S⋅(t′(t)+s′(t))cos(h(t))sin(t(t)+s(t))+Sh′(t)sin(h(t))cos(t(t)+s(t))+Tt′(t)cos(h(t))sin(t(t))+Th′(t)sin(h(t))cos(t(t))−Hh′(t)cos(h(t))

S*(wt+ws)*cos(h)*sin(t + s) +
S*wh*sin(h)*cos(t + s) +
T*wt*cos(h)*sin(t) + 
T*wh*sin(h)*cos(t) - 
H*wh*cos(h)

Second derivative
((−2Sh′(t)t′(t)−2Sh′(t)s′(t))sin(h(t))+(St′′(t)+Ss′′(t))cos(h(t)))sin(t(t)+s)+(Sh′′(t)sin(h(t))+(S⋅(t′(t))2+2Ss′(t)t′(t)+S⋅(s′(t))2+S⋅(h′(t))2)cos(h(t)))cos(t(t)+s(t))+(Tt′′(t)cos(h(t))−2Th′(t)t′(t)sin(h(t)))sin(t(t))+(Th′′(t)sin(h(t))+(T⋅(t′(t))2+T⋅(h′(t))2)cos(h(t)))cos(t(t))+H⋅(h′(t))2sin(h(t))−Hh′′(t)cos(h(t))

S * ((at + as) * cos(h) - 2 * wh * (wt + ws) * sin(h)) * sin(t + s) +
S * (ah * sin(h) + 2 * (wt + ws * wt + ws + wh) * cos(h)) * cos(t + s) + 
T * (at * cos(h) - 2 * wh * wt * sin(h)) * sin(t) +
T * (ah * sin(h) + 2 * (wt + wh) * cos(h)) * cos(t) +
2 * H * wh * sin(h) -
H * ah * cos(h)

Simplify for stationary position where wh, wt and ws are 0

kT * (
    S * (it + is) * cos(h) * sin(t + s) +
    S * ih * sin(h) * cos(t + s) + 
    T * it * cos(h) * sin(t) +
    T * ih * sin(h) * cos(t) +
    H * ih * cos(h)
)
-------------------------------------
y = hipLength * ch + shinLength * sh * st * ss - tieLength * ct * sh * (1 + cs)

calculator input
H * cos(func_h(t)) + S * sin(func_h(t)) * sin(func_t(t)) * sin(func_s(t)) - T * cos(func_t(t)) * sin(func_h(t)) * (1 + cos(func_s(t)))

First derivative
(Sh′(t)cos(h(t))sin(s(t))+(Tt′(t)+Ss′(t))sin(h(t))cos(s(t))+Tt′(t)sin(h(t)))sin(t(t))+((St′(t)+Ts′(t))sin(h(t))sin(s(t))−Th′(t)cos(h(t))cos(s(t))−Th′(t)cos(h(t)))cos(t(t))−Hh′(t)sin(h(t))

(S * wh * cos(h) * sin(s) + (T * wt + S * ws) * sin(h) * cos(s) + T * wt * sin(h)) * sin(t) + 
((S * wt + T * ws) * sin(h) * sin(s) - T * wh * cos(h) * (cos(s) + 1)) * cos(t) -
H * wh * sin(h)

Second derivative
(((−S⋅(t′(t))2−2Ts′(t)t′(t)−S⋅(s′(t))2−S⋅(h′(t))2)sin(h(t))+Sh′′(t)cos(h(t)))sin(s(t))+((Tt′′(t)+Ss′′(t))sin(h(t))+(2Th′(t)t′(t)+2Sh′(t)s′(t))cos(h(t)))cos(s(t))+Tt′′(t)sin(h(t))+2Th′(t)t′(t)cos(h(t)))sin(t(t))+(((St′′(t)+Ts′′(t))sin(h(t))+(2Sh′(t)t′(t)+2Th′(t)s′(t))cos(h(t)))sin(s(t))+((T⋅(t′(t))2+2Ss′(t)t′(t)+T⋅(s′(t))2+T⋅(h′(t))2)sin(h(t))−Th′′(t)cos(h(t)))cos(s(t))+(T⋅(t′(t))2+T⋅(h′(t))2)sin(h(t))−Th′′(t)cos(h(t)))cos(t(t))−Hh′′(t)sin(h(t))−H⋅(h′(t))2cos(h(t))

 ((S * ah * cos(h) - 2 * (S * (wt + ws + wh) + T * ws * wt) * sin(h)) * sin(s) +
  ((T * at + S * as) * sin(h) + 2 * (T * wh * wt + S * wh * ws) * cos(h)) * cos(s) + 
  T * at * sin(h) + 2 * T * wh * wt * cos(h)) * sin(t) + 
 (
  ((S * at + T * as) * sin(h) + 2 * (S * wh * wt + T * wh * ws) * cos(h)) * sin(s) +
  (2 * (T * (wt + ws + wh) + S * ws * wt) * sin(h) - T * ah * cos(h)) * cos(s) +
  (2 * T * (wt + wh) * sin(h) - T * ah * cos(h))) * cos(t) -
 H * ah * sin(h) - 
 2 * H * wh * cos(h)

Simplify for stationary position where wh, wt and ws are 0

(S * ah * cos(h) * sin(h) * sin(s) + (T * at + S * as) * sin(h) * cos(s) + T * at * sin(h)) * sin(t) + 
((S * at + T * as) * sin(h) * sin(s) - T * ah * cos(h) * cos(s) - T * ah * cos(h)) * cos(t) -
H * ah * sin(h)
-------------------------------------
x = tieLength * st + shinLength * sinf(t + s)

calculator input
T * sin(func_t(t)) + S * sin(func_t(t) + func_s(t))

First derivative
S⋅(t′(t)+s′(t))cos(t(t)+s(t))+Tt′(t)cos(t(t))

S * (wt + ws) * cos(t + s) + T * wt * cos(t)

Second derivative
−S⋅(t′(t)+s′(t))2sin(t(t)+s(t))+S⋅(t′′(t)+s′′(t))cos(t(t)+s(t))−T⋅(t′(t))2sin(t(t))+Tt′′(t)cos(t(t))

 S * (at + as) * cos(t + s) -
 2 * S * (wt + ws) * sin(t + s) -
 2 * T * wt * sin(t) +
 T * at * cos(t)

Simplify for stationary position where wh, wt and ws are 0

 S * (at + as) * cos(t + s) +
 T * at * cos(t)
*/

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
