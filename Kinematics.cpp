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
    //Oroginal equation for reference
    //y = -(tieLength * ct * sh - hipLength * ch +
    //      tieLength * ct * cs * sh - shinLength * sh * st * ss);
    y = hipLength * ch + shinLength * sh * st * ss - tieLength * ct * sh * (1 + cs);
    //Oroginal equation for reference
    //z = shinLength * ch * st * ss - tieLength * ch * ct -
    //    shinLength * ch * ct * cs - hipLength * sh;
    z = shinLength * ch * (st * ss - ct * cs) - tieLength * ch * ct - hipLength * sh;
    //Even simpler equation, but not sure if it is more performant.
    //z = -shinLength * ch * cosf(t + s) - tieLength * ch * ct - hipLength * sh;
    //Oroginal equation for reference
    //x = tieLength * st + shinLength * ct * ss + shinLength * cs * st;
    x = tieLength * st + shinLength * (ct * ss + cs * st);
    //Even simpler equation, but not sure if it is more performant.
    //x = tieLength * st + shinLength * sinf(t + s);
}

/*
z = - shinLength * ch * cos(t + s) - tieLength * ch * ct - hipLength * sh

calculator input:
- S * cos(func_h(t)) * cos(func_t(t) + func_s(t)) - T * cos(func_h(t)) * cos(func_t(t)) - H * sin(func_h(t))

First derivative:
cos(h(t))(S⋅(t′(t)+s′(t))sin(t(t)+s(t))+Tt′(t)sin(t(t)))+h′(t)sin(h(t))(Scos(t(t)+s(t))+Tcos(t(t)))−Hh′(t)cos(h(t))


Second derivative
((−2Sh′(t)t′(t)−2Sh′(t)s′(t))sin(h(t))+(St′′(t)+Ss′′(t))cos(h(t)))sin(t(t)+s(t))+(Sh′′(t)sin(h(t))+(S⋅(t′(t))2+2Ss′(t)t′(t)+S⋅(s′(t))2+S⋅(h′(t))2)cos(h(t)))cos(t(t)+s(t))+(Tt′′(t)cos(h(t))−2Th′(t)t′(t)sin(h(t)))sin(t(t))+(Th′′(t)sin(h(t))+(T⋅(t′(t))2+T⋅(h′(t))2)cos(h(t)))cos(t(t))+H⋅(h′(t))2sin(h(t))−Hh′′(t)cos(h(t))

((-2*S*wh*wt-2*S*wh*ws)sin(h)+(S*at+S*as)*cos(h))*sin(t+s) +
(S*ah*sin(h)+(S*(wt)*2+2*S*ws*wt+S*(ws)*2+S*(wh)*2)*cos(h))*cos(t+s) +
(T*at*cos(h)-2*T*wh*wt*sin(h))*sin(t) +
(T*ah*sin(h)+(T*(wt)*2 + T*(wh)*2)*cos(h))*cos(t) +
H*(wh)*2*sin(h)-
H*ah*cos(h)

S * (((at + as) * cos(h) - 2 * wh * (wt + ws) * sin(h)) * sin(t + s) +
     (ah * sin(h) + 2 * (wt + ws * wt + ws + wh) * cos(h)) * cos(t + s)) +
T * ((at * cos(h) - 2 * wh * wt * sin(h)) * sin(t) +
     (ah * sin(h) + 2 * (wt + wh) * cos(h)) * cos(t)) +
H * (2 * wh * sin(h)- ah * cos(h))

shinLength * (((at + as) * ch - 2 * wh * (wt + ws) * sh) * sinf(t + s) +
     (ah * sh + 2 * (wt + ws * wt + ws + wh) * ch) * cosf(t + s)) +
tieLength * ((at * ch - 2 * wh * wt * sh) * st +
     (ah * sh + 2 * (wt + wh) * ch) * ct) +
hipLength * (2 * wh * sh- ah * ch)

Simplify for stationary position where wh, wt and ws are 0

shinLength * ((at + as) * ch * sinf(t + s) + ah * sh * cosf(t + s)) +
tieLength * (at * ch * st + ah * sh * ct) -
hipLength * ah * ch
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

void forwardVelocities(
    float h, float t, float s,
    float wh, float wt, float ws,
    float &vx, float &vy, float &vz) {
  float ch = cosf(h);
  float sh = sinf(h);
  float ct = cosf(t);
  float st = sinf(t);
  float cs = cosf(s);
  float ss = sinf(s);
  float cts = cosf(t + s);
  vz = ch * (shinLength * (wt + ws) * sinf(t + s) + tieLength * wt * st) +
    wh * sh * (shinLength * cts + tieLength * ct) -
    hipLength * wh * ch;

  vy =
    (shinLength * wh * ch * ss +
     (tieLength * wt + shinLength * ws) * sh * cs +
     tieLength * wt * sh) * st +
    ((shinLength * wt + tieLength * ws) * sh * ss -
     tieLength * wh * ch * (cs + 1)) * ct -
    hipLength * wh * sh;

  vx = shinLength * (wt + ws) * cts + tieLength * wt * ct;
}

void forwardAcceleration(
    float h, float t, float s,
    float wh, float wt, float ws,
    float ah, float at, float as,
    float &ax, float &ay, float &az) {
  float ch = cosf(h);
  float sh = sinf(h);
  float ct = cosf(t);
  float st = sinf(t);
  float cs = cosf(s);
  float ss = sinf(s);
  float cts = cosf(t + s);
  float sts = sinf(t + s);

  az = shinLength * (
      ((at + as) * ch - 2 * wh * (wt + ws) * sh) * sts +
      (ah * sh + 2 * (wt + ws * wt + ws + wh) * ch) * cts) +
    tieLength * (
        (at * ch - 2 * wh * wt * sh) * st +
        (ah * sh + 2 * (wt + wh) * ch) * ct) +
    hipLength * (2 * wh * sh - ah * ch);

  ay =
   ((shinLength * ah * ch - 2 * (shinLength * (wt + ws + wh) + tieLength * ws * wt) * sh) * ss +
    ((tieLength * at + shinLength * as) * sh + 2 * wh * (tieLength * wt + shinLength * ws) * ch) * cs +
    tieLength * (at * sh + 2 * wh * wt * ch)) * st +
   (
    ((shinLength * at + tieLength * as) * sh + 2 * wh * (shinLength * wt + tieLength * ws) * ch) * ss +
    (2 * (tieLength * (wt + ws + wh) + shinLength * ws * wt) * sh - tieLength * ah * ch) * cs +
    tieLength * (2 * (wt + wh) * sh - ah * ch)) * ct -
   hipLength * (ah * sh - 2 * wh * ch);

  ax =  shinLength * ((at + as) * cts - 2 * (wt + ws) * sts) -
    tieLength * (2 * wt * st + at * ct);
}

void forwardStandingAcceleration(
    float h, float t, float s,
    float ah, float at, float as,
    float &ax, float &ay, float &az) {
  float ch = cosf(h);
  float sh = sinf(h);
  float ct = cosf(t);
  float st = sinf(t);
  float cs = cosf(s);
  float ss = sinf(s);
  float cts = cosf(t + s);

  az = shinLength * ((at + as) * ch * sinf(t + s) + ah * sh * cts) +
    tieLength * (at * ch * st + ah * sh * ct) -
    hipLength * ah * ch;

  ay =  (
   shinLength * ah * ch * ss +
   (tieLength * at + shinLength * as) * sh * cs + tieLength * at * sh) * st +
   ((shinLength * at + tieLength * as) * sh * ss - tieLength * ah * ch * (1 + cs)) * ct -
   hipLength * (ah * sh);

  ay = 
   (shinLength * ah * ch * ss + (tieLength * at + shinLength * as) * sh * cs + tieLength * at * sh) * st +
   ((shinLength * at + tieLength * as) * sh * ss - tieLength * ah * ch * cs - tieLength * ah * ch) * ct -
   hipLength * ah * sh;

  ax =  shinLength * (at + as) * cts - tieLength * at * ct;
}
