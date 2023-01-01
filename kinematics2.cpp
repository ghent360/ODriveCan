#include <stdint.h>
#include <math.h>

// Size in millimeters. Angles in rad.
constexpr float tieLength = 199.36;
constexpr float shinLength = 205;
constexpr float hipLength = 107.36;
constexpr float ballRadius = 32.5;
constexpr float tieLength2 = tieLength * tieLength;
constexpr float shinLength2 = shinLength * shinLength;
constexpr float hipLength2 = hipLength * hipLength;
constexpr float lenDiff = shinLength - tieLength;
constexpr float lenDiff2 = lenDiff * lenDiff;

const float sqrt2 = sqrtf(2);
const float sqrt2Half = sqrt2 / 2;
const float oneOverSqrt2 = 1 / sqrt2;

/* Simple kinematics for the leg in XY dimensions only. */
void forwardKinematics(float th1, float th2, float &x, float &y) {
    float b1 = tieLength * sinf(th1);
    float a1 = tieLength * cosf(th1);
    float a2 = shinLength * cos(th1 + th2);
    float b2 = shinLength * sin(th1 + th2);

    y = a1 + a2 + ballRadius;
    x = b1 + b2;
}

void inverseKinematics(float x, float y, float &th1, float &th2) {
    y -= ballRadius;
    float r2 = x * x + y * y;
    // check the destination is reachable
    // if (r2 > (tieLength2 + shinLength2)) return;
    // if (r2 < lenDiff2) return;
    float phy1 = acosf((tieLength2 + shinLength2 - r2) / (2 * tieLength * shinLength));
    th2 = phy1 - M_PI;
    float phy2 = atan2f(x, y);
    float r = sqrt(r2);
    float phy3 = acos((r2 + tieLength2 - shinLength2) / (2 * r * tieLength));
    th1 = phy2 + phy3;
}

/* Full kinematics for the leg in all 3 dimensions */
void forwardKinematics2(
    float h, float t, float k, float &x, float &y, float &z) {
    //h = 2*M_PI - h; // negate h
    x = shinLength*cos(t + k) + tieLength*cos(t);
    y = hipLength*sin(h) + cos(h)*(shinLength*sin(t + k) + tieLength*sin(t));
    z = hipLength*cos(h) + sin(h)*(shinLength*sin(t + k) + tieLength*sin(t));
}

void inverseKinematics2(
    float x, float y, float z, bool posth3, float &h, float &t, float &k) {
    float hypz = sqrtf(z * z + y * y);
    float alpha = acosf(fabs(z) / hypz);
    float beta = acosf(hipLength / hypz);
    if (z > 0) {
        h = alpha - beta;
    } else {
        h = M_PI - alpha - beta;
    }
    float xprime = x;
    float yprime = -sqrtf(y*y + z*z - hipLength2);
    float hypx2 = xprime * xprime + yprime * yprime;
    float hypx = sqrtf(hypx2);
    float phy = acosf(fabs(xprime) / hypx);
    float xsi = acosf((tieLength2 + hypx2 - shinLength2) / (2 * tieLength * hypx));
    k  = acosf((tieLength2 + shinLength2 - hypx2) / (2 * tieLength * shinLength));
    if (fabsf(k - (float)M_PI) < 0.0005f) {
        k = 0;
    }
    if (posth3) {
        if (xprime >= 0) {
            t = M_PI / 2 - xsi - phy;
        } else {
            t = -M_PI / 2 - xsi + phy;
        }
    } else {
        k = -k;
        if (xprime >= 0) {
            t = M_PI / 2 + xsi - phy;
        } else {
            t = -M_PI / 2 + xsi + phy;
        }
    }
}

static float angleClamp(float angle) {
    while (angle >= (M_PI - 0.0005f)) {
        angle -= M_PI;
    }
    while (angle <= -(M_PI - 0.0005f)) {
        angle += M_PI;
    }
    return angle;
}

static float asinR(float v) {
    if (v > 1.0f && v < 1.0005f) {
        v = 1.0f;
    } else if (v < -1.0f && v > -1.0005f) {
        v = -1.0f;
    }
    return asinf(v);
}

static float acosR(float v) {
    if (v > 1.0f && v < 1.0005f) {
        v = 1.0f;
    } else if (v < -1.0f && v > -1.0005f) {
        v = -1.0f;
    }
    return acosf(v);
}

void inverseKinematics3(
    float x, float y, float z, float &h, float &t, float &k) {
    float z2 = z * z;
    float y2 = y * y;
    float z2y2 = z2 + y2;
    float d = 2 * sqrtf(z2 - z2y2 * (hipLength2 - y2) / hipLength2);
    float a = (2 * z2y2) / hipLength2;
    float z1a = (-2 * z + d) / a;
    float z1b = (-2 * z - d) / a;
    float th1a = asinR(z1a / hipLength);
    float th1b = asinR(z1b / hipLength);
    float z1;
    if (fabsf(z1a) < fabsf(z1b)) {
        h = th1a;
        z1 = z1a;
    } else {
        h = th1b;
        z1 = z1b;
    }
    float l2a = x * x + (z1 + z) * (z1 + z);
    float la = sqrtf(l2a);
    float xsi1 = atan2f(x, z1 + z);
    float xsi2 = acosR((l2a + tieLength2 - shinLength2) / (2 * la * tieLength));
    float phi = acosR((shinLength2 + tieLength2 - l2a) / (2 * shinLength * tieLength));
    float th2a = xsi1 - xsi2;
    float th3a = phi - M_PI;
    float th2b = xsi1 + xsi2;
    float th3b = phi + M_PI;
    th2a = angleClamp(th2a);
    th3a = angleClamp(th3a);
    th2b = angleClamp(th2b);
    th3b = angleClamp(th3b);
    if (fabsf(th2a) < fabsf(th2b)) {
        t = th2a;
        k = th3a;
    } else {
        t = th2b;
        k = th3b;
    }
}
