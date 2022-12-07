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
    float q, float s, float t, float &x, float &y, float &z) {
    float Ct = cosf(t);
    float St = sinf(t);
    float tmp = (shinLength * sinf(q+s) + tieLength * sinf(s));

    x = -cosf(s) * (tieLength + shinLength * (cosf(q) - sinf(q)));
    y = Ct * tmp + hipLength * St;
    z = St * tmp + hipLength * Ct;
}

void inverseKinematics2(
    float x, float y, float z, bool posth3, float &th1, float &th2, float &th3) {
    float hypz = sqrtf(z * z + y * y);
    float alpha = acosf(fabs(z) / hypz);
    float beta = acosf(hipLength / hypz);
    if (z > 0) {
        th3 = alpha - beta;
    } else {
        th3 = M_PI - alpha - beta;
    }
    float xprime = x;
    float yprime = -sqrtf(y*y + z*z - hipLength2);
    float hypx2 = xprime * xprime + yprime * yprime;
    float hypx = sqrtf(hypx2);
    float phy = acosf(fabs(xprime) / hypx);
    float xsi = acosf((tieLength2 + hypx2 - shinLength2) / (2 * tieLength * hypx));
    th1  = acosf((tieLength2 + shinLength2 - hypx2) / (2 * tieLength * shinLength));
    if (fabsf(th1 - (float)M_PI) < 0.0005f) {
        th1 = 0;
    }
    if (posth3) {
        if (xprime >= 0) {
            th2 = M_PI / 2 - xsi - phy;
        } else {
            th2 = -M_PI / 2 - xsi + phy;
        }
    } else {
        th1 = -th1;
        if (xprime >= 0) {
            th2 = M_PI / 2 + xsi - phy;
        } else {
            th2 = -M_PI / 2 + xsi + phy;
        }
    }
}