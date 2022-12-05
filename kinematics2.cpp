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
void forwardKinematics(float th1, float th2, float& x, float & y) {
    float b1 = tieLength * sinf(th1);
    float a1 = tieLength * cosf(th1);
    float a2 = shinLength * cos(th1 + th2);
    float b2 = shinLength * sin(th1 + th2);

    y = a1 + a2 + ballRadius;
    x = b1 + b2;
}

void inverseKinematics(float x, float y, float& th1, float& th2) {
    y -= ballRadius;
    float r2 = x*x + y*y;
    // check the destination is reachable
    //if (r2 > (tieLength2 + shinLength2)) return;
    //if (r2 < lenDiff2) return; 
    float phy1 = acosf((tieLength2 + shinLength2 - r2) / (2 * tieLength * shinLength));
    th2 = phy1 - M_PI;
    float phy2 = atan2f(x, y);
    float r = sqrt(r2);
    float phy3 = acos((r2 + tieLength2 - shinLength2) / (2 * r * tieLength));
    th1 = phy2 + phy3;
}

/* Full kinematics for the leg in all 3 dimensions */
void forwardKinematics2(
    float th1, float th2, float th3, float& x, float& y, float& z) {
    float x1 = tieLength * cosf(th1);
    float y1 = tieLength * sinf(th1);
    float x2 = shinLength * cosf(th1 + th2);
    float y2 = shinLength * sinf(th1 + th2);
    x = y1 + y2;
    float ll2 = (x1 + x2) * (x1 + x2) + (y1 + y2) * (y1 + y2);
    float ll = sqrtf(ll2);
    float z1 = hipLength * cosf(th3);
    float z2 = ll * cosf(M_PI / 2 - th3);
    z = z1 + z2;
    float yy1 = hipLength * sinf(th3);
    float yy2 = ll * sinf(M_PI / 2 - th3);
    y = yy1 + yy2 + ballRadius;
}

void inverseKinematics2(
    float x, float y, float z, float& th1, float& th2, float& th3) {
    float r1sqr = z*z + y*y;
    float ll2 = r1sqr - hipLength2;
    float ll = sqrtf(ll2);
    float phy1 = atan2f(y, z);
    float phy2 = atan2f(ll, hipLength);
    th3 = phy1 + phy2;
    float h = hipLength * sinf(th3);
    float r2sqr = (h + y) * (h + y) + x*x; // ll2 == r2sqr
    float r2 = sqrt(r2sqr);
    float phy3 = acosf((tieLength2 + shinLength2 - r2sqr) / (2 * tieLength * shinLength));
    th2 = phy3 - M_PI;
    phy1 = atan2f(x, y + h);
    phy2 = acosf((tieLength2 + r2sqr - shinLength2) / (2 * tieLength * r2));
    th1 = phy1 + phy2;
}