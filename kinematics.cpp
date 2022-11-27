#include <math.h>
#include "kinematics.h"
#include <Ramp.h>

class Interpolation {
public:
    Interpolation()
        : interpolationFlag_(false),
          savedValue_(0xbadbeef) {}

    int go(int input, int duration) {
        if (input != savedValue_) { // check for new data
            interpolationFlag_ = false;
        }
        savedValue_ = input; // bookmark the old value

        if (!interpolationFlag_) { // only do it once until the flag is reset
            ramp_.go(input, duration, LINEAR, ONCEFORWARD); // start interpolation (value to go to, duration)
            interpolationFlag_ = true;
        }
        return ramp_.update();
    }

private:
    rampInt ramp_;
    bool interpolationFlag_;
    int savedValue_;
};

Interpolation interpFRX; // Front Right leg
Interpolation interpFRY;
Interpolation interpFRZ;
Interpolation interpFRT;

Interpolation interpFLX; // Front Left leg
Interpolation interpFLY;
Interpolation interpFLZ;
Interpolation interpFLT;

Interpolation interpBRX; // Back Rigth leg
Interpolation interpBRY;
Interpolation interpBRZ;
Interpolation interpBRT;

Interpolation interpBLX; // Back Left leg
Interpolation interpBLY;
Interpolation interpBLZ;
Interpolation interpBLT;

extern bool interpFlag;

void driveJoints(DogLegJoint joint, float pos) {
    // takes into account the original setup offsets for motor 
    // positions, and also turns around directions so they are
    // consistent, also constrains the motion limits for each joint

    pos = constrain(pos, -2.5, 2.5);
    switch (joint) {
    //case BACK_RIGHT_KNEE:
    case FRONT_LEFT_KNEE:
    //case FRONT_RIGHT_SHOULDER:
    case BACK_LEFT_SHOULDER:
    //case BACK_RIGHT_HIP:
    case BACK_LEFT_HIP:
        pos *= -1;
        break;
    }
    axes[joint].SetInputPos(pos + jointOffsets[joint]);
}

void kinematics(
    DogLeg leg,
    float xIn,
    float yIn,
    float zIn,
    float roll,
    float pitch,
    float yawIn,
    bool interOn,
    int dur) {
    // moving the foot sideways on the end plane
    float hipOffset = 108; // distance from the hip pivot to the centre of the leg
    float lengthY;
    float hipAngle1a;
    float hipAngle1b;
    float hipAngle1;
    float hipAngle1Degrees;
    float hipHyp;

    // moving the foot forwards or backwards in the side plane
    float shoulderAngle2;
    float shoulderAngle2Degrees;
    float z2;

    // side plane of individual leg only
    const float shinLength = 200;
    const float thighLength = 200;
    float z3;
    float shoulderAngle1;
    float shoulderAngle1Degrees;
    float shoulderAngle1a;
    float shoulderAngle1b;
    float shoulderAngle1c;
    float shoulderAngle1d;
    float kneeAngle;
    float kneeAngleDegrees;

    // *** ROTATION AXIS

    // roll axis
    const float bodyWidth = 59;      // half the distance between the hip  pivots (the front)
    float legDiffRoll;               // difference in height for each leg
    float bodyDiffRoll;              // how much shorter the 'virtual body' gets
    float footDisplacementRoll;      // where the foot actually is
    float footDisplacementAngleRoll; // smaller angle
    float footWholeAngleRoll;        // whole leg angle
    float hipRollAngle;              // angle for hip when roll axis is in use
    float rollAngle;                 // angle in RADIANS that the body rolls
    float zz1a;                      // hypotenuse of final triangle
    float zz1;                       // new height for leg to pass onto the next bit of code
    float yy1;                       // new position for leg to move sideways

    // pitch axis
    const float bodyLength = 272;     // half the distance between shoulder pivots  (the side)
    float legDiffPitch;               // difference in height for each leg
    float bodyDiffPitch;              // how much shorter the 'virtual body' gets
    float footDisplacementPitch;      // where the foot actually is
    float footDisplacementAnglePitch; // smaller angle
    float footWholeAnglePitch;        // whole leg angle
    float shoulderPitchAngle;         // angle for hip when roll axis is in use
    float pitchAngle;                 // angle in RADIANS that the body rolls
    float zz2a;                       // hypotenuse of final triangle
    float zz2;                        // new height for the leg to pass onto the next bit of code
    float xx1;                        // new position to move the leg fowwards/backwards

    // yaw axis
    float yawAngle;      // angle in RADIANs for rotation in yaw
    float existingAngle; // existing angle of leg from centre
    float radius;        // radius of leg from centre of robot based on x and y from sticks
    float demandYaw;     // demand yaw postion - existing yaw plus the stick yaw
    float xx3;           // new X coordinate based on demand angle
    float yy3;           // new Y coordinate based on demand angle

    float x;
    float y;
    float z;
    float yaw;

    // ** INTERPOLATION **
    // use Interpolated values if Interpolation is on
    if (interOn) {
        switch (leg) {
        case FRONT_RIGHT:
            z = interpFRZ.go(zIn, dur);
            x = interpFRX.go(xIn, dur);
            y = interpFRY.go(yIn, dur);
            yaw = interpFRT.go(yawIn, dur);
            break;
        case FRONT_LEFT:
            z = interpFLZ.go(zIn, dur);
            x = interpFLX.go(xIn, dur);
            y = interpFLY.go(yIn, dur);
            yaw = interpFLT.go(yawIn, dur);
            break;
        case BACK_RIGHT:
            z = interpBRZ.go(zIn, dur);
            x = interpBRX.go(xIn, dur);
            y = interpBRY.go(yIn, dur);
            yaw = interpBRT.go(yawIn, dur);
            break;
        case BACK_LEFT:
            z = interpBLZ.go(zIn, dur);
            x = interpBLX.go(xIn, dur);
            y = interpBLY.go(yIn, dur);
            yaw = interpBLT.go(yawIn, dur);
            break;
        }
        // wait for filters to settle before using Interpolated values
        // set a timer for filter to settle
        if (!interpFlag) {
            z = zIn; // in the meantime use raw values
            x = xIn;
            y = yIn;
            yaw = yawIn;
            if (currentMillis - previousInterpMillis >= 300) {
                interpFlag = true;
            }
        }
    } else {
        // Interpolation is off then use the original values
        z = zIn;
        x = xIn;
        y = yIn;
        yaw = yawIn;
    }

    // **** START INVERSE KINEMATICS CALCS ****
    // yy3 = y;
    // zz2 = z;
    // xx3 = x;

    // ** YAW AXIS **
    // convert degrees to radians for the calcs
    yawAngle = ((float)(M_PI / 180)) * yaw;

    // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
    switch (leg) {
    case FRONT_RIGHT:
        y = y - (bodyWidth + hipOffset);
        x = x - bodyLength;
        break;
    case FRONT_LEFT:
        y = y + (bodyWidth + hipOffset);
        x = x - bodyLength;
        break;
    case BACK_LEFT:
        y = y - (bodyWidth + hipOffset);
        x = x + bodyLength;
        break;
    case BACK_RIGHT:
        y = y + (bodyWidth + hipOffset);
        x = x + bodyLength;
        break;
    }

    // calc existing angle of leg from centre
    existingAngle = atanf(y / x);
    // calc radius from centre
    radius = y / sinf(existingAngle);
    // calc demand yaw angle
    demandYaw = existingAngle + yawAngle;
    // calc new X and Y based on demand yaw angle
    xx3 = radius * cosf(demandYaw); // calc new X and Y based on new yaw angle
    yy3 = radius * sinf(demandYaw);

    // remove the offsets so we pivot around 0/0 x/y
    switch (leg) {
    case FRONT_RIGHT:
        yy3 = yy3 + (bodyWidth + hipOffset);
        xx3 = xx3 + bodyLength;
        break;
    case FRONT_LEFT:
        yy3 = yy3 - (bodyWidth + hipOffset);
        xx3 = xx3 + bodyLength;
        break;
    case BACK_LEFT:
        yy3 = yy3 + (bodyWidth + hipOffset);
        xx3 = xx3 - bodyLength;
        break;
    case BACK_RIGHT:
        yy3 = yy3 - (bodyWidth + hipOffset);
        xx3 = xx3 - bodyLength;
        break;
    }

    // ** PITCH AXIS ***
    if (leg == FRONT_LEFT || leg == FRONT_RIGHT) {
        pitch = pitch * -1;
        xx3 = xx3 * -1;
    }

    // convert pitch to degrees
    pitchAngle = ((float)(M_PI / 180)) * pitch;

    // calc top triangle sides
    legDiffPitch = sinf(pitchAngle) * bodyLength;
    bodyDiffPitch = cosf(pitchAngle) * bodyLength;

    // calc actual height from the ground for each side
    legDiffPitch = z - legDiffPitch;

    // calc foot displacement
    footDisplacementPitch = ((bodyDiffPitch - bodyLength) * -1) + xx3;

    // calc smaller displacement angle
    footDisplacementAnglePitch = atanf(footDisplacementPitch / legDiffPitch);

    // calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz2a = legDiffPitch / cosf(footDisplacementAnglePitch);

    // calc the whole angle for the leg
    footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;

    // calc actual leg length - the new Z to pass on
    zz2 = cosf(footWholeAnglePitch) * zz2a;

    // calc new Z to pass on
    xx1 = sinf(footWholeAnglePitch) * zz2a;

    if (leg == FRONT_LEFT || leg == FRONT_RIGHT) {
        xx1 = xx1 * -1;
    }

    // *** ROLL AXIS ***

    // turn around roll angle for each side of the robot
    if (leg == FRONT_LEFT || leg == BACK_LEFT) {
        roll = -roll;
        yy3 = yy3 * -1;
    }
    /*else if (leg == 1 || leg == 4) {
        roll = 0 + roll;
    }*/

    // convert roll angle to radians
    rollAngle = ((float)(M_PI / 180)) * roll; // covert degrees from the stick to radians

    // calc the top triangle sides
    legDiffRoll = sinf(rollAngle) * bodyWidth;
    bodyDiffRoll = cosf(rollAngle) * bodyWidth;

    // calc actual height from the ground for each side
    legDiffRoll = zz2 - legDiffRoll;

    // calc foot displacement
    footDisplacementRoll = (((bodyDiffRoll - bodyWidth) * -1) + hipOffset) - yy3;

    // calc smaller displacement angle
    footDisplacementAngleRoll = atanf(footDisplacementRoll / legDiffRoll);

    // calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz1a = legDiffRoll / cosf(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;

    // calc actual leg length - the new Z to pass on
    zz1 = cosf(footWholeAngleRoll) * zz1a;

    // calc new Y to pass on
    yy1 = (sinf(footWholeAngleRoll) * zz1a) - hipOffset; // take away the offset so we can pivot around zero

    // *** TRANSLATION AXIS ***

    // calculate the hip joint and new leg length based on how far the robot moves sideways
    // Y axis - side to side
    // first triangle

    if (leg == FRONT_RIGHT || leg == BACK_RIGHT) {
        // reverse the calcs for each side of the robot
        hipOffset *= -1;
        yy1 = yy1 * -1;
    }

    yy1 = yy1 + hipOffset; // add on hip offset because there is default distance in Y
    hipAngle1a = atanf(yy1 / zz1);
    hipAngle1Degrees = (hipAngle1a * ((float)(180 / M_PI))); // convert to degrees
    hipHyp = zz1 / cosf(hipAngle1a);                         // this is the hypotenuse of the first triangle

    // second triangle
    hipAngle1b = asinf(hipOffset / hipHyp); // calc 'the other angle' in the triangle
    // hipAngle1 = (M_PI - (M_PI / 2) - hipAngle1b) + hipAngle1a; // calc total hip angle
    hipAngle1 = ((float)(M_PI / 2) - hipAngle1b) + hipAngle1a; // calc total hip angle
    hipAngle1 = hipAngle1 - 1.5708f;                           // take away offset for rest position
    hipAngle1Degrees = (hipAngle1 * ((float)(180 / M_PI)));    // convert to degrees

    // calc new leg length to give to the code  below
    z2 = hipOffset / tanf(hipAngle1b); // new leg length

    // ****************

    // X axis - front to back
    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atanf(xx1 / z2); // calc how much extra to add to the shoulder joint
    shoulderAngle2Degrees = shoulderAngle2 * ((float)(180 / M_PI));
    z3 = z2 / cosf(shoulderAngle2); // calc new leg length to feed to the next bit of code below

    // ****************

    // Z axis - up and down
    // calculate leg length based on shin/thigh length and knee and shoulder angle
    z3 = constrain(z3, 200, 390); // constrain leg length to stop it turning inside out and breaking the trig
    shoulderAngle1a = sqrtf(thighLength) + sqrtf(z3) - sqrtf(shinLength);
    shoulderAngle1b = 2 * thighLength * z3;
    shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
    shoulderAngle1 = acosf(shoulderAngle1c);          // radians
    kneeAngle = ((float)M_PI) - (shoulderAngle1 * 2); // radians

    // calc degrees from angles
    shoulderAngle1Degrees = shoulderAngle1 * ((float)(180 / M_PI)); // degrees
    kneeAngleDegrees = kneeAngle * ((float)(180 / M_PI));           // degrees

    // write to joints

    // factor for converting degrees to motor turns used by the ODrive
    const float conversion = 0.02777777777777777777777777777778f;

    switch (leg) {
/*
    case FRONT_RIGHT: {
        float shoulderAngle1Counts = (shoulderAngle1Degrees - 45f) * conversion;// convert to encoder counts
        float shoulderAngle2Counts = shoulderAngle2Degrees * conversion;        // convert to encoder counts
        float shoulderAngleCounts = shoulderAngle1Counts + shoulderAngle2Counts;
        float kneeAngleCounts = (kneeAngleDegrees - 90) * conversion;// convert to encoder counts
        float hipAngleCounts = hipAngle1Degrees * conversion;         // convert to encoder counts
        driveJoints(FRONT_RIGHT_SHOULDER, shoulderAngleCounts);       // front right shoulder
        driveJoints(FRONT_RIGHT_KNEE, kneeAngleCounts);               // front right knee
        driveJoints(FRONT_RIGHT_HIP, hipAngleCounts);                 // front right hip
    }
    break;
*/
    case FRONT_LEFT: {
        float shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * conversion; // convert to encoder counts
        float shoulderAngle2Counts = shoulderAngle2Degrees * conversion;        // convert to encoder counts
        float shoulderAngleCounts = shoulderAngle1Counts + shoulderAngle2Counts;
        float kneeAngleCounts = (kneeAngleDegrees - 90) * conversion; // convert to encoder counts
        float hipAngleCounts = hipAngle1Degrees * conversion;         // convert to encoder counts
        driveJoints(FRONT_LEFT_SHOULDER, shoulderAngleCounts);        // front left shoulder
        driveJoints(FRONT_LEFT_KNEE, kneeAngleCounts);                // front left knee
        driveJoints(FRONT_LEFT_HIP, hipAngleCounts);                  // front left hip
    }
    break;

    case BACK_LEFT: {
        float shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * conversion; // convert to encoder counts
        float shoulderAngle2Counts = shoulderAngle2Degrees * conversion;        // convert to encoder counts
        float shoulderAngleCounts = shoulderAngle1Counts - shoulderAngle2Counts;
        float kneeAngleCounts = (kneeAngleDegrees - 90) * conversion; // convert to encoder counts
        float hipAngleCounts = hipAngle1Degrees * conversion;         // convert to encoder counts
        driveJoints(BACK_LEFT_SHOULDER, shoulderAngleCounts);         // back left shoulder
        driveJoints(BACK_LEFT_KNEE, kneeAngleCounts);                 // back left knee
        driveJoints(BACK_LEFT_HIP, hipAngleCounts);                   // back left hip
    }
    break;
/*
    case BACK_RIGHT: {
        float shoulderAngle1Counts = (shoulderAngle1Degrees - 45) * conversion; // convert to encoder counts
        float shoulderAngle2Counts = shoulderAngle2Degrees * conversion;        // convert to encoder counts
        float shoulderAngleCounts = shoulderAngle1Counts - shoulderAngle2Counts;
        float kneeAngleCounts = (kneeAngleDegrees - 90) * conversion; // convert to encoder counts
        float hipAngleCounts = hipAngle1Degrees * conversion;         // convert to encoder counts
        driveJoints(BACK_RIGHT_SHOULDER, shoulderAngleCounts);        // back right shoulder
        driveJoints(BACK_RIGHT_KNEE, kneeAngleCounts);                // back right knee
        driveJoints(BACK_RIGHT_HIP, hipAngleCounts);                  // back right hip
    }
    break;
*/
    }
}

const float jointOffsets[numAxes] = {
//  [FRONT_RIGHT_KNEE] = 0,
  [FRONT_LEFT_KNEE] = 0.22,
//  [BACK_RIGHT_KNEE] = 0,
  [BACK_LEFT_KNEE] = -0.18,
//  [FRONT_RIGHT_SHOULDER] = 0,
  [FRONT_LEFT_SHOULDER] = -0.08,
//  [BACK_RIGHT_SHOULDER] = 0,
  [BACK_LEFT_SHOULDER] = -0.06,
//  [FRONT_RIGHT_HIP] = 0,
  [FRONT_LEFT_HIP] = 0.31,
//  [BACK_RIGHT_HIP] = 0,
  [BACK_LEFT_HIP] = 0.27
};

const AxisClass jointClass[numAxes] = {
//  [FRONT_RIGHT_KNEE] = AxisClass::CLASS_KNEE,
  [FRONT_LEFT_KNEE] = AxisClass::CLASS_KNEE,
//  [BACK_RIGHT_KNEE] = AxisClass::CLASS_KNEE,
  [BACK_LEFT_KNEE] = AxisClass::CLASS_KNEE,
//  [FRONT_RIGHT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [FRONT_LEFT_SHOULDER] = AxisClass::CLASS_SHOULDER,
//  [BACK_RIGHT_SHOULDER] = AxisClass::CLASS_SHOULDER,
  [BACK_LEFT_SHOULDER] = AxisClass::CLASS_SHOULDER,
//  [FRONT_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [FRONT_LEFT_HIP] = AxisClass::CLASS_HIP,
//  [BACK_RIGHT_HIP] = AxisClass::CLASS_HIP,
  [BACK_LEFT_HIP] = AxisClass::CLASS_HIP
};
