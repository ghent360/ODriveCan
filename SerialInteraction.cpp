/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "globals.h"
#include "kinematics.h"
#include "kinematics2.h"
#include "TaskManager.hpp"
#include "TaskIds.h"

using odrive::AxisState;

//#define DEBUG_AXIS_POS
//#define DEBUG_LEG_POS

#if defined(DEBUG_AXIS_POS)
static int8_t activeAxis = -1;
static float activeAxisPos = 0;
#endif
static int8_t activeLeg = -1;
static float activeLegX = 0;
static float activeLegY = 0;
static float activeLegZ = 0;

#if defined(DEBUG_AXIS_POS)
static void deactivateAxis() {
  activeAxis = -1;
  activeAxisPos = 0;
}
#endif

static void activateLeg(DogLeg leg) {
#if defined(DEBUG_AXIS_POS)
  deactivateAxis();
#endif
  Serial.print("Active leg ");
  Serial.println(getLegName(leg));
  activeLeg = leg;
  activeLegX = 20;
  activeLegY = 107.36;
  activeLegZ = -320;
}

#if defined(DEBUG_AXIS_POS)
static void deactivateLeg() {
  activeLeg = -1;
  activeLegX = activeLegY = activeLegZ = 0;
}

static void activateAxis(DogLegJoint axis) {
  deactivateLeg();
  Serial.print("Active axis ");
  Serial.println(axisName[axis]);
  activeAxis = axis;
  activeAxisPos = 0;
}

static bool isAxisActive() {
  return activeAxis >= 0;
}
#endif

static bool isLegActive() {
  return activeLeg >= 0;
}

static void printHelp() {
  Serial.println("Help:");
  Serial.println("  '?' - print this message.");
  Serial.println("  'l' - set limits and enter closed loop control mode.");
  Serial.println("  'i' - enter idle mode.");
  Serial.println("  'c' - clear all axis errors.");
  Serial.println("  'g' - modify gains.");
#if defined(DEBUG_AXIS_POS)
  Serial.println("  '1'..'6' - set axis active.");
  Serial.println("  '+', '-' - change the active axis position.");
#endif
  Serial.println("  '7'..'0' - set leg active.");
  Serial.println("  'x', 'X', 'y', 'Y', 'z', 'Z' - change the active leg position.");
  Serial.println("  'h' - move active/all axis to 'home' position.");
#if defined(DEBUG_AXIS_POS)
  if (isAxisActive()) {
    Serial.print("Active axis ");
    Serial.println(axisName[static_cast<DogLegJoint>(activeAxis)]);
  }
#endif
  if (isLegActive()) {
    Serial.print("Active leg ");
    Serial.println(getLegName(static_cast<DogLeg>(activeLeg)));
  }
}

static void setAxisLimitsAndStart() {
  for(auto& axis: axes) {
    axis.SetLimits(2.0f, 10.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
}

static void setAxisIdle() {
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
#if defined(DEBUG_AXIS_POS)
  deactivateAxis();
#endif
}

static void axesGoHome() {
  for (int idx=0; idx<numAxes; idx++) {
    if (axes[idx].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      driveJoints(static_cast<DogLegJoint>(idx), parkPosition[idx]);
    } 
  }
}

static void modifyGains() {
  constexpr float posGainKnee = 20.0f;
  constexpr float posGainHips = 60.0f;
  constexpr float posGainShoulder = 20.0f;
  constexpr float velGain = 0.1f;
  constexpr float integrator = 0.2f;
  float posGain = 20.0f;
  for (int idx=0; idx<numAxes; idx++) {
    switch(jointClass[idx]) {
      case CLASS_HIP:
        posGain = posGainHips;
        break;
      case CLASS_KNEE:
        posGain = posGainKnee;
        break;
      case CLASS_SHOULDER:
        posGain = posGainShoulder;
        break;
    }
    axes[idx].SetPosGain(posGain);
    axes[idx].SetVelGains(velGain, integrator);
  }
}

#if defined(DEBUG_AXIS_POS)
static void moveAxisPos() {
  if (isAxisActive()) {
    //activeAxisPos = constrain(activeAxisPos, -2.5, 2.5);
    Serial.print("pos = ");
    Serial.println(activeAxisPos);
    driveJoints(static_cast<DogLegJoint>(activeAxis), activeAxisPos);
  }
}
#endif

static void computeAnglesAndMove() {
  float ha, ta, sa;
  bool posShinAngle = (activeLeg == BACK_RIGHT) || (activeLeg == FRONT_RIGHT);
  float x = activeLegX;
  float y = activeLegY;
  float z = activeLegZ;
  if ((activeLeg == BACK_LEFT) || (activeLeg == FRONT_LEFT)) {
    x = -x;
  }
  inverseKinematics(x, z, y, posShinAngle, ha, ta, sa);
  float hp, tp, sp;
  hp = -ha * radToPos;
  tp = ta * radToPos;
  sp = sa * radToPos;
#if defined(DEBUG_LEG_POS)
  Serial.print("Leg pos x:");
  Serial.print(activeLegX, 3);
  Serial.print(" y:");
  Serial.print(activeLegY, 3);
  Serial.print(" z:");
  Serial.println(activeLegZ, 3);
  Serial.print("Angles shin:");
  Serial.print(sa, 3);
  Serial.print(" tie:");
  Serial.print(ta, 3);
  Serial.print(" hip:");
  Serial.println(ha, 3);
  Serial.print("Pos shin:");
  Serial.print(sp, 3);
  Serial.print(" tie:");
  Serial.print(tp, 3);
  Serial.print(" hip:");
  Serial.println(hp, 3);
#endif
  if (!isnan(sa) && !isnan(ta) && !isnan(ha)) {
    switch (activeLeg) {
      case BACK_RIGHT:
        driveJoints(BACK_RIGHT_HIP, hp);
        driveJoints(BACK_RIGHT_SHOULDER, tp);
        driveJoints(BACK_RIGHT_KNEE, sp);
        break;
      case FRONT_RIGHT:
        driveJoints(FRONT_RIGHT_HIP, hp);
        driveJoints(FRONT_RIGHT_SHOULDER, tp);
        driveJoints(FRONT_RIGHT_KNEE, sp);
        break;
      case BACK_LEFT:
        driveJoints(BACK_LEFT_HIP, hp);
        driveJoints(BACK_LEFT_SHOULDER, tp);
        driveJoints(BACK_LEFT_KNEE, sp);
        break;
      case FRONT_LEFT:
        driveJoints(FRONT_LEFT_HIP, hp);
        driveJoints(FRONT_LEFT_SHOULDER, tp);
        driveJoints(FRONT_LEFT_KNEE, sp);
        break;
    }
  }
}

void checkSerialInput(TaskNode*, uint32_t) {
  if (Serial.available()) {
    auto ch = Serial.read();
    if (ch < 32) return;
    Serial.println((char)ch); // Echo the incoming character.
    switch(ch) {
      default:
          Serial.print("Unknown command '");
          Serial.print((char)ch);
          Serial.println("'");
          // Fall trough.
      case '?':
          printHelp();
          break;
      case 'l':
          setAxisLimitsAndStart();
          break;
      case 'i':
          setAxisIdle();
          break;
      case 'h':
          axesGoHome();
          break;
      case 'c':
          for(auto& axis: axes) {
            axis.ClearErrors();
          }
          break;
      case 'g':
          modifyGains();
          break;
#if defined(DEBUG_AXIS_POS)
      case '1':
          activateAxis(FRONT_LEFT_KNEE);
          break;
      case '!':
          activateAxis(FRONT_RIGHT_KNEE);
          break;
      case '2':
          activateAxis(BACK_LEFT_KNEE);
          break;
      case '@':
          activateAxis(BACK_RIGHT_KNEE);
          break;
      case '3':
          activateAxis(FRONT_LEFT_SHOULDER);
          break;
      case '#':
          activateAxis(FRONT_RIGHT_SHOULDER);
          break;
      case '4':
          activateAxis(BACK_LEFT_SHOULDER);
          break;
      case '$':
          activateAxis(BACK_RIGHT_SHOULDER);
          break;
      case '5':
          activateAxis(FRONT_LEFT_HIP);
          break;
      case '%':
          activateAxis(FRONT_RIGHT_HIP);
          break;
      case '6':
          activateAxis(BACK_LEFT_HIP);
          break;
      case '^':
          activateAxis(BACK_RIGHT_HIP);
          break;
      case '+':
          if (isAxisActive()) {
            activeAxisPos += 0.05;
            moveAxisPos();
          }
          break;
      case '-':
          if (isAxisActive()) {
            activeAxisPos -= 0.05;
            moveAxisPos();
          }
          break;
#endif
      case '7':
          activateLeg(FRONT_LEFT);
          break;
      case '8':
          activateLeg(FRONT_RIGHT);
          break;
      case '9':
          activateLeg(BACK_LEFT);
          break;
      case '0':
          activateLeg(BACK_RIGHT);
          break;
      case 'x':
          if (isLegActive()) {
            activeLegX += 5;
            computeAnglesAndMove();
          }
          break;
      case 'X':
          if (isLegActive()) {
            activeLegX -= 5;
            computeAnglesAndMove();
          }
          break;
      case 'y':
          if (isLegActive()) {
            activeLegY += 5;
            computeAnglesAndMove();
          }
          break;
      case 'Y':
          if (isLegActive()) {
            activeLegY -= 5;
            computeAnglesAndMove();
          }
          break;
      case 'z':
          if (isLegActive()) {
            activeLegZ += 5;
            computeAnglesAndMove();
          }
          break;
      case 'Z':
          if (isLegActive()) {
            activeLegZ -= 5;
            computeAnglesAndMove();
          }
          break;
    }
  }
}

void initSerialInteraction() {
  printHelp();
#if defined(DEBUG_AXIS_POS)
  activeAxis = -1;
  activeAxisPos = 0;
#endif
  activeLeg = -1;
}
