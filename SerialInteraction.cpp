/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "globals.h"
#include "JointDriver.h"
#include "Kinematics.h"
#include "StepTrajectory.h"
#include "TaskManager.hpp"
#include "TaskIds.h"

using odrive::AxisState;

//#define DEBUG_AXIS_POS
#define DEBUG_LEG_POS

#if defined(DEBUG_AXIS_POS)
static int8_t activeAxis = -1;
static float activeAxisPos = 0;
#endif
static int8_t activeLeg = -1;
static float activeLegX = 0;
static float activeLegY = 0;
static float activeLegZ = 0;
static uint32_t last = 0;

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
    axis.SetLimits(20.0f, 10.0f); // Should be 6000.0f, 20.0f
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
  constexpr float posGainShin = 20.0f;
  constexpr float posGainHips = 60.0f;
  constexpr float posGainTie = 20.0f;
  constexpr float velGain = 0.1f;
  constexpr float integrator = 0.2f;
  float posGain = 20.0f;
  for (int idx=0; idx<numAxes; idx++) {
    switch(jointClass[idx]) {
      case CLASS_HIP:
        posGain = posGainHips;
        break;
      case CLASS_SHIN:
        posGain = posGainShin;
        break;
      case CLASS_TIE:
        posGain = posGainTie;
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

static void computeAnglesAndMove(float x, float y, float z) {
  float ha, ta, sa;
#if defined(DEBUG_LEG_POS)
  //Serial.print("Leg pos x:");
  //Serial.print(x, 3);
  //Serial.print(" y:");
  //Serial.print(y, 3);
  //Serial.print(" z:");
  //Serial.println(z, 3);
#endif
  bool posShinAngle = (activeLeg == BACK_RIGHT) || (activeLeg == FRONT_RIGHT);
  if ((activeLeg == BACK_LEFT) || (activeLeg == FRONT_LEFT)) {
    x = -x;
  }
  inverseKinematics(x, y, z, posShinAngle, ha, ta, sa);
  float hp, tp, sp;
  hp = -ha * radToPos;
  tp = ta * radToPos;
  sp = sa * radToPos;
  if (!isnan(sa) && !isnan(ta) && !isnan(ha)) {
    switch (activeLeg) {
      case BACK_RIGHT:
        driveJoints(BACK_RIGHT_HIP, hp);
        driveJoints(BACK_RIGHT_TIE, tp);
        driveJoints(BACK_RIGHT_SHIN, sp);
        break;
      case FRONT_RIGHT:
        driveJoints(FRONT_RIGHT_HIP, hp);
        driveJoints(FRONT_RIGHT_TIE, tp);
        driveJoints(FRONT_RIGHT_SHIN, sp);
        break;
      case BACK_LEFT:
        driveJoints(BACK_LEFT_HIP, hp);
        driveJoints(BACK_LEFT_TIE, tp);
        driveJoints(BACK_LEFT_SHIN, sp);
        break;
      case FRONT_LEFT:
        driveJoints(FRONT_LEFT_HIP, hp);
        driveJoints(FRONT_LEFT_TIE, tp);
        driveJoints(FRONT_LEFT_SHIN, sp);
        break;
      default:
        break;
    }
  } else {
#if defined(DEBUG_LEG_POS)
    Serial.print("Leg pos x:");
    Serial.print(x, 3);
    Serial.print(" y:");
    Serial.print(y, 3);
    Serial.print(" z:");
    Serial.println(z, 3);
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
  }
}

void walk(float t) {
  float x, z;
  gaitPos(t, x, z);
  x += activeLegX;
  z += activeLegZ;
  //Serial.print("Walk x=");
  //Serial.print(x, 3);
  //Serial.print(" z=");
  //Serial.println(z, 3);
  computeAnglesAndMove(x, activeLegY, z);
}

void checkSerialInput(TaskNode*, uint32_t now) {
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
      case 'r':
          computeAnglesAndMove(activeLegX, activeLegY, activeLegZ + 50);
          break;
      case 'R':
          initStepCurve(0, 0);
          walk(0);
          break;
      case 'w':
          initStepCurve(0, 0);
          last = now;
          break;
      case 's':
          last = 0;
          break;
#if defined(DEBUG_AXIS_POS)
      case '1':
          activateAxis(FRONT_LEFT_SHIN);
          break;
      case '!':
          activateAxis(FRONT_RIGHT_SHIN);
          break;
      case '2':
          activateAxis(BACK_LEFT_SHIN);
          break;
      case '@':
          activateAxis(BACK_RIGHT_SHIN);
          break;
      case '3':
          activateAxis(FRONT_LEFT_TIE);
          break;
      case '#':
          activateAxis(FRONT_RIGHT_TIE);
          break;
      case '4':
          activateAxis(BACK_LEFT_TIE);
          break;
      case '$':
          activateAxis(BACK_RIGHT_TIE);
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
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
      case 'X':
          if (isLegActive()) {
            activeLegX -= 5;
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
      case 'y':
          if (isLegActive()) {
            activeLegY += 5;
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
      case 'Y':
          if (isLegActive()) {
            activeLegY -= 5;
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
      case 'z':
          if (isLegActive()) {
            activeLegZ += 5;
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
      case 'Z':
          if (isLegActive()) {
            activeLegZ -= 5;
            computeAnglesAndMove(activeLegX, activeLegY, activeLegZ);
          }
          break;
    }
  }
  if (last != 0) {
    float t = float(now - last) / 3000;
    if (t > 1) {
      t = 0;
      last = now;
    }
    if (t < 0) t = 0;
    walk(t);
  }
}

void initSerialInteraction() {
#if defined(DEBUG_AXIS_POS)
  activeAxis = -1;
  activeAxisPos = 0;
#endif
  last = 0;
  activeLeg = -1;
  printHelp();
}
