/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "globals.h"
#include "kinematics.h"
#include "TaskManager.hpp"
#include "TaskIds.h"

using odrive::AxisState;

extern TaskManager tm;

static int8_t activeAxis = -1;
static float activeAxisPos = 0;
static int8_t activeLeg = -1;
static float activeLegX = 0;
static float activeLegY = 0;
static float activeLegZ = 0;

static void deactivateLeg() {
  activeLeg = -1;
  activeLegX = activeLegY = activeLegZ = 0;
}

static void deactivateAxis() {
  activeAxis = -1;
  activeAxisPos = 0;
}

static void activateLeg(DogLeg leg) {
  deactivateAxis();
  Serial.print("Active leg ");
  Serial.println(getLegName(leg));
  activeLeg = leg;
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

static void printHelp() {
  Serial.println("Help:");
  Serial.println("  '?' - print this message.");
  Serial.println("  'l' - set limits and enter closed loop control mode.");
  Serial.println("  'i' - enter idle mode.");
  Serial.println("  'C' - clear all axis errors.");
  Serial.println("  'g' - modify gains.");
  Serial.println("  '1'..'6' - set axis active.");
  Serial.println("  '7'..'0' - set leg active.");
  Serial.println("  '`' - deactivate selected axis.");
  Serial.println("  '+', '-' - change the active axis position.");
  Serial.println("  'x', 'X', 'y', 'Y', 'z', 'Z' - change the active leg position.");
  Serial.println("  'h', 'H' - move active/all axis to 'home' position.");
  Serial.println("  'c' - clear active axis errors.");
}

static void printAxesHomePos(TaskNode*, uint32_t) {
  for (int idx=0; idx<numAxes; idx++) {
    Serial.print("Axis ");
    Serial.print(idx);
    Serial.print(" (");
    Serial.print(axes[idx].node_id);
    Serial.print(") pos = ");
    Serial.println(axes[idx].enc_est.pos);
  }
}

static void setAxisLimitsAndStart() {
  for(auto& axis: axes) {
    axis.SetLimits(6.0f, 10.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  tm.addBack(tm.newSimpleTask(PrintPosition, 5000, printAxesHomePos));
}

static void setAxisIdle() {
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
  deactivateAxis();
}

static void axesGoHome() {
  for (int idx=0; idx<numAxes; idx++) {
    if (axes[idx].hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
      axes[idx].SetInputPos(jointOffsets[idx]);
    } 
  }
}

static void modifyGains() {
  const float posGainKnee = 20.0f;
  const float posGainHips = 60.0f;
  const float posGainShoulder = 20.0f;
  const float velGain = 0.1f;
  const float integrator = 0.2f;
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

static void moveAxisPos() {
  if (isAxisActive()) {
    activeAxisPos = constrain(activeAxisPos, -2.5, 2.5);
    Serial.print("pos = ");
    Serial.println(activeAxisPos);
    driveJoints(static_cast<DogLegJoint>(activeAxis), activeAxisPos);
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
      case 'H':
          axesGoHome();
          break;
      case 'C':
          for(auto& axis: axes) {
            axis.ClearErrors();
          }
          break;
      case 'g':
          modifyGains();
          break;
      case '`':
          deactivateAxis();
          deactivateLeg();
          break;
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
      case '+':
          if (activeAxis >= 0) {
            activeAxisPos += 0.05;
            moveAxisPos();
          }
          break;
      case '-':
          if (activeAxis >= 0) {
            activeAxisPos -= 0.05;
            moveAxisPos();
          }
          break;
      case 'x':
          if (activeLeg > 0) {
            activeLegX += 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'X':
          if (activeLeg > 0) {
            activeLegX -= 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'y':
          if (activeLeg > 0) {
            activeLegY += 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'Y':
          if (activeLeg > 0) {
            activeLegY -= 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'z':
          if (activeLeg > 0) {
            activeLegZ += 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'Z':
          if (activeLeg > 0) {
            activeLegZ -= 0.1;
            kinematics(static_cast<DogLeg>(activeLeg), activeLegX, activeLegY, activeLegZ, 0, 0, 0);
          }
          break;
      case 'h':
          if (activeAxis >= 0) {
            activeAxisPos = 0;
            moveAxisPos();
          }
          break;
      case 'c':
          if (activeAxis >= 0) {
            axes[activeAxis].ClearErrors();
          }
          break;
    }
  }
}

void initSerialInteraction() {
  printHelp();
  activeAxis = -1;
  activeAxisPos = 0;
}