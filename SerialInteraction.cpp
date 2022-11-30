/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <Arduino.h>
#include "ODriveCan.hpp"
#include "kinematics.h"
#include "TaskManager.hpp"
#include "TaskIds.h"

using odrive::AxisState;

extern TaskManager tm;
int8_t activeAxis = -1;
float activeAxisPos = 0;

void printHelp() {
  Serial.println("Help:");
  Serial.println("  '?' - print this message.");
  Serial.println("  'l' - set limits and enter closed loop control mode.");
  Serial.println("  'i' - enter idle mode.");
  Serial.println("  'H' - move all axis to 'home' position.");
  Serial.println("  'C' - clear all axis errors.");
  Serial.println("  'g' - modify gains.");
  Serial.println("  '1'..'6' - set axis active.");
  Serial.println("  '0' - deactivate selected axis.");
  Serial.println("  '+', '-' - change the active axis position.");
  Serial.println("  'h' - move active axis to 'home' position.");
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
    axis.SetLimits(2.0f, 10.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  tm.addBack(tm.newSimpleTask(PrintPosition, 5000, printAxesHomePos));
}

static void setAxisIdle() {
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
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
  activeAxisPos = constrain(activeAxisPos, -2.5, 2.5);
  Serial.print("pos = ");
  Serial.println(activeAxisPos);
  driveJoints(static_cast<DogLegJoint>(activeAxis), activeAxisPos);
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
      case '0':
          activeAxis = -1;
          activeAxisPos = 0;
          break;
      case '1':
          activeAxis = FRONT_LEFT_KNEE;
          activeAxisPos = 0;
          break;
      case '!':
          activeAxis = FRONT_RIGHT_KNEE;
          activeAxisPos = 0;
          break;
      case '2':
          activeAxis = BACK_LEFT_KNEE;
          activeAxisPos = 0;
          break;
      case '@':
          activeAxis = BACK_RIGHT_KNEE;
          activeAxisPos = 0;
          break;
      case '3':
          activeAxis = FRONT_LEFT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '#':
          activeAxis = FRONT_RIGHT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '4':
          activeAxis = BACK_LEFT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '$':
          activeAxis = BACK_RIGHT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '5':
          activeAxis = FRONT_LEFT_HIP;
          activeAxisPos = 0;
          break;
      case '%':
          activeAxis = FRONT_RIGHT_HIP;
          activeAxisPos = 0;
          break;
      case '6':
          activeAxis = BACK_LEFT_HIP;
          activeAxisPos = 0;
          break;
      case '^':
          activeAxis = BACK_RIGHT_HIP;
          activeAxisPos = 0;
          break;
      case '+':
          if (activeAxis >= 0) {
            activeAxisPos += 0.1;
            moveAxisPos();
          }
          break;
      case '-':
          if (activeAxis >= 0) {
            activeAxisPos -= 0.1;
            moveAxisPos();
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
