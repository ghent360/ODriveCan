/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include <Arduino.h>
#include "RobotDefinition.h"
#include "globals.h"

static int8_t activeLeg = -1;
//static uint32_t last = 0;

static void activateLeg(DogLeg leg) {
  Serial.print("Active leg ");
  Serial.println(getLegName(leg));
  //robotBody.resetLeg(leg, false);
  activeLeg = leg;
}

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
  Serial.println("  '7'..'0' - set leg active.");
  Serial.println("  'x', 'X', 'y', 'Y', 'z', 'Z' - change the active leg position.");
  Serial.println("  'h' - move active/all axis to 'home' position.");
  if (isLegActive()) {
    Serial.print("Active leg ");
    Serial.println(getLegName(DogLeg(activeLeg)));
  }
}

#if 0
void walk(float t) {
  float x, z;
  traj.gaitPos(t, x, z);
  x += activeLegX;
  z += activeLegZ;
  //Serial.print("Walk x=");
  //Serial.print(x, 3);
  //Serial.print(" z=");
  //Serial.println(z, 3);
  computeAnglesAndMove(x, activeLegY, z);
}
#endif

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
          robotBody.setAllAxesActive();
          break;
      case 'i':
          robotBody.setAllAxesIdle();
          break;
      case 'h':
          robotBody.parkLegs();
          break;
      case 'c':
          for(auto& axis: axes) {
            axis.ClearErrors();
          }
          break;
      case 'g':
          robotBody.modifyAxesGains();
          break;
      case 't':
          robotBody.resetAll();
          break;
      case 'w':
          robotBody.startWalking();
          break;
      case 's':
          robotBody.stopWalking();
          break;
      case 'p':
          robotBody.printPositions();
          break;
#if 0
      case 'r':
          computeAnglesAndMove(activeLegX, activeLegY, activeLegZ + 50);
          break;
      case 'R':
          walk(0);
          break;
      case 'w':
          last = now;
          break;
      case 's':
          last = 0;
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
      case '-':
          activeLeg = -1;
          break;
      case 'x':
          if (isLegActive()) {
            robotBody.incrementX(DogLeg(activeLeg), 5);
          }
          break;
      case 'X':
          if (isLegActive()) {
            robotBody.incrementX(DogLeg(activeLeg), -5);
          }
          break;
      case 'y':
          if (isLegActive()) {
            robotBody.incrementY(DogLeg(activeLeg), 5);
          }
          break;
      case 'Y':
          if (isLegActive()) {
            robotBody.incrementY(DogLeg(activeLeg), -5);
          }
          break;
      case 'z':
          if (isLegActive()) {
            robotBody.incrementZ(DogLeg(activeLeg), 5);
          }
          break;
      case 'Z':
          if (isLegActive()) {
            robotBody.incrementZ(DogLeg(activeLeg), -5);
          }
          break;
    }
  }
  if (isLegActive()) {
    Serial.print("Leg position error ");
    Serial.println(robotBody.getPosError(DogLeg(activeLeg)), 3);
  }
#if 0  
  if (last != 0) {
    float t = float(now - last) / 3000;
    if (t > 1) {
      t = 0;
      last = now;
    }
    if (t < 0) t = 0;
    walk(t);
  }
#endif
}

void initSerialInteraction() {
//  last = 0;
  activeLeg = -1;
  printHelp();
}
