/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3 remote.
 */
#include <Arduino.h>

#define PROFILE_LOOP

#include "RemoteInput.h"
#include "RemoteDisplay.h"
#include "RemoteRadio.h"
#include "RemoteTouch.h"
#include "TaskManager.hpp"

TaskManager taskManager;
#ifdef PROFILE_LOOP
#define PROFILE_CALL(x, v) \
{\
  startTime = micros();\
  x;\
  duration = micros() - startTime;\
  if (duration > (v)) {\
    (v) = duration;\
  }\
}

static uint32_t taskLoopDuration;
static uint32_t radioPollDuration;

void resetProcessProfiler() {
  taskLoopDuration = 0;
  radioPollDuration = 0;
}
#else
#define PROFILE_CALL(x, v) x
#endif

static const char* sw3ToStr(SW3POS value) {
  switch(value) {
  case SW3_ON: return "ON";
  case SW3_MID: return "MID";
  case SW3_OFF: return "OFF";
  }
  return "unk";
}

static const char* sw2ToStr(SW2POS value) {
  switch(value) {
  case SW2_ON: return "ON";
  case SW2_OFF: return "OFF";
  }
  return "unk";
}

static void updateRemoteValues() {
#if 0
  Serial.print("Input: ");
  Serial.print(remoteInputs.getX1());
  Serial.print("  ");
  Serial.print(remoteInputs.getY1());
  Serial.print("  ");
  Serial.print(remoteInputs.getZ1());
  Serial.print("  ");
  Serial.print(remoteInputs.getX2());
  Serial.print("  ");
  Serial.print(remoteInputs.getY2());
  Serial.print("  ");
  Serial.print(remoteInputs.getZ2());
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW1()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW2()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW3()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW4()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getSW5()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB1()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB2()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB3()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB4()));
  Serial.println();
#endif
  remoteDisplay.setSW1Label(sw3ToStr(remoteInputs.getSW1()));
  remoteDisplay.setSW2Label(sw3ToStr(remoteInputs.getSW2()));
  remoteDisplay.setSW3Label(sw3ToStr(remoteInputs.getSW3()));
  remoteDisplay.setSW4Label(sw3ToStr(remoteInputs.getSW4()));
  remoteDisplay.setSW5Label(sw2ToStr(remoteInputs.getSW5()));
  remoteDisplay.setSW1Active(remoteInputs.getB1() == SW2_ON);
  remoteInputs.setB1Led(remoteInputs.getB4() == SW2_ON);
  remoteInputs.setB2Led(remoteInputs.getB1() == SW2_ON);
  remoteInputs.setB3Led(remoteInputs.getB2() == SW2_ON);
  remoteInputs.setB4Led(remoteInputs.getB3() == SW2_ON);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  remoteInputs.initPins();
  remoteDisplay.initPins();
  remoteRadio.initPins();
  remoteTouch.initPins();

  remoteInputs.begin();
  remoteDisplay.begin();
  remoteRadio.begin();
  remoteTouch.begin();

  taskManager.addBack(taskManager.newPeriodicTask(
    100,
    5,
    [](TaskNode*, uint32_t) { remoteInputs.readStickValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    101,
    100,
    [](TaskNode*, uint32_t) { remoteInputs.readSwitchValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    20,
    66,
    [](TaskNode*, uint32_t) { updateRemoteValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    2,
    66, // 15 fps should be good enough for now
    [](TaskNode*, uint32_t) { remoteDisplay.updateScreen(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    21,
    100,
    [](TaskNode*, uint32_t) { remoteRadio.txData((const uint8_t*)"Hello", 6); }));
#ifdef PROFILE_LOOP
  taskManager.addBack(taskManager.newPeriodicTask(
    100,
    5000, // once per 5 seconds
    [](TaskNode*, uint32_t) {
    Serial.print("Task loop ");
    Serial.print(taskLoopDuration);
    uint32_t id = taskManager.getLongestTaskId();
    if (id != (uint32_t)-1) {
      Serial.print(" longest task ID:");
      Serial.print(id);
      Serial.print(" duration ");
      Serial.println(taskManager.getMaxTaskTime());
      taskManager.resetProfiler();
    }
    Serial.print("Radio poll ");
    Serial.println(radioPollDuration);
    resetProcessProfiler();
  }));
#endif
}

void loop() {
#ifdef PROFILE_LOOP
  uint32_t startTime, duration;
#endif
  PROFILE_CALL(taskManager.runNext(), taskLoopDuration);
  PROFILE_CALL(remoteRadio.poll(), radioPollDuration);
}
