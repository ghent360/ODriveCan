/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3 remote.
 */
#include <Arduino.h>

#define PROFILE_LOOP

#include "RemoteInput.h"
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

void resetProcessProfiler() {
  taskLoopDuration = 0;
}
#else
#define PROFILE_CALL(x, v) x
#endif

static const char* sw3ToStr(SW3POS value) {
  switch(value) {
  case SW3_ON: return "on ";
  case SW3_MID: return "mid";
  case SW3_OFF: return "off";
  }
  return "unk";
}

static const char* sw2ToStr(SW2POS value) {
  switch(value) {
  case SW2_ON: return "on ";
  case SW2_OFF: return "off";
  }
  return "unk";
}

static void printRemoteValues() {
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

  remoteInputs.setB1Led(remoteInputs.getB4() == SW2_ON);
  remoteInputs.setB2Led(remoteInputs.getB1() == SW2_ON);
  remoteInputs.setB3Led(remoteInputs.getB2() == SW2_ON);
  remoteInputs.setB4Led(remoteInputs.getB3() == SW2_ON);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  remoteInputs.initPins();
  remoteInputs.begin();

  taskManager.addBack(taskManager.newPeriodicTask(
    1,
    1,
    [](TaskNode*, uint32_t) {
    remoteInputs.readValues();
  }));

  taskManager.addBack(taskManager.newPeriodicTask(
    20,
    250,
    [](TaskNode*, uint32_t) {
    printRemoteValues();
  }));

#ifdef PROFILE_LOOP
  taskManager.addBack(taskManager.newPeriodicTask(
    100,
    5000, // once per 5 seconds
    [](TaskNode*, uint32_t) {
    Serial.print("Task loop ");
    Serial.print (taskLoopDuration);
    uint32_t id = taskManager.getLongestTaskId();
    if (id != (uint32_t)-1) {
      Serial.print(" longest task ID:");
      Serial.print(id);
      Serial.print(" duration ");
      Serial.println(taskManager.getMaxTaskTime());
      taskManager.resetProfiler();
    }
    resetProcessProfiler();
  }));
#endif
}

void loop() {
#ifdef PROFILE_LOOP
  uint32_t startTime, duration;
#endif
  PROFILE_CALL(taskManager.runNext(), taskLoopDuration);
}