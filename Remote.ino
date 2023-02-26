/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3 remote.
 */
#include <Arduino.h>

//#define PROFILE_LOOP

#include "Fixed.hpp"
#include "RemoteInput.h"
#include "RemoteDisplay.h"
#include "RemoteProtocol.h"
#include "RemoteRadio.h"
#include "RemoteTaskIds.h"
#include "RemoteTouch.h"
#include "TaskManager.hpp"

TaskManager taskManager;

using BatteryVoltage6S = Fixed<uint8_t, 8, 5, 18>;
using BatteryVoltage2S = Fixed<uint8_t, 8, 6, 6>;

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

static void sendRemotePacket() {
  struct TxDataPacket data;
  data.state1.sw1 = (uint8_t)remoteInputs.getSW1();
  data.state1.sw2 = (uint8_t)remoteInputs.getSW2();
  data.state1.sw3 = (uint8_t)remoteInputs.getSW3();
  data.state1.sw4 = (uint8_t)remoteInputs.getSW4();
  data.state2.sw5 = (remoteInputs.getSW5() == SW2_ON);
  data.state2.reserved = 0;
  data.x1 = remoteInputs.getX1();
  data.y1 = remoteInputs.getY1();
  data.z1 = remoteInputs.getZ1();
  data.x2 = remoteInputs.getX2();
  data.y2 = remoteInputs.getY2();
  data.z2 = remoteInputs.getZ2();
  data.cmd = CMD_NOOP;

  remoteRadio.txData((const uint8_t*)&data, sizeof(data));
}

static void receiveRemotePacket(const uint8_t* data, uint8_t len) {
  const struct RxPacket* rxPacket =
    reinterpret_cast<const struct RxPacket*>(data);

  if (len >= sizeof(RxPacketHdr)) {
    BatteryVoltage6S s6voltage;
    BatteryVoltage2S s2voltage;
    s6voltage = BatteryVoltage6S::fromBytes(&rxPacket->hdr.b1_voltage, 1);
    remoteDisplay.setBus1BatteryVoltage(s6voltage);
    s6voltage = BatteryVoltage6S::fromBytes(&rxPacket->hdr.b2_voltage, 1);
    remoteDisplay.setBus3BatteryVoltage(s6voltage);
    s2voltage = BatteryVoltage2S::fromBytes(&rxPacket->hdr.rx_voltage, 1);
    remoteDisplay.setTeensyBatteryVoltage(s2voltage);
    remoteDisplay.setSW1Active(rxPacket->hdr.state.walk);
    remoteDisplay.setSW5Active(rxPacket->hdr.state.control);
  }
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
  switch(remoteInputs.getSW1()) {
    case SW3_OFF: remoteDisplay.setSW1Label("WALK"); break;
    case SW3_MID: remoteDisplay.setSW1Label("STOP"); break;
    case SW3_ON: remoteDisplay.setSW1Label("RESET"); break;
  }
  switch(remoteInputs.getSW2()) {
    case SW3_OFF: remoteDisplay.setSW2Label("STATIC"); break;
    case SW3_MID: remoteDisplay.setSW2Label("WALK"); break;
    case SW3_ON: remoteDisplay.setSW2Label("PARK"); break;
  }
  remoteDisplay.setSW3Label(sw3ToStr(remoteInputs.getSW3()));
  remoteDisplay.setSW4Label(sw3ToStr(remoteInputs.getSW4()));
  remoteDisplay.setSW5Label(remoteInputs.getSW5() ? "ENGAGED" : "IDLE");
//  remoteDisplay.setX1(-float(remoteInputs.getX1()) / 2000);
//  remoteDisplay.setY1(-float(remoteInputs.getY1()) / 2000);
  // B1 clicked
  if (remoteInputs.B1clicked()) {
    remoteDisplay.controller().back();
    remoteInputs.B1reset();
  }
  if (remoteInputs.B2clicked()) {
    remoteDisplay.controller().prev();
    remoteInputs.B2reset();
  }
  if (remoteInputs.B3clicked()) {
    remoteDisplay.controller().next();
    remoteInputs.B3reset();
  }
  if (remoteInputs.B4clicked()) {
    remoteDisplay.controller().select();
    remoteInputs.B4reset();
  }
  sendRemotePacket();
}

void setup() {
  Serial.begin(115200);

  remoteInputs.initPins();
  remoteDisplay.initPins();
  remoteRadio.initPins();
  remoteTouch.initPins();

  remoteInputs.begin();
  remoteDisplay.begin();
  remoteRadio.begin();
  remoteTouch.begin();

  remoteRadio.setRxDataCallback(receiveRemotePacket);

  taskManager.addBack(taskManager.newPeriodicTask(
    ReadStickValues,
    5,
    [](TaskNode*, uint32_t) { remoteInputs.readStickValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    ReadSwitchValues,
    100,
    [](TaskNode*, uint32_t) { remoteInputs.readSwitchValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    SendRemoteValues,
    66,
    [](TaskNode*, uint32_t) { updateRemoteValues(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    DisplayUpdate,
    66, // 15 fps should be good enough for now
    [](TaskNode*, uint32_t) { remoteDisplay.updateScreen(); }));
#ifdef PROFILE_LOOP
  taskManager.addBack(taskManager.newPeriodicTask(
    ReportProfileStats,
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
