/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include <same51_can.h>
#include "ODriveCan.hpp"
#include "PeriodicTimer.hpp"

using odrive::CanMsgData;
using odrive::ODriveAxis;
using odrive::ParseCanMsg;
using odrive::VbusVoltage;

SAME51_CAN can;

// Function that interfaces ODrive with our can hardware. If you want to
// use multiple CAN buses you would need one for each bus.
static void SendCmdCh0(uint32_t canId, uint8_t len, uint8_t *buf) {
  can.sendMsgBuf(canId, 0, len, buf);
}

// Helper to print CAN messages we can not parse.
static void PrintCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
  Serial.print("Id: ");
  Serial.print(id);
  Serial.print(" Len: ");
  Serial.print(len);
  Serial.print(" Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(", ");
  }
  Serial.println();
}

// We communicate with 3 ODrive boards, each board has 2 axes. Each axis
// has to be setup separately. If you have boards that are connected to a
// different CAN bus, you would need separate callback for each bus. For
// example SendCmdCh1.
//
// It is requered to have unique node_id even if some axis are on a
// separate CAN bus. Otherwise the parsing code has to be modified to 
// process each CAN bus separately.
ODriveAxis axesCh0[] = {
  ODriveAxis(1, SendCmdCh0),
  ODriveAxis(3, SendCmdCh0),
  ODriveAxis(5, SendCmdCh0),
  ODriveAxis(7, SendCmdCh0),
  ODriveAxis(9, SendCmdCh0),
  ODriveAxis(11, SendCmdCh0),
};

PeriodicTimer myTimers[] = {
  // Check that hearthbeat message was received every 150ms.
  PeriodicTimer(150, [](uint32_t)->void {
    for(auto& axis : axesCh0) {
      axis.hb.PeriodicCheck(axis);
    }
  }),
  // Request each axis vbus voltage every second and cycle trough
  // the axes. Only request vbus voltage is the axis is alive.
  PeriodicTimer(1000, [](uint32_t)->void {
    static uint8_t axis = 0;
    if (axesCh0[axis].hb.alive) {
      axesCh0[axis].RequestVbusVoltage();
    }
    axis++;
    if (axis > 5) axis = 0;
  })
};

// Helper to parse incomming CAN messages or print if parsing was
// nos successful.
void ProcessCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
  bool parseSuccess = ParseCanMsg(axesCh0, id, len, buf);
  if (!parseSuccess) {
    PrintCanMessage(id, len, buf);
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  uint8_t ret;
  ret = can.begin(MCP_ANY, CAN_1000KBPS, MCAN_MODE_CAN);
  if (ret == CAN_OK) {
      Serial.println("CAN Initialized Successfully!");
  } else {
      Serial.println("Error Initializing CAN...");
      while(1);
  }
  // Setup callbcaks, when asix becomes unavailable or
  // when we receive the vbus voltage data.
  for(auto& axis : axesCh0) {
    axis.vbus.SetCallback([](ODriveAxis& axis, VbusVoltage& v) { 
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.print(" vbus = ");
      Serial.println(v.val);
    });
    axis.hb.SetUnreachableCallback([](ODriveAxis& axis) {
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.println(" unreachable");
    });
  }
  StartAllTimers(myTimers, millis());
}

void loop() {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];

  // process periodic timers
  CheckAllTimers(myTimers, millis());
  // check for CAN messages and precess them.
  if (can.readMsgBuf(&id, &len, buf) == CAN_OK) {
    ProcessCanMessage(id, len, buf);
  }
}
