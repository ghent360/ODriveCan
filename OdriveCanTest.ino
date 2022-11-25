/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
 * This is a very basic example of communicating with 3 ODrive boards
 * over CAN on SAME51 board. The example would request vbus voltage from
 * each axis and print it on the serial console. It would also verify
 * heartbeat messages are received from each axis or it will print that
 * the axis is unavailable.
*/
#include <same51_can.h>
#include "ODriveCan.hpp"
#include "TaskManager.hpp"

using odrive::CanMsgData;
using odrive::ODriveAxis;
using odrive::ParseCanMsg;
using odrive::VbusVoltage;

SAME51_CAN can;
TaskManager tm;

// Function that interfaces ODrive with our can hardware. If you want to
// use multiple CAN buses you would need one for each bus.
static void sendCmdCh0(uint32_t canId, uint8_t len, uint8_t *buf) {
  can.sendMsgBuf(canId, 0, len, buf);
}

// Helper to print CAN messages we can not parse.
static void printCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
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
// It is required to have unique node_id even if some axis are on a
// separate CAN bus. Otherwise the parsing code has to be modified to 
// process each CAN bus separately.
//
// On my setup the axes have even node_ids starting with 1. These have to
// be configured using the odrivetool.
ODriveAxis axes[] = {
  ODriveAxis(1, sendCmdCh0),
  ODriveAxis(3, sendCmdCh0),
  ODriveAxis(5, sendCmdCh0),
  ODriveAxis(7, sendCmdCh0),
  ODriveAxis(9, sendCmdCh0),
  ODriveAxis(11, sendCmdCh0),
};

#define NUM_AXES (sizeof(axes)/sizeof(axes[0]))

void readAndProcessCan() {
  uint32_t id;
  uint8_t len;
  static uint8_t buf[8];

  if (can.readMsgBuf(&id, &len, buf) == CAN_OK) {
    bool parseSuccess = ParseCanMsg(axes, id, len, buf);
    if (!parseSuccess) {
      printCanMessage(id, len, buf);
    }
  }
}

/*
 * The following code is a basic two state machine. We start with the state
 * where we wait for all axes to become alive. The first state is implemented
 * in the checkAllAxesArePresent periodical task. When all axes become available
 * we switch to the second state.
 * 
 * In the secomd state we execute two periodical tasks:
 *   checkAxisConnection - check if axes are loosing heartbeat and if so revert
 *                         to the fisrt state.
 *   checkAxisVbusVoltage - periodically request the vbus voltage for each axis
 *                          at the moment this just prints to voltage.
 */
void checkAllAxesArePresent(TaskNode* self, uint32_t timeNow);

void axisVbusValueCheck(ODriveAxis& axis, VbusVoltage& v) {
  Serial.print("Axis ");
  Serial.print(axis.node_id);
  Serial.print(" vbus=");
  Serial.println(v.val);
}

void checkAxisVbusVoltage(TaskNode* self, uint32_t timeNow) {
  static uint8_t axisIdx = 0;
  if (axes[axisIdx].hb.alive) {
    axes[axisIdx].RequestVbusVoltage();
  }
  axisIdx++;
  if (axisIdx >= NUM_AXES) {
    axisIdx = 0;
  }
}

void checkAxisConnection(TaskNode* self, uint32_t timeNow) {
  bool allAlive = true;
  for(auto& axis: axes) {
    axis.hb.PeriodicCheck(axis);
    if (!axis.hb.alive) {
      Serial.print("Lost connection to axis ");
      Serial.println(axis.node_id);
      allAlive = false;
    }
  }
  if (!allAlive) {
    // Switch back to first state
    for(auto& axis: axes) {
      axis.vbus.SetCallback(nullptr);
    }
    tm.remove(tm.findById(2), true);
    tm.addBack(tm.newPeriodicTask(0, 100, checkAllAxesArePresent));
    tm.remove(self, true);
  }
}

void checkAllAxesArePresent(TaskNode* self, uint32_t timeNow) {
  bool allAlive = true;
  for(auto& axis: axes) {
    if (!axis.hb.alive) {
      allAlive = false;
      break;
    }
  }
  if (allAlive) {
    // We have all axes available, if there are any error states clear them.
    for(auto& axis: axes) {
      if (axis.hb.error != 0) {
        Serial.print("Axis ");
        Serial.print(axis.node_id);
        Serial.print(" error: 0x");
        Serial.println(axis.hb.error, HEX);
        axis.ClearErrors();
      }
    }
    // Switch to second state
    tm.addBack(tm.newPeriodicTask(1, 150, checkAxisConnection));
    for(auto& axis: axes) {
      axis.vbus.SetCallback(axisVbusValueCheck);
    }
    tm.addBack(tm.newPeriodicTask(2, 1000, checkAxisVbusVoltage));
    tm.remove(self, true);
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
  // Start in state one
  tm.addFront(tm.newPeriodicTask(0, 100, checkAllAxesArePresent));
}

void loop() {
  readAndProcessCan();
  tm.runNext(millis());
}
