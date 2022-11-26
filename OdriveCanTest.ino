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

using odrive::AxisState;
using odrive::CanMsgData;
using odrive::ODriveAxis;
using odrive::ParseCanMsg;
using odrive::VbusVoltage;

// AXIS class definitions:
enum AxisClass {
  CLASS_KNEE = 0x1,
  CLASS_HIP  = 0x2,
  CLASS_SHOULDER = 0x3
};

enum AxisSide {
  SIDE_LEFT = 0x10,
  SIDE_RIGHT = 0x20
};

enum AxisLocation {
  LOC_FRONT = 0x100,
  LOC_BACK  = 0x200
};

enum PeriodicTaskId {
  StateOne,
  StateTwo,
  StateThreeConnection,
  StateThreeVoltage,
  StateThreeSerial,
};

enum SimpleTaskId {
  PrintPosition = 1000
};

SAME51_CAN can;
TaskManager tm;

// Function that interfaces ODrive with our can hardware. If you want to
// use multiple CAN buses you would need one for each bus.
//
// Note that sometimes it is possible to send too many messages at once
// and overwhelm the TX queue, this code detects such situation and adds
// a delay, so the message can be transmitted correctly.
static void sendCmdCh0(uint32_t canId, uint8_t len, uint8_t *buf) {
  uint8_t ret;
  do {
    ret = can.sendMsgBuf(canId, 0, len, buf);
    if (ret == CAN_FAILTX) {  // TX queue is full, wait a bit.
      delayMicroseconds(200); // Time to transmit 200 bits @ 1MBit/s
    }
  } while (ret == CAN_FAILTX);

  if (ret != CAN_OK) {
    Serial.print("Failed to send message ID ");
    Serial.print(canId, HEX);
    Serial.print(" error ");
    Serial.println(ret);
  }
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

static AxisClass getAxisClass(uint32_t userId) {
  switch (userId & 0xf) {
    case 0x1: return CLASS_KNEE;
    case 0x2: return CLASS_HIP;
    case 0x3:
    default:
      return CLASS_SHOULDER;
  }
}

static AxisSide getAxisSide(uint32_t userId) {
  switch (userId & 0xf0) {
    case 0x10: return SIDE_LEFT;
    case 0x20:
    default:
      return SIDE_RIGHT;
  }
}

static AxisLocation getAxisLocation(uint32_t userId) {
  switch (userId & 0xf00) {
    case 0x100: return LOC_FRONT;
    case 0x200:
    default:
      return LOC_BACK;
  }
}

static const struct {
  uint32_t node_id;
  float home_pos;
} axesHomePosition[] = {
  {1, 0.31},
  {3, -0.08},
  {5, -0.18},
  {7, 0.22},
  {9, 0.27},
  {11, -0.06}
};

// We communicate with 3 ODrive boards, each board has 2 axes. Each axis
// has to be setup separately. If you have boards that are connected to a
// different CAN bus, you would need separate CAN callback for each bus.
// For example SendCmdCh1.
//
// It is required to have unique node_id even if some axis are on a
// separate CAN bus. Otherwise the parsing code has to be modified to 
// process each CAN bus separately.
//
// On my setup the axes have even node_ids starting with 1. These have to
// be configured using the odrivetool.
ODriveAxis axes[] = {
  ODriveAxis( 1, SIDE_LEFT | LOC_FRONT | CLASS_HIP, sendCmdCh0),
  ODriveAxis( 3, SIDE_LEFT | LOC_FRONT | CLASS_SHOULDER, sendCmdCh0),
  ODriveAxis( 5, SIDE_LEFT | LOC_BACK  | CLASS_KNEE, sendCmdCh0),
  ODriveAxis( 7, SIDE_LEFT | LOC_FRONT | CLASS_KNEE, sendCmdCh0),
  ODriveAxis( 9, SIDE_LEFT | LOC_BACK  | CLASS_HIP, sendCmdCh0),
  ODriveAxis(11, SIDE_LEFT | LOC_BACK  | CLASS_SHOULDER, sendCmdCh0),
};

#define NUM_AXES (sizeof(axes)/sizeof(axes[0]))

// Note this should be non-blocking code, if there are no CAN
// messages in the RX queue, readMsgBuf should return some error
// but not wait for messages to appear.
static void readAndProcessCan() {
  uint32_t id;
  uint8_t len;
  static uint8_t buf[8];

  if (can.readMsgBuf(&id, &len, buf) == CAN_OK) {
    bool parseSuccess = ParseCanMsg(axes, id, len, buf);
    if (!parseSuccess) {
      printCanMessage(id, len, buf);
    }
  }
  // Add reading from other CAN busses here, process in similar fashion.
}

/*
 * The following code is a basic three state machine. We start with the state
 * where we wait for all axes to become alive. The first state is implemented
 * in the checkAllAxesArePresent periodical task. When all axes become available
 * we switch to the second state.
 * 
 * The second state is implemented in the clearErrorsAndSwitchToStateThree periodical
 * task. It checks all axis error states, if any axis reports an error it calls 
 * ClearErrors and tries again. When all axes report error free state it switches to
 * state three. Also it checks if any axis gets disconnected it switches back to state
 * one.
 * 
 * In the third state we execute three periodical tasks:
 *   checkAxisConnection  - check if axes are loosing heartbeat and if so revert
 *                          to the first state.
 *   checkAxisVbusVoltage - periodically request the vbus voltage for each axis
 *                          if the voltage is too low it calls EStop on all axes.
 *   checkSerialInput     - read serial input and perform actions.
 */
static void checkAllAxesArePresent(TaskNode*, uint32_t);

static void printHelp() {
  Serial.println("Help:");
  Serial.println("  '?' - print this message.");
  Serial.println("  'l' - set limits and enter closed loop control mode.");
  Serial.println("  'i' - enter idle mode.");
  Serial.println("  'h' - move all axis to 'home' position.");
  Serial.println("  'g' - modify gains.");
}

static void setAxisLimitsAndStart() {
  for(auto& axis: axes) {
    axis.SetLimits(60.0f, 10.0f); // Should be 6000.0f, 20.0f
    axis.SetState(AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
}

static void setAxisIdle() {
  for(auto& axis: axes) {
    axis.SetState(AxisState::AXIS_STATE_IDLE);
  }
}

static void axesGoHome() {
  for(auto& axis: axes) {
    for(auto& pos: axesHomePosition) {
      if (pos.node_id == axis.node_id) {
        if (axis.hb.state == AxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
          axis.SetInputPos(pos.home_pos);
        }
        break;
      }
    }
  }
}

static void modifyGains() {
  const float posGainKnee = 20.0f;
  const float posGainHips = 60.0f;
  const float posGainShoulder = 20.0f;
  const float velGain = 0.1f;
  const float integrator = 0.2f;
  for(auto& axis: axes) {
    switch(getAxisClass(axis.user_id)) {
      case CLASS_HIP:
        axis.SetPosGain(posGainHips);
        break;
      case CLASS_KNEE:
        axis.SetPosGain(posGainKnee);
        break;
      case CLASS_SHOULDER:
        axis.SetPosGain(posGainShoulder);
        break;
    }
    axis.SetVelGains(velGain, integrator);
  }
}

static void checkSerialInput(TaskNode*, uint32_t) {
  if (Serial.available()) {
    auto ch = Serial.read();
    if (ch < 32) return;
    Serial.println((char)ch); // Echo the incomming character.
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
      case 'g':
          modifyGains();
          break;
    }
  }
}

static void axisVbusValueCheck(ODriveAxis&, VbusVoltage& v) {
  if (v.val < 19.4f) { // Cut off voltage for 6S battery.
    for(auto& axis: axes) {
      axis.EStop(); // Call EStop to reduce the axis power consumption.
    }
    Serial.println("Battery voltage low.");
  }
/*
  Serial.print("Axis ");
  Serial.print(axis.node_id);
  Serial.print(" vbus=");
  Serial.println(v.val);
*/  
}

static void checkAxisVbusVoltage(TaskNode*, uint32_t) {
  static uint8_t axisIdx = 0;
  axes[axisIdx].RequestVbusVoltage();
  axisIdx++;
  if (axisIdx >= NUM_AXES) {
    axisIdx = 0;
  }
}

static void checkAxisConnection(TaskNode* self, uint32_t) {
  bool allAlive = true;
  for(auto& axis: axes) {
    axis.hb.PeriodicCheck(axis);
    if (!axis.hb.alive) {
      Serial.print("Lost connection to axis ");
      Serial.println(axis.node_id);
      allAlive = false;
    }
    if (axis.hb.error != 0) {
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.print(" error: 0x");
      Serial.println(axis.hb.error, HEX);
    }
  }
  if (!allAlive) {
    // Switch back to first state:
    tm.remove(tm.findById(StateThreeSerial), true); // Remove the checkSerialInput task.
    // Remove the voltage checking callbacks.
    for(auto& axis: axes) {
      axis.vbus.SetCallback(nullptr);
    }
    tm.remove(tm.findById(StateThreeVoltage), true); // Remove the checkAxisVbusVoltage task.
    Serial.println("Waiting for odrives to connect...");
    tm.addBack(tm.newPeriodicTask(StateOne, 100, checkAllAxesArePresent));
    tm.remove(self, true); // Remove the checkAxisConnection task.
  }
}

// Sometimes we get axis errors on startup. Clear the errors, if all axes
// are error free switch to state three. If we loose any axis switch to
// state one.
static void clearErrorsAndSwitchToStateThree(TaskNode* self, uint32_t) {
  // We have all axes available, if there are any error states clear them.
  bool allClear = true;
  for(auto& axis: axes) {
    axis.hb.PeriodicCheck(axis);
    if (!axis.hb.alive) {
      Serial.print("Lost connection to axis ");
      Serial.println(axis.node_id);
      // Lost axis connection, back to square one:
      Serial.println("Waiting for odrives to connect...");
      tm.addBack(tm.newPeriodicTask(StateOne, 100, checkAllAxesArePresent));
      tm.remove(self, true);
      return;
    }
    if (axis.hb.error != 0) {
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.print(" error: 0x");
      Serial.println(axis.hb.error, HEX);
      axis.ClearErrors();
      allClear = false;
    }
  }
  if (!allClear) {
    // We cleared some axis errors, wait and try again.
    return;
  }
  // All is good switch to state three.
  Serial.println("All odrives active...");
  printHelp();
  // Switch to second state
  tm.addBack(tm.newPeriodicTask(StateThreeConnection, 150, checkAxisConnection));
  for(auto& axis: axes) {
    axis.vbus.SetCallback(axisVbusValueCheck);
  }
  tm.addBack(tm.newPeriodicTask(StateThreeVoltage, 1000, checkAxisVbusVoltage));
  tm.addBack(tm.newPeriodicTask(StateThreeSerial, 10, checkSerialInput));
  tm.remove(self, true); // Remove the clearErrorsAndSwitchToStateThree task.
}

// This function implements state one - we wait until we receive heartbeat
// message from each axis in the list, then we switch to state two.
static void checkAllAxesArePresent(TaskNode* self, uint32_t) {
  bool allAlive = true;
  for(auto& axis: axes) {
    if (!axis.hb.alive) {
      allAlive = false;
      break;
    }
  }
  if (allAlive) {
    // Switch to state two.
    tm.addBack(tm.newPeriodicTask(StateTwo, 200, clearErrorsAndSwitchToStateThree));
    tm.remove(self, true); // Remove the checkAllAxesArePresent task.
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
  Serial.println("Waiting for odrives to connect...");
  // Start in state one
  tm.addFront(tm.newPeriodicTask(StateOne, 100, checkAllAxesArePresent));
}

void loop() {
  readAndProcessCan();
  tm.runNext(millis());
}
