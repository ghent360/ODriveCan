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
#include "kinematics.h"

using odrive::AxisState;
using odrive::CanMsgData;
using odrive::ODriveAxis;
using odrive::ParseCanMsg;
using odrive::VbusVoltage;

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
static void sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf) {
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
ODriveAxis axes[numAxes] = {
  [FRONT_RIGHT_KNEE] = ODriveAxis( 8, sendCmdCh1),
//  [FRONT_LEFT_KNEE] = ODriveAxis( 7, sendCmdCh0),
  [BACK_RIGHT_KNEE] = ODriveAxis( 6, sendCmdCh1),
//  [BACK_LEFT_KNEE] = ODriveAxis( 5, sendCmdCh0),
  [FRONT_RIGHT_SHOULDER] = ODriveAxis( 4, sendCmdCh1),
//  [FRONT_LEFT_SHOULDER] = ODriveAxis( 3, sendCmdCh0),
  [BACK_RIGHT_SHOULDER] = ODriveAxis(12, sendCmdCh1),
//  [BACK_LEFT_SHOULDER] = ODriveAxis(11, sendCmdCh0),
  [FRONT_RIGHT_HIP] = ODriveAxis( 2, sendCmdCh1),
//  [FRONT_LEFT_HIP] = ODriveAxis( 1, sendCmdCh0),
  [BACK_RIGHT_HIP] = ODriveAxis( 10, sendCmdCh1),
//  [BACK_LEFT_HIP] = ODriveAxis( 9, sendCmdCh0)
};

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
  float posGain;
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

int8_t activeAxis = -1;
float activeAxisPos = 0;

static void moveAxisPos() {
  activeAxisPos = constrain(activeAxisPos, -2.5, 2.5);
  Serial.print("pos = ");
  Serial.println(activeAxisPos);
  driveJoints(static_cast<DogLegJoint>(activeAxis), activeAxisPos);
}

static void checkSerialInput(TaskNode*, uint32_t) {
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
          //activeAxis = FRONT_LEFT_KNEE;
          activeAxis = FRONT_RIGHT_KNEE;
          activeAxisPos = 0;
          break;
      case '2':
          //activeAxis = BACK_LEFT_KNEE;
          activeAxis = BACK_RIGHT_KNEE;
          activeAxisPos = 0;
          break;
      case '3':
          //activeAxis = FRONT_LEFT_SHOULDER;
          activeAxis = FRONT_RIGHT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '4':
          //activeAxis = BACK_LEFT_SHOULDER;
          activeAxis = BACK_RIGHT_SHOULDER;
          activeAxisPos = 0;
          break;
      case '5':
          //activeAxis = FRONT_LEFT_HIP;
          activeAxis = FRONT_RIGHT_HIP;
          activeAxisPos = 0;
          break;
      case '6':
          //activeAxis = BACK_LEFT_HIP;
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
  if (axisIdx >= numAxes) {
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
  activeAxis = -1;
  activeAxisPos = 0;
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
