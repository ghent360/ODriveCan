/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
 * This is a very basic example of communicating with 12 ODrive boards
 * over CAN on SAME51/Teensy4.1 board. The example would request vbus
 * voltage from each axis and print it on the serial console. It would 
 * also verify heartbeat messages are received from each axis or it will
 * print that the axis is unavailable.
 * 
 * There is a simple serial interface, where one can start all axes in
 * closed loop control more, or ser it ti idle.
*/
#include "ODriveCan.hpp"
#include "CanInterface.h"
#include "TaskManager.hpp"
#include "kinematics.h"

using odrive::AxisState;
using odrive::ODriveAxis;
using odrive::VbusVoltage;

// This class implements a very basic cooperative multitasking controller.
TaskManager tm;

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
  Serial.println("  '?' or 'h' - print this message.");
  Serial.println("  'l'        - set limits and enter closed loop control mode.");
  Serial.println("  'i'        - enter idle mode.");
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
      case 'h':
          printHelp();
          break;
      case 'l':
          setAxisLimitsAndStart();
          break;
      case 'i':
          setAxisIdle();
          break;
    }
  }
}

static void axisVbusValueCheck(
  ODriveAxis& axis,
  VbusVoltage&,
  VbusVoltage& v) {
  if (v.val < 19.4f) { // Cut off voltage for 6S battery.
    for(auto& a: axes) {
      a.EStop(); // Call EStop to reduce the axis power consumption.
    }
    Serial.print("Axis ");
    Serial.print(axis.node_id);
    Serial.print(" battery voltage low: ");
    Serial.println(v.val);
  }
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
    tm.remove(tm.findById(3), true); // Remove the checkSerialInput task.
    // Remove the voltage checking callbacks.
    for(auto& axis: axes) {
      axis.vbus.SetCallback(nullptr);
    }
    tm.remove(tm.findById(2), true); // Remove the checkAxisVbusVoltage task.
    Serial.println("Waiting for odrives to connect...");
    tm.addBack(tm.newPeriodicTask(0, 100, checkAllAxesArePresent));
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
      tm.addBack(tm.newPeriodicTask(0, 100, checkAllAxesArePresent));
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
  tm.addBack(tm.newPeriodicTask(1, 150, checkAxisConnection));
  for(auto& axis: axes) {
    axis.vbus.SetCallback(axisVbusValueCheck);
  }
  tm.addBack(tm.newPeriodicTask(2, 1000, checkAxisVbusVoltage));
  tm.addBack(tm.newPeriodicTask(3, 10, checkSerialInput));
  tm.remove(self, true); // Remove the clearErrorsAndSwitchToStateThree task.
}

// This function implements state one - we wait until we receive heartbeat
// message from each axis in the list, then we switch to state two.
static void checkAllAxesArePresent(TaskNode* self, uint32_t) {
  bool allAlive = true;
  for(auto& axis: axes) {
    axis.hb.PeriodicCheck(axis);
    if (!axis.hb.alive) {
      allAlive = false;
    }
  }
  if (allAlive) {
    // Switch to state two.
    tm.addBack(tm.newPeriodicTask(1000, 200, clearErrorsAndSwitchToStateThree));
    tm.remove(self, true); // Remove the checkAllAxesArePresent task.
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  canInit();
  Serial.println("Waiting for odrives to connect...");
  // Start in state one
  tm.addFront(tm.newPeriodicTask(0, 100, checkAllAxesArePresent));
}

void loop() {
  readAndProcessCan();
  tm.runNext(millis());
}
