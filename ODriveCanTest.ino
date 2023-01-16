/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3.
*/

// Enable profiling of the main loop code
//#define PROFILE_LOOP

#include "ODriveCan.hpp"
#include "TaskManager.hpp"
#include "CanInterface.h"
#include "SerialInteraction.h"
#include "TaskIds.h"
#include "VoltageMonitor.h"
#include "globals.h"
#include "Display.h"
#include "Radio.h"

using odrive::EncoderEstimate;
using odrive::ODriveAxis;
using odrive::VbusVoltage;

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

static uint32_t canProcessDuration;
static uint32_t radioProcessDuration;
static uint32_t taskLoopDuration;
static uint32_t yieldDuration;

void resetProcessProfiler() {
  canProcessDuration = 0;
  radioProcessDuration = 0;
  taskLoopDuration = 0;
  yieldDuration = 0;
}
#else
#define PROFILE_CALL(x, v) x
#endif

/*
 * The following code is a basic three state machine. We start with the state
 * where we wait for all axes to become alive. The first state is implemented
 * in the checkAllAxesArePresent periodical task. When all axes become available
 * we switch to the second state. In the first state there is also the 
 * reportAxesNotPresent task, that reports axes that are not responding every 5 seconds.
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
 *   checkSerialInput     - read serial input and perform actions. (defined in SerialInteraction.cpp)
 */
static void checkAllAxesArePresent(TaskNode*, uint32_t);
static void reportAxesNotPresent(TaskNode* self, uint32_t);
static void startStateOne();
static void startStateTwo();
static void startStateThree();

//RadioV2 radio(NRF24_CE_PIN, NRF24_CS_PIN, NRF24_IRQ_PIN);

static void panic() {
  for(auto& axis: axes) {
    axis.EStop(); // Call EStop to reduce the axis power consumption.
  }
}

static void axisVbusValueCheck(
  ODriveAxis& axis, VbusVoltage&, VbusVoltage& newV) {
  if ((axis.node_id % 2) != 0) {
    display.setBus1BatteryVoltage(newV.val);
  } else {
    display.setBus3BatteryVoltage(newV.val);
  }

  if (newV.val < (6*cellMinVoltage)) {
    Serial.println("ODrive battery voltage too low (estop)");
    panic();
  }
}

#ifdef AXIS_POS_DISPLAY
static void axisPosUpdate(
  ODriveAxis& axis, EncoderEstimate&, EncoderEstimate& newV) {
  display.setJoinPos(axis.node_id, newV.pos);
}
#endif

static void checkAxisVbusVoltage(TaskNode*, uint32_t) {
  static uint8_t axisIdx = FRONT_RIGHT_HIP;
  axes[axisIdx].RequestVbusVoltage();
  axisIdx++;
  // Check battery voltage only the axes with node ID 1 and 2.
  // Axes are connected to battery bus bars and the first two
  // would represent the voltage on both bus bars. That is sufficient
  // for monitoring.
  if (axisIdx > FRONT_LEFT_HIP) {
    axisIdx = FRONT_RIGHT_HIP;
  }
}

static void checkBatteryVoltage(TaskNode*, uint32_t) {
  float batVoltage = voltageMonitor.readBatteryVoltage();
  display.setTeensyBatteryVoltage(batVoltage);
  if (batVoltage < (2*cellMinVoltage)) {
    Serial.println("Battery voltage too low (estop)");
    panic();
    delay(250);
    voltageMonitor.lowPowerMode();
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
      String status("Axis ");
      status += String(axis.node_id);
      status += " error: 0x";
      status += String(axis.hb.error, HEX);
      Serial.println(status);
      display.setCanStatus(status);
    }
  }
  if (!allAlive) {
    // Clean state three and switch back to first state:
    // Remove the checkSerialInput task.
    taskManager.remove(taskManager.findById(StateThreeSerial), true);
    // Remove the voltage checking callbacks.
    for(auto& axis: axes) {
      axis.vbus.SetCallback(nullptr);
      axis.enc_est.SetCallback(nullptr);
    }
    // Remove the checkAxisVbusVoltage task.
    taskManager.remove(taskManager.findById(StateThreeODriveVoltage), true);
    // Remove the checkBatteryVoltage task.
    taskManager.remove(taskManager.findById(StateThreeBatteryVoltage), true);
    // Remove the checkAxisConnection task.
    taskManager.remove(self, true);
    startStateOne();
  }
}

static void startStateThree() {
  Serial.println("All odrives active...");
  display.setCanStatus("Ready");
  display.setCanStatusColor(ST7735_GREEN);
  initSerialInteraction();
  taskManager.addBack(
    taskManager.newPeriodicTask(
      StateThreeConnection, 750, checkAxisConnection));
  for(auto& axis: axes) {
    axis.vbus.SetCallback(axisVbusValueCheck);
#ifdef AXIS_POS_DISPLAY
    axis.enc_est.SetCallback(axisPosUpdate);
#endif
  }
  taskManager.addBack(
    taskManager.newPeriodicTask(
      StateThreeODriveVoltage, 1000, checkAxisVbusVoltage));
  taskManager.addBack(
    taskManager.newPeriodicTask(
      StateThreeBatteryVoltage, 1000, checkBatteryVoltage));
  taskManager.addBack(
    taskManager.newPeriodicTask(StateThreeSerial, 20, checkSerialInput));
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
      taskManager.remove(self, true);
      startStateOne();
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
  // Remove the clearErrorsAndSwitchToStateThree task.
  taskManager.remove(self, true);
  startStateThree();
}

static void startStateTwo() {
  display.setCanStatus("Initializing axes");
  display.setCanStatusColor(ST7735_YELLOW);
  taskManager.addBack(
    taskManager.newPeriodicTask(
      StateTwo, 250, clearErrorsAndSwitchToStateThree));
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
    // Remove the reportAxesNotPresent task.
    taskManager.remove(taskManager.findById(StateOneReport), true);
    // Remove the checkAllAxesArePresent task.
    taskManager.remove(self, true);
    startStateTwo();
  }
}

static void reportAxesNotPresent(TaskNode* self, uint32_t) {
  String status("Axis:");
  for(auto& axis: axes) {
    if (!axis.hb.alive) {
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.println(" not responding");
      status += " ";
      status += String(axis.node_id);
    }
  }
  display.setCanStatus(status);
}

static void startStateOne() {
  Serial.println("Waiting for odrives to connect...");
  display.setCanStatus("Waiting to connect");
  display.setCanStatusColor(ST7735_RED);
  taskManager.addBack(
    taskManager.newPeriodicTask(StateOneCheck, 250, checkAllAxesArePresent));
  taskManager.addBack(
    taskManager.newPeriodicTask(StateOneReport, 5000, reportAxesNotPresent));
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A17) * micros());

  display.initDisplay();
  canInterface.canInit();
  voltageMonitor.initVoltageMonitor();
  radio.initRadio();
  //while(!radio.ok());

#ifdef PROFILE_LOOP
  tm.addBack(tm.newPeriodicTask(
    CheckTaskDuration,
    5000, // once per second
    [](TaskNode*, uint32_t) {
    Serial.print("CAN processing ");
    Serial.println(canProcessDuration);
    Serial.print("Task loop ");
    Serial.print (taskLoopDuration);
    uint32_t id = tm.getLongestTaskId();
    if (id != (uint32_t)-1) {
      Serial.print(" longest task ID:");
      Serial.print(id);
      Serial.print(" duration ");
      Serial.println(tm.getMaxTaskTime());
      tm.resetProfiler();
    }
    Serial.print("Radio processing ");
    Serial.println(radioProcessDuration);
    resetProcessProfiler();
    Serial.print("Yield duration ");
    Serial.println(yieldDuration);
    resetProcessProfiler();
  }));
#endif

  taskManager.addBack(taskManager.newPeriodicTask(
    DisplayUpdate,
    66, // 15 fps should be good enough for now
    [](TaskNode*, uint32_t) { display.updateScreen(); }));

  taskManager.addBack(taskManager.newPeriodicTask(
    RadioUpdate,
    10,
    [](TaskNode*, uint32_t now) { radio.poll10ms(now); }));

  startStateOne();
}

void runRobotControlCycle() {
#ifdef PROFILE_LOOP
  uint32_t startTime, duration;
#endif
  PROFILE_CALL(canInterface.readAndProcessCan(), canProcessDuration);
  PROFILE_CALL(taskManager.runNext(), taskLoopDuration);
  PROFILE_CALL(radio.poll(), radioProcessDuration);
}

void robotYield() {
  runRobotControlCycle();
}

void loop() {
#ifdef PROFILE_LOOP
  uint32_t startTime, duration;
#endif
  runRobotControlCycle();
  PROFILE_CALL(yield(), yieldDuration);
}
