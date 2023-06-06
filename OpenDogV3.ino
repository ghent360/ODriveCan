/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3.
 */
#include <Arduino.h>

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
#include "RadioController.h"

using odrive::EncoderError;
using odrive::EncoderEstimate;
using odrive::Heartbeat;
using odrive::IqValues;
using odrive::MotorError;
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
static void startStateOne();
static void startStateTwo();
static void startStateThree();

static void panic() {
  for(auto& axis: axes) {
    axis.EStop(); // Call EStop to reduce the axis power consumption.
  }
}

static void checkAxisConnection(TaskNode* self, uint32_t) {
  bool allAlive = true;
  for(auto& axis: axes) {
    axis.hb.PeriodicCheck(axis);
    if (!axis.hb.alive) {
      allAlive = false;
    }
    if (axis.hb.error != 0) {
      String status("Axis ");
      status += String(axis.node_id);
      status += " error: 0x";
      status += String(axis.hb.error, HEX);
      display.setCanStatus(status);
    }
  }
  if (!allAlive) {
    radioController.setReady(false);
    // Clean state three and switch back to first state:
    taskManager.removeById(RobotBodyStateExecutor);
    taskManager.removeById(RobotBodyRecalcLegPos);
    // Remove the checkSerialInput task.
    taskManager.removeById(StateThreeSerial);
    // Remove the voltage checking callbacks.
    for(auto& axis: axes) {
      axis.vbus.SetCallback(nullptr);
      axis.hb.SetCallback(nullptr);
      axis.mot_err.SetCallback(nullptr);
      axis.enc_err.SetCallback(nullptr);
      axis.iq.SetCallback(nullptr);
    }
    // Remove the checkAxisVbusVoltage task.
    taskManager.removeById(StateThreeODriveVoltage);
    taskManager.removeById(StateThreeAxisIq);
    taskManager.removeById(StateThreeReportStandingAccl);
    startStateOne();
    // Remove the checkAxisConnection task.
    taskManager.remove(self, true);
  }
}

static void startStateThree() {
  display.setCanStatus("Ready");
  display.setCanStatusColor(ST7735_GREEN);
  //TODO: remove the serial interaction stuff
  initSerialInteraction();
  robotBody.init();

  // Check if we lost CAN connection to any axis.
  taskManager.addBack(
    taskManager.newPeriodicTask(
      StateThreeConnection, 750, checkAxisConnection));

  // Configure various can message callbacks
  for(auto& axis: axes) {
    // When we receive battery voltage report from an axis
    // check the value, report it to the display and turn off
    // power if battery is too low.
    axis.vbus.SetCallback(
      [](ODriveAxis& axis, VbusVoltage&, VbusVoltage& newV) {
        if ((axis.node_id % 2) != 0) {
          display.setBus1BatteryVoltage(newV.val);
          radioController.setB1Voltage(newV.val);
        } else {
          display.setBus3BatteryVoltage(newV.val);
          radioController.setB2Voltage(newV.val);
        }

        if (newV.val < (6*cellMinVoltage)) {
          panic();
        }
      });

    // When we get heartbeat message, check for errors and report.
    axis.hb.SetCallback(
      [](ODriveAxis& axis, Heartbeat &old, Heartbeat &newVal) {
        // If the error value changed
        if (old.error != newVal.error) {
          radioController.reportAxisError(axis.node_id, newVal.error);
          // Request more error info if needed.
          if (newVal.error & Heartbeat::ERROR_MOTOR_FAILED) {
            axis.RequestMotorError();
          }
          if (newVal.error & Heartbeat::ERROR_ENCODER_FAILED) {
            axis.RequestEncoderError();
          }
        }
      });

    // When we get motor error message send it to the controller.
    axis.mot_err.SetCallback(
      [](ODriveAxis& axis, MotorError&, MotorError &newVal) {
        radioController.reportMotorError(axis.node_id, newVal.err);
      });

    // When we get encoder error message send it to the controller.
    axis.enc_err.SetCallback(
      [](ODriveAxis& axis, EncoderError&, EncoderError &newVal) {
        radioController.reportEncoderError(axis.node_id, newVal.err);
      });

    axis.iq.SetCallback(
      [](ODriveAxis& axis, IqValues&, IqValues& newVal) {
        radioController.reportAxisIq(axis.node_id, newVal.iqSetpoint);
      });
  }

  // Request battery voltage report periodically.
  // checkAxisVbusVoltage task
  taskManager.addBack(taskManager.newPeriodicTask(
    StateThreeODriveVoltage,
    1000,
    [](TaskNode*, uint32_t) {
      static uint8_t axisIdx = FRONT_RIGHT_HIP;
      axes[axisIdx].RequestVbusVoltage();
      axisIdx++;
      if (axisIdx > FRONT_LEFT_HIP) {
        axisIdx = FRONT_RIGHT_HIP;
      }
    }));

  radioController.setReady(true);
  //TODO: remove the serial interaction stuff
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
      // Lost axis connection, back to square one:
      startStateOne();
      taskManager.remove(self, true);
      return;
    }
    if (axis.hb.error != 0) {
      axis.ClearErrors();
      allClear = false;
    }
  }
  if (!allClear) {
    // We cleared some axis errors, wait and try again.
    return;
  }
  // All is good switch to state three.
  startStateThree();
  // Remove the clearErrorsAndSwitchToStateThree task.
  taskManager.remove(self, true);
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
    taskManager.removeById(StateOneReport);
    startStateTwo();
    // Remove the checkAllAxesArePresent task.
    taskManager.remove(self, true);
  }
}

static void reportAxesNotPresent(TaskNode* self, uint32_t) {
  String status("Axis:");
  for(auto& axis: axes) {
    if (!axis.hb.alive) {
      status += " ";
      status += String(axis.node_id);
    }
  }
  display.setCanStatus(status);
}

static void startStateOne() {
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

  // Refresh the display screen periodically
  taskManager.addBack(taskManager.newPeriodicTask(
    DisplayUpdate,
    66, // 15 fps should be good enough for now
    [](TaskNode*, uint32_t) { display.updateScreen(); }));

  // Do radio work periodically
  taskManager.addBack(taskManager.newPeriodicTask(
    RadioUpdate,
    10,
    [](TaskNode*, uint32_t now) { radio.poll10ms(now); }));

  // Periodically check the teensy battery and report.
  taskManager.addBack(taskManager.newPeriodicTask(
    RxBatteryVoltage,
    1000,
    [](TaskNode*, uint32_t) {
      float batVoltage = voltageMonitor.readBatteryVoltage();
      display.setTeensyBatteryVoltage(batVoltage);
      radioController.setRXVoltage(batVoltage);
      if (batVoltage < (2*cellMinVoltage)) {
        panic();
        delay(250);
        voltageMonitor.lowPowerMode();
      }
  }));

  // Send the radio data to the RadioController class
  radio.setRxDataCallback(
    [](const uint8_t* data, uint8_t len) {
      radioController.processRxData(data, len);
    });

#ifdef PROFILE_LOOP
  // Report performance stats periodically
  taskManager.addBack(taskManager.newPeriodicTask(
    ReportTaskDuration,
    5000, // once per 5 seconds
    [](TaskNode*, uint32_t) {
    Serial.print("CAN processing ");
    Serial.println(canProcessDuration);
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
    Serial.print("Radio processing ");
    Serial.println(radioProcessDuration);
    Serial.print("Yield duration ");
    Serial.println(yieldDuration);
    resetProcessProfiler();
  }));
#endif

  // Start the ODrive connection state machine.
  startStateOne();
}

void loop() {
#ifdef PROFILE_LOOP
  uint32_t startTime, duration;
#endif
  PROFILE_CALL(canInterface.readAndProcessCan(), canProcessDuration);
  PROFILE_CALL(taskManager.runNext(), taskLoopDuration);
  PROFILE_CALL(radio.poll(), radioProcessDuration);
  PROFILE_CALL(yield(), yieldDuration);
}
