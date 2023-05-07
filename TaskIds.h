/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

enum PeriodicTaskId {
  ReportTaskDuration,
  DisplayUpdate,
  RadioUpdate,
  StateOneCheck,
  StateOneReport,
  StateTwo,
  StateThreeConnection,
  StateThreeODriveVoltage,
  //StateThreeAxisIq,
  //StateThreeReportStandingAccl,
  RxBatteryVoltage,
  StateThreeSerial,
  RobotBodyRecalcLegPos,
  RobotBodyStateExecutor,
};

enum SimpleTaskId {
  PrintPosition = 1000
};

