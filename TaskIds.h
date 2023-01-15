/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

enum PeriodicTaskId {
  CheckTaskDuration,
  DisplayUpdate,
  RadioUpdate,
  StateOneCheck,
  StateOneReport,
  StateTwo,
  StateThreeConnection,
  StateThreeODriveVoltage,
  StateThreeBatteryVoltage,
  StateThreeSerial,
  RebotBodyRecalsLegPos,
};

enum SimpleTaskId {
  PrintPosition = 1000
};

