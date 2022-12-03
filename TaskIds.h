/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

enum PeriodicTaskId {
  StateOneCheck,
  StateOneReport,
  StateTwo,
  StateThreeConnection,
  StateThreeODriveVoltage,
  StateThreeBatteryVoltage,
  StateThreeSerial,
};

enum SimpleTaskId {
  PrintPosition = 1000
};

