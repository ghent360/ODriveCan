/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

enum PeriodicTaskId {
  StateOneCheck,
  StateOneReport,
  StateTwo,
  StateThreeConnection,
  StateThreeVoltage,
  StateThreeSerial,
};

enum SimpleTaskId {
  PrintPosition = 1000
};

