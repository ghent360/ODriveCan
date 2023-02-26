/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

enum PeriodicTaskId {
  ReadStickValues,
  ReadSwitchValues,
  SendRemoteValues,
  DisplayUpdate,
  ReportProfileStats,
  RadioTestChannel,
  RadioAnnounceChannel,
};
