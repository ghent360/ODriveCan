/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <Arduino.h>
#include "TaskManager.hpp"
#include <stdint.h>

class RemoteRadio {
public:
  void initPins();
  void begin();

  bool txData(const uint8_t *data, uint8_t len);
  void poll();
private:
  static constexpr uint8_t testRetries = 5;
  static constexpr uint8_t testPktRetries = 2;
  static constexpr uint8_t dataPktRetries = 12;
  static constexpr uint8_t announcePktRetries = 5;
  static constexpr uint8_t dataPktMinArc = 8; // Must be less than dataPktRetries
  static constexpr uint8_t announceRetries = 3;

  void testChannelCB(TaskNode*);
  void announceChannelCB(TaskNode*);
  void startConnection();
  void startAnnounceNewChannel();

  uint8_t newChannelNo() {
    uint8_t nextCh = channel_ + 2;
    if (nextCh > 125) nextCh = 0;
      return nextCh;
  }

  uint8_t channel_;
  bool    channelConnected_;
  uint8_t testChannelTriesRemaining_;
  uint8_t announceChannelTriesRemaining_;
  uint8_t ackData_[32];
};

extern RemoteRadio remoteRadio;
