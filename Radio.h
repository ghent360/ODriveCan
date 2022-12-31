
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include  <limits>

class Radio {
public:
  typedef void (*Callback)(const uint8_t* data, uint8_t len);

  Radio();

  void initRadio();

  bool ok() const {
    return init_ok_;
  }

  void powerDown();

  // Call from the main app loop to handle radio data interrupts
  void poll();

  // Call periodically to check if the receiver has lost connection.
  void poll10ms(uint32_t timeNow);

  // Should not be smaller that the interval poll10ms is called.
  void setRxTimeout(uint32_t timeout) {
    rx_timeout_ms_ = timeout;
  }

  uint32_t lastRxTime() const {
    return last_received_ts_;
  }

  void setRxDataCallback(const Callback cb) {
    cb_ = cb;
  }

  bool writeTxData(const uint8_t* data, uint8_t len);
private:
  uint32_t init_ok_;
  uint32_t last_received_ts_;
  uint32_t rx_timeout_ms_;
  uint8_t rx_data_[32];
  Callback cb_;
};
