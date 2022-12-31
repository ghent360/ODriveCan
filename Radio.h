
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include  <limits>

class Radio {
public:
  void initRadio();

  bool ok() const {
    return init_ok_;
  }

  void powerDown();
  void poll();
  void poll10ms(uint32_t timeNow);

  uint32_t lastRxTime() const {
    return last_received_ts_;
  }
private:
  uint32_t init_ok_;
  uint32_t last_received_ts_;
  uint32_t rx_timeout_ms_;
  uint8_t rx_data_[32];
};
