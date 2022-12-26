
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include  <limits>
#include "slot_rf_protocol.h"

class Radio {
public:
  bool initRadio();
  void powerDown();
};

class RadioV2 {
public:
  static constexpr int kTimeoutMs = 1000;

  RadioV2(uint16_t ce_pin, uint16_t cs_pin, uint16_t irq_pin) 
    : rf_([&]() {
        nrf24::Nrf24l01::Pins pins;
        pins.cs = cs_pin;
        pins.irq = irq_pin;
        pins.ce = cs_pin;

        nrf24::SlotRfProtocol::Options options;
        options.ids[0] = id_;
        options.ptx = false;
        options.pins = pins;

        return options;
      }()) {
    rf_.Start();
  }

  void Poll() {
    rf_.Poll();
  }

  void PollMillisecond(uint32_t now) {
    auto old_timeout = remaining_timeout_ms_;
    if (old_timeout == 0) {
      // do nothing
    } else {
      if (old_timeout == 1) {
        disable();
      }
      remaining_timeout_ms_ = old_timeout - 1;
    }
    rf_.PollMillisecond(now);

    if (rf_.locked()) {
      lock_age_ms_ = 0;
    } else {
      if (lock_age_ms_ != std::numeric_limits<uint32_t>::max()) {
        lock_age_ms_++;
      }
    }
  }

  uint32_t readId() const {
    return id_;
  };

  void writeId(uint32_t id) {
    id_ = id;
  }

  void writeSlotData(
    uint8_t slot_no, uint32_t priority, uint8_t *data, uint8_t data_len) {
    nrf24::SlotRfProtocol::Slot slot;
    if (slot_no > nrf24::SlotRfProtocol::kNumSlots ||
        data_len > sizeof(slot.data)) return;
    slot.priority = priority;
    memcpy(slot.data, data, data_len);
    slot.size = data_len;
    rf_.remote()->tx_slot(slot_no, slot);
    remaining_timeout_ms_ = kTimeoutMs;
  }

  uint32_t bitfield() {
    return rf_.remote()->slot_bitfield();
  }

  uint32_t lock_age_ms() const {
    return lock_age_ms_;
  };

  void readSlotData(
    uint8_t slot_no, uint32_t &slot_age, uint8_t *data, uint8_t &data_len) {
    if (slot_no > nrf24::SlotRfProtocol::kNumSlots) return;
    const auto slot = rf_.remote()->rx_slot(slot_no);
    slot_age = slot.age;
    if (data_len < slot.size) {
      data_len = 0;
      return;
    }
    data_len = slot.size;
    memcpy(data, slot.data, slot.size);
  }
private:
  void disable() {
    for (int slot_index = 0;
         slot_index < nrf24::SlotRfProtocol::kNumSlots;
         slot_index++) {
      auto slot = rf_.remote()->tx_slot(slot_index);
      slot.priority = 0;
      rf_.remote()->tx_slot(slot_index, slot);
    }
  }

  nrf24::SlotRfProtocol rf_;
  uint32_t id_ = 0x3045;
  int32_t remaining_timeout_ms_ = 0;
  uint32_t lock_age_ms_ = 0;
  uint32_t last_bitfield_ = 0;
};
