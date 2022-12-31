/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>

class BitstreamReader {
public:
  BitstreamReader(const uint8_t* data, uint8_t len)
    : curr_(0),
      curr_bits_left_(0),
      data_(data),
      data_left_(len),
      error_(false) {}

    uint8_t readBit() {
      if (!curr_bits_left_) load();
      uint8_t result = (curr_ >> 31);
      curr_ <<= 1;
      curr_bits_left_--;
      return result;
    }

    uint32_t readBits(uint8_t n) {
      if (!n || n > 32) return 0;
      uint32_t result = curr_ >> (32 - n);
      if (n <= curr_bits_left_) {
        curr_bits_left_ -= n;
        curr_ <<= n;
        return result;
      } else {
        uint8_t rem = n - curr_bits_left_;
        load();
        result |= curr_ >> (32 - rem);
        if (rem == 32) {
          curr_ = 0;
        } else {
          curr_ <<= rem;
        }
        curr_bits_left_ -= rem;
      }
      return result;
    }

    bool error() const {
      return error_;
    }
private:
  void load() {
    curr_bits_left_ = 32;
    if (data_left_ >= 4) {
      curr_ = (data_[0] << 24) |
              (data_[1] << 16) |
              (data_[2] << 8) |
              data_[3];
      data_ += 4;
      data_left_ -= 4;
      return;
    }
    if (data_left_ == 3) {
      curr_ = (data_[0] << 24) |
              (data_[1] << 16) |
              (data_[2] << 8);
      data_ += 3;
      data_left_ = 0;
      return;
    }
    if (data_left_ == 2) {
      curr_ = (data_[0] << 24) |
              (data_[1] << 16);
      data_ += 2;
      data_left_ = 0;
      return;
    }
    if (data_left_ == 1) {
      curr_ = (data_[0] << 24);
      data_ += 1;
      data_left_ = 0;
      return;
    }
    curr_ = 0;
    error_ = true;
  }

  uint32_t curr_;
  uint8_t curr_bits_left_;
  const uint8_t* data_;
  uint8_t data_left_;
  bool error_;
};