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

class BitstreamWriter {
public:
  BitstreamWriter(uint8_t* data, uint8_t len)
    : curr_(0),
      curr_bits_left_(32),
      data_(data),
      data_left_(len),
      error_(false) {}

  void flush() {
    store();
  }

  void writeBit(uint8_t v) {
    if (!curr_bits_left_) store();
    curr_ |= ((uint32_t)(v & 1) << (curr_bits_left_ - 1));
    curr_bits_left_--;
  }

  void writeBits(uint32_t v, uint8_t n) {
    if (!n || n > 32) return;
    v &= masks_[n];
    if (curr_bits_left_ >= n) {
      curr_ |= (v << (curr_bits_left_ - n));
      curr_bits_left_ -= n;
    } else {
      uint8_t rem = n - curr_bits_left_;
      curr_ |= (v >> rem);
      curr_bits_left_ = 0;
      store();
      v &= masks_[rem];
      curr_ |= (v << (curr_bits_left_ - rem));
      curr_bits_left_ -= rem;
    }
  }

  bool error() const {
    return error_;
  }
private:
  static constexpr uint32_t masks_[33] = {
    0,
    0x00000001,
    0x00000003,
    0x00000007,
    0x0000000F,
    0x0000001F,
    0x0000003F,
    0x0000007F,
    0x000000FF,
    0x000001FF,
    0x000003FF,
    0x000007FF,
    0x00000FFF,
    0x00001FFF,
    0x00003FFF,
    0x00007FFF,
    0x0000FFFF,
    0x0001FFFF,
    0x0003FFFF,
    0x0007FFFF,
    0x000FFFFF,
    0x001FFFFF,
    0x003FFFFF,
    0x007FFFFF,
    0x00FFFFFF,
    0x01FFFFFF,
    0x03FFFFFF,
    0x07FFFFFF,
    0x0FFFFFFF,
    0x1FFFFFFF,
    0x3FFFFFFF,
    0x7FFFFFFF,
    0xFFFFFFFF,
  };

  void store() {
    uint8_t old_bits_left = curr_bits_left_;
    curr_bits_left_ = 32;
    if (data_left_ >= 4) {
      data_[0] = curr_ >> 24;
      data_[1] = curr_ >> 16;
      data_[2] = curr_ >> 8;
      data_[3] = curr_;
      curr_ = 0;
      data_ += 4;
      data_left_ -= 4;
      return;
    }
    if (data_left_ == 3) {
      data_[0] = curr_ >> 24;
      data_[1] = curr_ >> 16;
      data_[2] = curr_ >> 8;
      curr_ = 0;
      data_ += 3;
      data_left_ = 0;
      if (old_bits_left < 8)
        error_ = true;
      return;
    }
    if (data_left_ == 2) {
      data_[0] = curr_ >> 24;
      data_[1] = curr_ >> 16;
      curr_ = 0;
      data_ += 2;
      data_left_ = 0;
      if (old_bits_left < 16)
        error_ = true;
      return;
    }
    if (data_left_ == 1) {
      data_[0] = curr_ >> 24;
      curr_ = 0;
      data_ += 1;
      data_left_ = 0;
      if (old_bits_left < 24)
        error_ = true;
      return;
    }
    curr_ = 0;
    error_ = true;
  }

  uint32_t curr_;
  uint8_t curr_bits_left_;
  uint8_t* data_;
  uint8_t data_left_;
  bool error_;
};
