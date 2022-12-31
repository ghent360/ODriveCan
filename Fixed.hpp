/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include <type_traits>

template<
  typename T,
  uint8_t sp,
  uint8_t dp, 
  int bias = 0,
  bool is_signed = std::is_signed<T>::value>
class Fixed {
public:
  constexpr Fixed() = default;

  constexpr Fixed(float v) {
    static_assert(
        sp > dp, "Decimal bits must be less that total number of bits");
    static_assert(
        ((sizeof(T) * 8) >= sp), "The holding type must be large enough");

    if (v < min()) {
        v = min();
    } else if (v > max()) {
        v = max();
    }

    float bv = v - bias;

    if (is_signed) {
      value_ = T(bv * (1 << dp) + (bv >= 0) ? 0.5f : -0.5f);
    } else {
      if (bv < 0) {
        bv = 0;
      }
      value_ = T(bv * (1 << dp) + 0.5f);
    }
  }

  constexpr operator float() const {
    return (float(value_) / (1 << dp)) + bias;
  }

  uint8_t size() const {
    return sizeof(value_);
  }

  const uint8_t* to_tybes() const {
    return reinterpret_cast<uint8_t*>(&value_);
  }

  static constexpr Fixed maxValue() {
    if (is_signed)
        return fromValue((1 << (sp - 1)) - 1);
    else
        return fromValue((1 << sp) - 1);
  }

  static constexpr float max() {
    return float(maxValue());
  }

  static constexpr Fixed minValue() {
    if (is_signed)
        return fromValue(-(1 << (sp - 1)) + 1);
    else
        return fromValue(0);
  }

  static constexpr float min() {
    return float(minValue());
  }
private:
  static constexpr Fixed fromValue(T v) {
    Fixed r;
    r.value_ = v;
    return r;
  }

  T value_;
};
