/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include <type_traits>
#include <cstring>

template<
  typename T,   // storage integer type int32_t, uint32_t etc. must be large enough
  uint8_t sp,   // number of total bits including the sign
  uint8_t dp,   // number of bits for fractional value
  int bias = 0> // value range is centered around the bias value
class Fixed {
public:
  constexpr Fixed() = default;

  constexpr Fixed(float v) {
    static_assert(
        sp > dp, "Decimal bits must be less that total number of bits");
    static_assert(
        ((sizeof(T) * 8) >= sp), "The holding type must be large enough");

    if (v < minf()) {
        v = minf();
    } else if (v > maxf()) {
        v = maxf();
    }

    float bv = v - bias;

    if (std::is_signed<T>::value) {
      value_ = T(bv * (1 << dp) + ((bv >= 0) ? 0.5f : -0.5f));
    } else {
      value_ = T(bv * (1 << dp) + 0.5f);
    }
  }

  constexpr Fixed(double v) {
    static_assert(
        sp > dp, "Decimal bits must be less that total number of bits");
    static_assert(
        ((sizeof(T) * 8) >= sp), "The holding type must be large enough");

    if (v < min()) {
        v = min();
    } else if (v > max()) {
        v = max();
    }

    double bv = v - bias;

    if (std::is_signed<T>::value) {
      value_ = T(bv * (1 << dp) + ((bv >= 0) ? 0.5 : -0.5));
    } else {
      value_ = T(bv * (1 << dp) + 0.5);
    }
  }

  constexpr operator float() const {
    return (float(value_) / (1 << dp)) + bias;
  }

  constexpr operator double() const {
    return (double(value_) / (1 << dp)) + bias;
  }

  constexpr Fixed& operator = (const Fixed&_) = default;

  static constexpr uint8_t size() {
    return sizeof(value_);
  }

  const uint8_t* to_bytes() const {
    return reinterpret_cast<const uint8_t*>(&value_);
  }

  const T value() const {
    return value_;
  }

  static constexpr uint8_t bitSize() {
    return sp;
  }

  static constexpr Fixed maxValue() {
    if (std::is_signed<T>::value)
        return fromValue((1 << (sp - 1)) - 1);
    else
        return fromValue((1 << sp) - 1);
  }

  static constexpr float maxf() {
    return float(maxValue());
  }

  static constexpr double max() {
    return double(maxValue());
  }

  static constexpr Fixed minValue() {
    if (std::is_signed<T>::value)
        return fromValue(-(1 << (sp - 1)) + 1);
    else
        return fromValue(0);
  }

  static constexpr float minf() {
    return float(minValue());
  }

  static constexpr double min() {
    return double(minValue());
  }

  static constexpr Fixed fromBytes(const uint8_t* data, uint8_t len) {
    Fixed result;
    if (len == sizeof(result.value_)) {
        std::memcpy(&result.value_, data, sizeof(result.value_));
    }
    return result;
  }
private:
  static constexpr Fixed fromValue(T v) {
    Fixed r;
    r.value_ = v;
    return r;
  }

  T value_;
};
