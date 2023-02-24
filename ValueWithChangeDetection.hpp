/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#pragma once

#include <cmath>
#include <limits>
#include <type_traits>

template<typename T>
class ValueWithChangeDetection {
  static constexpr float epsilon = 0.001f;

  template <bool isInteger> class Discrim {};

  bool isEqual(const T& other, Discrim<true>) const {
    return v_ == other;
  }

  bool isEqual(const T& other, Discrim<false>) const {
    return std::fabs(v_ - other) <= epsilon;
  }
public:
  ValueWithChangeDetection() : v_(), is_changed_(false) {}
  ValueWithChangeDetection(const T& value)
    : v_(value), is_changed_(false) {}

  operator T& () {
    return v_;
  }

  operator const T& () const {
    return v_;
  }

  ValueWithChangeDetection& operator = (const T& other) {
    is_changed_ =
      !isEqual(other, Discrim<std::numeric_limits<T>::is_integer || std::is_enum<T>::value>());
    if (is_changed_) {
      v_ = other;
    }
    return *this;
  }

  bool changed() const {
    return is_changed_;
  }

  void changeReset() {
    is_changed_ = false;
  }
private:
  T v_;
  bool is_changed_;
};
