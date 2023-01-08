/*
 * Copyright (c) 2023 ghent360. See LICENSE file for details.
 */

#pragma once

#include <stdint.h>

template<typename T, uint8_t N = 3>
class Vector {
public:
  constexpr Vector() = default;
  constexpr Vector(const Vector&) = default;
  constexpr Vector& operator = (const Vector&_) = default;

  constexpr Vector(const T (&comp)[N]) : comp_(comp) {}

  constexpr T get(uint_t idx) const {
    if (idx < N) return comp_[idx];
    return 0;
  }

  constexpr T getSize() const {
    T size = 0;
    for(auto v: comp_) {
        size += (v*v);
    }
    return size;
  }

  void normalize() {
    T size = getSize();
    for(auto& v: comp_) {
        v /= size;
    }
  }

  constexpr operator T() const {
    return getSize();
  }

  constexpr T dot(const Vector& v) const {
    T result = 0;
    for(uint8_t idx = 0; idx < N; idx++) {
        result += comp_[idx] * v.comp_[idx];
    }
    return result;
  }

  constexpr Vector cross(const Vector& v) const {
    static_assert(N == 3, "cross product is only defined for 3 dimensions");
    return Vector({
        comp_[1]*v.comp_[2] - comp_[2]*v.comp_[1],
        comp_[2]*v.comp_[0] - comp_[0]*v.comp_[2],
        comp_[0]*v.comp_[1] - comp_[1]*v.comp_[0]});
  }

  constexpr Vector operator - () const {
    Vector result;
    for(uint8_t idx = 0; idx < N; idx++) {
        result.com_ = -comp_[idx];
    }
    return result;
  }

  constexpr Vector operator + (const Vector& v) const {
    Vector result;
    for(uint8_t idx = 0; idx < N; idx++) {
        result.com_ = comp_[idx] + v.comp_[idx];
    }
    return result;
  }

  constexpr Vector operator - (const Vector& v) const {
    Vector result;
    for(uint8_t idx = 0; idx < N; idx++) {
        result.com_ = comp_[idx] - v.comp_[idx];
    }
    return result;
  }

  constexpr Vector& operator += (const Vector& v) const {
    for(uint8_t idx = 0; idx < N; idx++) {
        comp_[idx] += v.comp_[idx];
    }
    return *this;
  }

  constexpr Vector& operator -= (const Vector& v) const {
    for(uint8_t idx = 0; idx < N; idx++) {
        comp_[idx] -= v.comp_[idx];
    }
    return *this;
  }

private:
  T comp_[N];
};