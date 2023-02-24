/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once
#include <stdint.h>

class Widget {
public:
  Widget(uint16_t x, uint16_t y)
    : x_(x), y_(y), dirty_(true) {}
  virtual ~Widget() {}

  virtual void init() = 0;
  bool dirty() const { return dirty_; }
  virtual void draw() = 0;
protected:
  uint16_t x_;
  uint16_t y_;
  bool dirty_;
};
