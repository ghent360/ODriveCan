/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <math.h>
#include <stdint.h>
#include <ILI9341_t3n.h> // Hardware-specific library

class Widget {
public:
  Widget(uint8_t x, uint8_t y)
    : x_(x), y_(y), dirty_(false) {}
  virtual ~Widget() {}

  virtual void init() = 0;
  bool dirty() const { return dirty_; }
  virtual void draw() = 0;
  virtual void getSize(uint16_t &w, uint16_t &h) = 0;
protected:
  const uint8_t x_;
  const uint8_t y_;
  bool dirty_;
};

class BatteryWidget: public Widget {
public:
  static constexpr uint8_t batteryBarWidth = 30;
  static constexpr uint8_t batteryBarHeight = 8;

  BatteryWidget(const char* label, uint8_t x, uint8_t y, uint8_t numCells);

  void setVoltage(float v) {
    if (fabsf(v - voltage_) > 0.01f) {
      voltage_ = v;
      dirty_ = true;
    }
  }

  void init() override;
  void draw() override;
  void getSize(uint16_t &w, uint16_t &h) override {
    w = w_;
    h = h_;
  }
private:
  static constexpr uint16_t batteryBarColor = ILI9341_WHITE;

  uint16_t w_ = batteryBarWidth;
  uint16_t h_ = batteryBarHeight;
  const char *label_;
  const uint8_t num_cells_;
  float voltage_;
  uint8_t label_y_offset_ = 0;
  uint8_t bar_y_offset_ = 0;
  uint8_t bar_x_offset_ = 0;
};

class StatusWidget: public Widget {
public:
  StatusWidget(uint8_t x, uint8_t y, uint8_t fontSize, uint16_t color)
    : Widget(x, y), status_(), font_size_(fontSize), color_(color) {}

  void setStatus(const char* status) {
    if (status_ != status) {
      if (!dirty_) {
        getSize(old_w_, old_h_);
      }
      status_ = status;
      dirty_ = true;
    }
  }
  
  void setStatus(const String& status) {
    if (status_ != status) {
      if (!dirty_) {
        getSize(old_w_, old_h_);
      }
      status_ = status;
      dirty_ = true;
    }
  }

  void setColor(uint16_t color) {
    if (color_ != color) {
      if (!dirty_) {
        getSize(old_w_, old_h_);
      }
      color_ = color;
      dirty_ = true;
    }
  }

  void init() override {};
  void draw() override;
  void getSize(uint16_t &w, uint16_t &h) override;
private:
  String status_;
  const uint8_t font_size_;
  uint16_t color_;
  uint16_t old_w_;
  uint16_t old_h_;
};

class RemoteDisplay {
public:
  RemoteDisplay();

  void initPins();
  void begin();
  void stopDisplay();
  void updateScreen();
  bool busy();

  void setTeensyBatteryVoltage(float v) {
    teensy_battery_.setVoltage(v);
  }

  void setBus1BatteryVoltage(float v) {
    bus1_battery_.setVoltage(v);
  }

  void setBus3BatteryVoltage(float v) {
    bus3_battery_.setVoltage(v);
  }

  void setRadioStatus(const char* msg) {
    radio_status_.setStatus(msg);
  }

  void setRadioStatus(const String& msg) {
    radio_status_.setStatus(msg);
  }

  void setRadioStatusColor(uint16_t color) {
    radio_status_.setColor(color);
  }

private:
  void drawUi();
  bool dirty() const {
    for(auto widget : widgets_) {
      if (widget->dirty()) return true;
    }
    return false;
  }

  BatteryWidget teensy_battery_;
  BatteryWidget bus1_battery_;
  BatteryWidget bus3_battery_;
  StatusWidget radio_status_;
  Widget* widgets_[4] = {
    &teensy_battery_,
    &bus1_battery_,
    &bus3_battery_,
    &radio_status_,
  };
};

extern RemoteDisplay remoteDisplay;
