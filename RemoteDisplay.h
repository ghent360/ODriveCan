/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <math.h>
#include <stdint.h>
#include <ILI9341_t3n.h> // Hardware-specific library

class Widget {
public:
  Widget(uint16_t x, uint16_t y)
    : x_(x), y_(y), dirty_(true) {}
  virtual ~Widget() {}

  virtual void init() = 0;
  bool dirty() const { return dirty_; }
  virtual void draw() = 0;
  virtual void getSize(uint16_t &w, uint16_t &h) = 0;
protected:
  const uint16_t x_;
  const uint16_t y_;
  bool dirty_;
};

class BatteryWidget: public Widget {
public:
  static constexpr uint8_t batteryBarWidth = 50;
  static constexpr uint8_t batteryBarHeight = 16;

  BatteryWidget(
    const char* label, uint16_t x, uint16_t y, uint8_t numCells)
    : Widget(x, y), label_(label), num_cells_(numCells) {
  }

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
  StatusWidget(
    uint16_t x, uint16_t y, const ILI9341_t3_font_t& font, uint16_t color)
    : Widget(x, y), status_(), font_(font), color_(color) {}

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
  const ILI9341_t3_font_t& font_;
  uint16_t color_;
  uint16_t old_w_;
  uint16_t old_h_;
};

class ButtonStyle {
public:
  const ILI9341_t3_font_t* font_;
  uint16_t bg_normal_color_;
  uint16_t bg_active_color_;
  uint16_t text_normal_color_;
  uint16_t text_active_color_;
  uint16_t border_normal_color_;
  uint16_t border_active_color_;
};

class ButtonWidget: public Widget {
public:
  ButtonWidget(
    uint16_t x,
    uint16_t y,
    uint16_t w,
    uint16_t h,
    const char* label,
    const ButtonStyle& style)
    : Widget(x, y),
      active_(false),
      label_(label),
      style_(style),
      w_(w),
      h_(h) {
  }

  void setLabel(const char* value) {
    if (label_ != value) {
      label_ = value;
      centerLabel();
      dirty_ = true;
    }
  }
  
  void setLabel(const String& value) {
    if (label_ != value) {
      label_ = value;
      centerLabel();
      dirty_ = true;
    }
  }

  void setStyle(const ButtonStyle& style) {
    style_ = style;
    dirty_ = true;
  }

  void activate(bool value) {
    active_ = value;
    dirty_ = true;
  }

  void init() override {
    centerLabel();
  };

  void draw() override;

  void getSize(uint16_t &w, uint16_t &h) override {
    w = w_;
    h = h_;
  }

  bool isHit(uint16_t x, uint16_t y);
private:
  void centerLabel();

  bool active_;
  String label_;
  ButtonStyle style_;
  const uint16_t w_;
  const uint16_t h_;
  uint16_t label_x_offset_;
  uint16_t label_y_offset_;
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

  void setSW1Label(const char* v) {
    sw1_.setLabel(v);
  }
  void setSW2Label(const char* v) {
    sw2_.setLabel(v);
  }
  void setSW3Label(const char* v) {
    sw3_.setLabel(v);
  }
  void setSW4Label(const char* v) {
    sw4_.setLabel(v);
  }
  void setSW5Label(const char* v) {
    sw5_.setLabel(v);
  }
  void setSW1Active(bool v) {
    sw1_.activate(v);
  }
  void setSW2Active(bool v) {
    sw2_.activate(v);
  }
  void setSW3Active(bool v) {
    sw3_.activate(v);
  }
  void setSW4Active(bool v) {
    sw4_.activate(v);
  }
  void setSW5Active(bool v) {
    sw5_.activate(v);
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
  ButtonWidget sw1_;
  ButtonWidget sw2_;
  ButtonWidget sw3_;
  ButtonWidget sw4_;
  ButtonWidget sw5_;
  Widget* widgets_[9] = {
    &teensy_battery_,
    &bus1_battery_,
    &bus3_battery_,
    &radio_status_,
    &sw1_,
    &sw2_,
    &sw3_,
    &sw4_,
    &sw5_,
  };
};

extern RemoteDisplay remoteDisplay;
