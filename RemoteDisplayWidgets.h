/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include "RemoteDisplayWidget.h"
#include <stdint.h>
#include <ILI9341_t3n.h> // Hardware-specific library

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
  void getSize(uint16_t &w, uint16_t &h);
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
    if (active_ != value) {
      active_ = value;
      dirty_ = true;
    }
  }

  void init() override {
    centerLabel();
  };

  void draw() override;

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

class HBarWidget: public Widget {
public:
  HBarWidget(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool sgn = false)
    : Widget(x, y), w_(w), h_(h), sgn_(sgn) {
  }

  void setValue(float v) {
    if (v > 1.0f) {
      v = 1.0f;
    }
    if (sgn_) {
      if (v < -1.0f) {
        v = -1.0f;
      }
    } else {
      if (v < 0) {
        v = 0;
      }
    }
    if (fabsf(value_ - v) > 0.01) {
      value_ = v;
      dirty_ = true;
    }
  }

  void init() override {}
  void draw() override;
private:
  uint16_t w_;
  uint16_t h_;
  bool sgn_;
  float value_;
};

class VBarWidget: public Widget {
public:
  VBarWidget(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool sgn = false)
    : Widget(x, y), w_(w), h_(h), sgn_(sgn) {
  }

  void setValue(float v) {
    if (v > 1.0f) {
      v = 1.0f;
    }
    if (sgn_) {
      if (v < -1.0f) {
        v = -1.0f;
      }
    } else {
      if (v < 0) {
        v = 0;
      }
    }
    if (fabsf(value_ - v) > 0.01) {
      value_ = v;
      dirty_ = true;
    }
  }

  void init() override {}
  void draw() override;
private:
  uint16_t w_;
  uint16_t h_;
  bool sgn_;
  float value_;
};
