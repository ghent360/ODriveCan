/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteDisplayWidgets.h"
#include <cmath>

#include "font_Inconsolata-Regular.h"

extern ILI9341_t3n tft;

// LiPO cell voltage levels
static constexpr float cellMaxVoltage = 4.2f;
static constexpr float cellWarnVoltage = 3.5f;
static constexpr float cellMinVoltage = 3.3f;

void BatteryWidget::init() {
  dirty_ = true;
  if (label_) {
    int16_t x1, y1;
    uint16_t labelW, labelH;
    tft.setFont(Inconsolata_Regular_12);
    tft.getTextBounds(label_, x_, y_, &x1, &y1, &labelW, &labelH);
    bar_x_offset_ = labelW + 2;
    w_ += bar_x_offset_;
    if (labelH > h_) {
      h_ = labelH;
      bar_y_offset_ = (h_ - batteryBarHeight) / 2;
    } else {
      label_y_offset_ = (h_ - labelH) / 2;
    }
  }
}

void BatteryWidget::draw() {
  if (!dirty_) return;

  // Clear the widget area
  tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);

  const float maxVoltage = cellMaxVoltage * num_cells_;
  const float warnVoltage = ((cellMaxVoltage + cellWarnVoltage) / 2) * num_cells_;
  const float lowVoltage = ((cellWarnVoltage + cellMinVoltage) / 2) * num_cells_;
  const float minVoltage = cellMinVoltage * num_cells_;
  uint16_t color;
  if (voltage_ > maxVoltage || voltage_ < lowVoltage) {
    color = ILI9341_RED;
  } else if (voltage_ < warnVoltage) {
    color = ILI9341_YELLOW;
  } else {
    color = ILI9341_GREEN;
  }

  float voltage = voltage_;
  if (voltage > maxVoltage) {
    voltage = maxVoltage;
  }

  if (label_) {
    tft.setTextColor(color);
    tft.setFont(Inconsolata_Regular_12);
    tft.drawString(label_, x_, y_ + label_y_offset_);
  }
  uint8_t x = x_ + bar_x_offset_;
  uint8_t y = y_ + bar_y_offset_;
  tft.drawRect(x, y, batteryBarWidth, batteryBarHeight, batteryBarColor);
  uint16_t barWidth = 
    (uint16_t)(
        (float)(batteryBarWidth - 2) * (voltage - minVoltage) / 
        (maxVoltage - minVoltage) + 
        0.5f);
  if (barWidth) {
    tft.fillRect(x + 1, y + 1, barWidth, batteryBarHeight - 2, color);
  }
  int16_t voltageInt = std::roundf(voltage * 100);
  String voltageStr = String(voltageInt / 100) + String(".") + String(voltageInt % 100);
  tft.drawString(voltageStr, x + batteryBarWidth + 2, y_ + label_y_offset_);
  int16_t x1, y1;
  uint16_t voltageW, voltageH;
  tft.getTextBounds(
    voltageStr,
    x + batteryBarWidth + 2,
    y_ + label_y_offset_,
    &x1,
    &y1,
    &voltageW,
    &voltageH);

  w_ = bar_x_offset_ + batteryBarWidth + 2 + voltageW;
  dirty_ = false;
}

void StatusWidget::draw() {
  if (!dirty_) return;
  // Clear the old widget area
  tft.fillRect(x_ - 1, y_ - 1, old_w_ + 2, old_h_ + 2, ILI9341_BLACK);

  if (status_.length() > 0) {
    tft.setTextColor(color_);
    tft.setFont(font_);
    tft.drawString(status_.c_str(), x_, y_);
  }
  dirty_ = false;
}

void StatusWidget::getSize(uint16_t &w, uint16_t &h) {
  if (status_.length() > 0) {
    int16_t x1, y1;
    tft.setFont(font_);
    tft.getTextBounds(status_.c_str(), x_, y_, &x1, &y1, &w, &h);
  } else {
    w = 0;
    h = 0;
  }
}

void ButtonWidget::draw() {
  if (!dirty_) return;
  tft.drawRect(
    x_, y_, w_, h_,
    active_ ? style_.border_active_color_ : style_.border_normal_color_);
  tft.fillRect(
    x_ + 1, y_ + 1, w_ - 2, h_ - 2,
    active_ ? style_.bg_active_color_ : style_.bg_normal_color_);
  if (label_.length() > 0) {
    tft.setFont(*style_.font_);
    tft.setTextColor(
      active_ ? style_.text_active_color_ : style_.text_normal_color_);
    tft.drawString(label_.c_str(), x_ + label_x_offset_ + 1, y_ + label_y_offset_ + 1);
  }
  dirty_ = false;
}

void ButtonWidget::centerLabel() {
  if (label_.length() > 0) {
    int16_t x1, y1;
    uint16_t width, height;
    tft.setFont(*style_.font_);
    tft.getTextBounds(label_.c_str(), x_, y_, &x1, &y1, &width, &height);
    label_x_offset_ = (w_ - width) / 2;
    label_y_offset_ = (h_ - height) / 2;
  } else {
    label_x_offset_ = 0;
    label_y_offset_ = 0;
  }
}

bool ButtonWidget::isHit(uint16_t x, uint16_t y) {
  return (x >= x_ && x <= (x_ + w_)) && (y >= y_ && y <= (y_ + h_));
}

void HBarWidget::draw() {
  if (!dirty_) return;

  tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);
  tft.drawRoundRect(x_, y_, w_, h_, h_ / 2, ILI9341_WHITE);
  if (sgn_) {
    if (value_ > 0) {
      uint16_t w = (uint16_t)std::roundf((w_ / 2 - 1) * value_);
      tft.fillRoundRect(
        x_ + w_ / 2,
        y_ + 1,
        w,
        h_ - 2,
        (h_ - 2) / 2,
        ILI9341_GREEN);
    } else if (value_ < 0) {
      int16_t w = -(int16_t)std::roundf((w_ / 2 - 1) * value_);
      tft.fillRoundRect(
        x_ + 1 + (w_ / 2 - w),
        y_ + 1,
        w,
        h_ - 2,
        (h_ - 2) / 2,
        ILI9341_GREEN);
    }
  } else {
    if (value_ > 0) {
      uint16_t w = (uint16_t)std::roundf((w_ - 2) * value_);
      tft.fillRoundRect(
        x_ + 1,
        y_ + 1,
        w,
        h_ - 2,
        (h_ - 2) / 2,
        ILI9341_GREEN);
    }
  }
  dirty_ = false;
}

void VBarWidget::draw() {
  if (!dirty_) return;

  tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);
  tft.drawRoundRect(x_, y_, w_, h_, w_ / 2, ILI9341_WHITE);
  if (sgn_) {
    if (value_ > 0) {
      uint16_t h = (uint16_t)std::roundf((h_ / 2 - 1) * value_);
      tft.fillRoundRect(
        x_ + 1,
        y_ + h_ / 2,
        w_ - 2,
        h,
        (w_ - 2) / 2,
        ILI9341_GREEN);
    } else if (value_ < 0) {
      int16_t h = -(int16_t)std::roundf((h_ / 2 - 1) * value_);
      tft.fillRoundRect(
        x_ + 1,
        y_ + 1 + (h_ / 2 - h),
        w_ - 2,
        h,
        (w_ - 2) / 2,
        ILI9341_GREEN);
    }
  } else {
    if (value_ > 0) {
      uint16_t h = (uint16_t)std::roundf((h_ - 2) * value_);
      tft.fillRoundRect(
        x_ + 1,
        y_ + 1,
        w_ - 2,
        h,
        (w_ - 2) / 2,
        ILI9341_GREEN);
    }
  }
  dirty_ = false;
}
