/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteDisplay.h"
//#include "globals.h"

#include "font_Inconsolata-Regular.h"

// Screen pin definitions
#define TFT_CS    10
#define TFT_RESET  9
#define TFT_DC     8
#define TFT_LED    7

#define TOUCH_CS   6

static ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RESET);

// LiPO cell voltage levels
constexpr float cellMaxVoltage = 4.2f;
constexpr float cellWarnVoltage = 3.5f;
constexpr float cellMinVoltage = 3.3f;

RemoteDisplay::RemoteDisplay() 
  : teensy_battery_("M:", 10, 5, 2),
    bus1_battery_("1:", 10, 5 + (BatteryWidget::batteryBarHeight + 1), 6),
    bus3_battery_("2:", 10, 5 + 2*(BatteryWidget::batteryBarHeight + 1), 6),
    radio_status_(
      80 + BatteryWidget::batteryBarWidth, 8, Inconsolata_Regular_16, ILI9341_WHITE)
    {}

void RemoteDisplay::initPins() {
  pinMode(TFT_LED, OUTPUT);
  pinMode(TFT_RESET, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);

  digitalWriteFast(TFT_CS, 1);
  digitalWriteFast(TOUCH_CS, 1);
  digitalWriteFast(TFT_RESET, 0);
  delayNanoseconds(50);
  digitalWriteFast(TFT_RESET, 1);
  delayMicroseconds(400);
  digitalWriteFast(TFT_LED, 1);
}

void RemoteDisplay::begin() {
  tft.begin(110000000);
  tft.setRotation(1);
  tft.useFrameBuffer(true);
  tft.updateChangedAreasOnly(true);
  tft.setFont(Inconsolata_Regular_12);
  tft.fillScreen(ILI9341_BLACK);
  tft.updateScreen();
  for(auto widget : widgets_) {
    widget->init();
  }
}

void RemoteDisplay::stopDisplay() {
  digitalWriteFast(TFT_LED, 0);
}

void RemoteDisplay::updateScreen() {
  if (dirty()) {
    //tft.waitUpdateAsyncComplete();
    drawUi();
    tft.updateScreen();
  }
}

void RemoteDisplay::drawUi() {
  for(auto widget : widgets_) {
    widget->draw();
  }
}

bool RemoteDisplay::busy() {
  return tft.asyncUpdateActive();
}

BatteryWidget::BatteryWidget(
  const char* label, uint8_t x, uint8_t y, uint8_t numCells)
  : Widget(x, y), label_(label), num_cells_(numCells) {
}

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
  tft.drawFastHLine(x, y, batteryBarWidth, batteryBarColor);
  tft.drawFastHLine(x, y + batteryBarHeight - 1, batteryBarWidth, batteryBarColor);
  tft.drawFastVLine(x, y, batteryBarHeight, batteryBarColor);
  tft.drawFastVLine(x + batteryBarWidth - 1, y, batteryBarHeight, batteryBarColor);
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
  tft.fillRect(x_, y_, old_w_, old_h_, ILI9341_BLACK);

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
    uint16_t width, height;
    tft.setFont(font_);
    tft.getTextBounds(status_.c_str(), x_, y_, &x1, &y1, &width, &height);
    w = width;
    h = height;
  } else {
    w = 0;
    h = 0;
  }
}

RemoteDisplay remoteDisplay;
