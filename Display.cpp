/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/

#include "Display.h"
#include "globals.h"

#include <st7735_t3_font_Arial.h>

// Screen pin definitions
#define TFT_MISO  39
#define TFT_MOSI  26
#define TFT_SCK   27
#define TFT_DC    34
#define TFT_CS    38
#define TFT_RST   36
#define TFT_LED   40

static ST7735_t3 tft = ST7735_t3(
  TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

Display::Display() 
  : teensy_battery_("M:", 0, 5, 2),
    bus1_battery_("1: ", 0, 5 + (BatteryWidget::batteryBarHeight + 1), 6),
    bus3_battery_("2: ", 0, 5 + 2*(BatteryWidget::batteryBarHeight + 1), 6),
    can_status_(
      0, 5 + 4*(BatteryWidget::batteryBarHeight + 1), 8, ST7735_WHITE) {
}

void Display::initDisplay() {
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, 1);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.useFrameBuffer(true);
  tft.setFont(Arial_8);
}

void Display::stopDisplay() {
  digitalWrite(TFT_LED, 0);
}

void Display::updateScreen() {
  if (dirty()) {
    tft.waitUpdateAsyncComplete();
    drawUi();
    tft.updateScreenAsync();
  }
}

void Display::drawUi() {
  for(auto widget : widgets_) {
    widget->draw();
  }
}

BatteryWidget::BatteryWidget(
  const char* label, uint8_t x, uint8_t y, uint8_t numCells)
  : Widget(x, y), label_(label), num_cells_(numCells) {
  if (label_) {
    int16_t x1, y1;
    uint16_t labelW, labelH;
    tft.setTextSize(6);
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
  tft.fillRect(x_, y_, w_, h_, ST7735_BLACK);

  const float maxVoltage = cellMaxVoltage * num_cells_;
  const float warnVoltage = ((cellMaxVoltage + cellWarnVoltage) / 2) * num_cells_;
  const float lowVoltage = ((cellWarnVoltage + cellMinVoltage) / 2) * num_cells_;
  const float minVoltage = cellMinVoltage * num_cells_;
  uint16_t color;
  if (voltage_ > maxVoltage || voltage_ < lowVoltage) {
    color = ST7735_RED;
  } else if (voltage_ < warnVoltage) {
    color = ST7735_YELLOW;
  } else {
    color = ST7735_GREEN;
  }

  float voltage = voltage_;
  if (voltage > maxVoltage) {
    voltage = maxVoltage;
  }

  if (label_) {
    tft.setTextColor(color);
    tft.setTextSize(6);
    tft.drawString(label_, x_, label_y_offset_);
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
  tft.fillRect(x + 1, y + 1, barWidth, batteryBarHeight - 2, color);
  dirty_ = false;
}

void StatusWidget::draw() {
  if (!dirty_) return;
  // Clear the old widget area
  tft.fillRect(x_, y_, old_w_, old_h_, ST7735_BLACK);

  if (status_) {
    tft.setTextColor(color_);
    tft.setTextSize(font_size_);
    tft.drawString(status_, x_, y_);
  }
  dirty_ = false;
}

void StatusWidget::getSize(uint16_t &w, uint16_t &h) {
  int16_t x1, y1;
  tft.setTextSize(font_size_);
  tft.getTextBounds(status_, x_, y_, &x1, &y1, &w, &h);
}

Display display;
