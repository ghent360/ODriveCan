/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
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
      0, 5 + 4*(BatteryWidget::batteryBarHeight + 1), 8, ST7735_WHITE),
    radio_status_(
      50 + BatteryWidget::batteryBarWidth, 5, 8, ST7735_WHITE)
#ifdef AXIS_POS_DISPLAY
    ,joint_pos_{
      PositionWidget(  0, 55, 6, ST7735_WHITE), //FRONT_RIGHT_SHIN
      PositionWidget( 40, 55, 6, ST7735_WHITE), //FRONT_LEFT_SHIN
      PositionWidget( 80, 55, 6, ST7735_WHITE), //BACK_RIGHT_SHIN
      PositionWidget(120, 55, 6, ST7735_WHITE), //BACK_LEFT_SHIN
      PositionWidget(  0, 65, 6, ST7735_WHITE), //FRONT_RIGHT_TIE
      PositionWidget( 40, 65, 6, ST7735_WHITE), //FRONT_LEFT_TIE
      PositionWidget( 80, 65, 6, ST7735_WHITE), //BACK_RIGHT_TIE
      PositionWidget(120, 65, 6, ST7735_WHITE), //BACK_LEFT_TIE
      PositionWidget(  0, 75, 6, ST7735_WHITE), //FRONT_RIGHT_HIP
      PositionWidget( 40, 75, 6, ST7735_WHITE), //FRONT_LEFT_HIP
      PositionWidget( 80, 75, 6, ST7735_WHITE), //BACK_RIGHT_HIP
      PositionWidget(120, 75, 6, ST7735_WHITE)  //BACK_LEFT_HIP
    }
#endif
    {}

void Display::initDisplay() {
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, 1);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.useFrameBuffer(true);
  tft.setFont(Arial_8);
  tft.fillScreen(ST7735_BLACK);
  tft.updateScreenAsync();
  for(auto widget : widgets_) {
    widget->init();
  }
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

#ifdef AXIS_POS_DISPLAY
void Display::setJoinColor(uint8_t axisId, uint16_t color) {
  DogLegJoint joint = getJointByAxisId(axisId);
  if (joint < numAxes) {
    joint_pos_[joint].setColor(color);
  }
}

void Display::setJoinPos(uint8_t axisId, float pos) {
  DogLegJoint joint = getJointByAxisId(axisId);
  if (joint < numAxes) {
    joint_pos_[joint].setPos(pos - jointOffsets[joint]);
  }
}
#endif

BatteryWidget::BatteryWidget(
  const char* label, uint8_t x, uint8_t y, uint8_t numCells)
  : Widget(x, y), label_(label), num_cells_(numCells) {
}

void BatteryWidget::init() {
  dirty_ = true;
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
  tft.fillRect(x_, y_, old_w_, old_h_, ST7735_BLACK);

  if (status_.length() > 0) {
    tft.setTextColor(color_);
    tft.setTextSize(font_size_);
    tft.drawString(status_.c_str(), x_, y_);
  }
  dirty_ = false;
}

void StatusWidget::getSize(uint16_t &w, uint16_t &h) {
  if (status_.length() > 0) {
    int16_t x1, y1;
    uint16_t width, height;
    tft.setTextSize(font_size_);
    tft.getTextBounds(status_.c_str(), x_, y_, &x1, &y1, &width, &height);
    w = width;
    h = height;
  } else {
    w = 0;
    h = 0;
  }
}

void PositionWidget::draw() {
  if (!dirty_) return;
  // Clear the old widget area
  tft.fillRect(x_, y_, old_w_, old_h_, ST7735_BLACK);

  if (posStr_.length() > 0) {
    tft.setTextColor(color_);
    tft.setTextSize(font_size_);
    tft.drawString(posStr_.c_str(), x_, y_);
  }
  dirty_ = false;
}

void PositionWidget::getSize(uint16_t &w, uint16_t &h) {
  if (posStr_.length() > 0) {
    int16_t x1, y1;
    uint16_t width, height;
    tft.setTextSize(font_size_);
    tft.getTextBounds(posStr_.c_str(), x_, y_, &x1, &y1, &width, &height);
    w = width;
    h = height;
  } else {
    w = 0;
    h = 0;
  }
}

void PositionWidget::convert() {
  if (pos_ != 0) {
    posStr_ = String(pos_, 3);
  } else {
    posStr_ = String();
  }
}

Display display;
