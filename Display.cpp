/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/

#include "Display.h"
#include "globals.h"

#include <stdint.h>
#include <ST7735_t3.h> // Hardware-specific library
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

void Display::initDisplay() {
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, 1);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.useFrameBuffer(true);
  tft.setFont(Arial_8);
  dirty_ = true;
}

void Display::stopDisplay() {
  digitalWrite(TFT_LED, 0);
}

void Display::updateScreen() {
  if (dirty_) {
    tft.waitUpdateAsyncComplete();
    drawUi();
    tft.updateScreenAsync();
  }
}

constexpr uint8_t batteryBarWidth = 30;
constexpr uint8_t batteryBarHeight = 8;
constexpr uint16_t batteryBarColor = ST7735_WHITE;

static void drawBatteryVoltage(
    uint8_t x, uint8_t y,
    const char* label,
    float voltage,
    uint8_t nCells
    ) {
  const float maxVoltage = cellMaxVoltage * nCells;
  const float warnVoltage = cellWarnVoltage * nCells;
  const float lowVoltage = ((cellWarnVoltage + cellMinVoltage) / 2) * nCells;
  const float minVoltage = cellMinVoltage * nCells;
  uint16_t color;
  int16_t x1, y1;
  uint16_t labelW, labelH;
  uint8_t barHeight = batteryBarHeight;
  if (voltage > maxVoltage || voltage < lowVoltage) {
    color = ST7735_RED;
  } else if (voltage < warnVoltage) {
    color = ST7735_YELLOW;
  } else {
    color = ST7735_GREEN;
  }

  if (voltage > maxVoltage) {
    voltage = maxVoltage;
  }

  if (label) {
    tft.setTextColor(color);
    tft.setTextSize(6);
    tft.drawString(label, x, y);
    tft.getTextBounds(label, x, y, &x1, &y1, &labelW, &labelH);
    x += labelW + 2;
    if (labelH > barHeight) {
        barHeight = labelH;
    }
  }
  tft.drawFastHLine(x, y, batteryBarWidth, batteryBarColor);
  tft.drawFastHLine(x, y + barHeight - 1, batteryBarWidth, batteryBarColor);
  tft.drawFastVLine(x, y, barHeight, batteryBarColor);
  tft.drawFastVLine(x + batteryBarWidth - 1, y, barHeight, batteryBarColor);
  uint16_t barWidth = 
    (uint16_t)(
        (float)(batteryBarWidth - 2) * (voltage - minVoltage) / 
        (maxVoltage - minVoltage) + 
        0.5f);
  tft.fillRect(x + 1, y + 1, barWidth, barHeight - 2, color);
}

void Display::drawCanStatus() {
  if (can_status_) {
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(8);
    tft.drawString(can_status_, 0, 5 + 4*(batteryBarHeight + 1));
  }
}

void Display::drawUi() {
  tft.fillScreen(ST7735_BLACK);
  drawBatteryVoltage(0, 5, "M:", teensy_battery_voltage_, 2);
  drawBatteryVoltage(0, 5 + (batteryBarHeight + 1), "1: ", bus1_battery_voltage_, 6);
  drawBatteryVoltage(0, 5 + 2*(batteryBarHeight + 1), "2: ", bus3_battery_voltage_, 6);
  drawCanStatus();
  dirty_ = false;
}

Display display;
