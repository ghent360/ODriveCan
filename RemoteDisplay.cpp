/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteDisplay.h"

#include "font_Inconsolata-Regular.h"
#include "font_JetBrainsMono-Regular.h"
#include "font_FiraCode-Retina.h"
#include "font_FiraCode-Regular.h"
#include "font_Anonymous_Pro.h"
#include "font_Anonymous_Pro_Minus.h"

// Screen pin definitions
#define TFT_CS    10
#define TFT_RESET  9
#define TFT_DC     8
#define TFT_LED    7

ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RESET);

static const ButtonStyle stdButton = {
  .font_ = &Inconsolata_Regular_10,
  .bg_normal_color_ = ILI9341_BLACK,
  .bg_active_color_ = ILI9341_YELLOW,
  .text_normal_color_ = ILI9341_YELLOW,
  .text_active_color_ = ILI9341_BLACK,
  .border_normal_color_ = ILI9341_WHITE,
  .border_active_color_ = ILI9341_YELLOW,
};

RemoteDisplay::RemoteDisplay() 
  : teensy_battery_("M:", 10, 5, 2),
    bus1_battery_("1:", 10, 5 + (BatteryWidget::batteryBarHeight + 1), 6),
    bus3_battery_("2:", 10, 5 + 2 * (BatteryWidget::batteryBarHeight + 1), 6),
    radio_status_(
      80 + BatteryWidget::batteryBarWidth, 8, FiraCodeRetina_14, ILI9341_WHITE),
    sw1_(5, 210, 60, 22, "", stdButton),
    sw2_(67, 210, 60, 22, "", stdButton),
    sw3_(129, 210, 60, 22, "", stdButton),
    sw4_(191, 210, 60, 22, "", stdButton),
    sw5_(253, 210, 60, 22, "", stdButton),
#if 0
    x1_(10, 100, 50, 10, true),
    y1_(30, 80, 10, 50, true),
#endif
    menu_(265, 5, 50, 20, "Menu", stdButton),
    menuController_() {}

void RemoteDisplay::initPins() {
  pinMode(TFT_LED, OUTPUT);
  pinMode(TFT_RESET, OUTPUT);
  pinMode(TFT_CS, OUTPUT);

  digitalWriteFast(TFT_CS, 1);
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
  tft.setFont(FiraCodeRetina_14);
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

RemoteDisplay remoteDisplay;
