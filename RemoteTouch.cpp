/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteTouch.h"
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define TOUCH_CS   6
#define TOUCH_IRQ  5

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 345
#define TS_MINY 368
#define TS_MAXX 3736
#define TS_MAXY 3839

static XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

void RemoteTouch::initPins() {
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, 1);
}

void RemoteTouch::begin() {
  ts.begin(SPI1);
  ts.setRotation(2);
}

/*
 *  bool isTouched = ts.touched();
 *  TS_Point p = ts.getPoint();
 * 
 *  // Scale from ~0->4000 to tft.width using the calibration #'s
 *  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
 *  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
 */
RemoteTouch remoteTouch;
