/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteInput.h"

#include <ADC.h>
#include <ADC_util.h>

// Pin assignment from the schematics
static constexpr int16_t readPinX1 = A0;
static constexpr int16_t readPinY1 = A1;
static constexpr int16_t readPinZ1 = A2;
static constexpr int16_t readPinX2 = A3;
static constexpr int16_t readPinY2 = A4;
static constexpr int16_t readPinZ2 = A5;
static constexpr int16_t readPinSW1 = A6;
static constexpr int16_t readPinSW2 = A7;
static constexpr int16_t readPinSW3 = A8;
static constexpr int16_t readPinSW4 = A9;
static constexpr int16_t readPinSW5 = 1;
static constexpr int16_t pinB1Led = 25;
static constexpr int16_t pinB1 = 24;
static constexpr int16_t pinB2Led = 27;
static constexpr int16_t pinB2 = 26;
static constexpr int16_t pinB3Led = 29;
static constexpr int16_t pinB3 = 28;
static constexpr int16_t pinB4Led = 31;
static constexpr int16_t pinB4 = 30;

static ADC adc;

void RemoteInputs::initPins() {
  pinMode(readPinX1, INPUT_DISABLE);
  pinMode(readPinY1, INPUT_DISABLE);
  pinMode(readPinZ1, INPUT_DISABLE);
  pinMode(readPinX2, INPUT_DISABLE);
  pinMode(readPinY2, INPUT_DISABLE);
  pinMode(readPinZ2, INPUT_DISABLE);
  pinMode(readPinSW1, INPUT_DISABLE);
  pinMode(readPinSW2, INPUT_DISABLE);
  pinMode(readPinSW3, INPUT_DISABLE);
  pinMode(readPinSW4, INPUT_DISABLE);
  pinMode(readPinSW5, INPUT_DISABLE);
  pinMode(pinB1, INPUT);
  pinMode(pinB2, INPUT);
  pinMode(pinB3, INPUT);
  pinMode(pinB4, INPUT);
  pinMode(pinB1Led, OUTPUT);
  pinMode(pinB2Led, OUTPUT);
  pinMode(pinB3Led, OUTPUT);
  pinMode(pinB4Led, OUTPUT);
}

void RemoteInputs::begin() {
  adc.adc0->setAveraging(8);
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc.adc0->disableCompare();

  //adc.adc0->enableInterrupts(adc0_isr);
  //adcMaxValue = adc.adc0->getMaxValue();
}

SW3POS RemoteInputs::sw3ValueFromAdc(int16_t value) const {
  if (value <= swOffPoint) return SW3_OFF;
  if (value >= swOnLowPoint && value <= swOnHighPoint) return SW3_ON;
  return SW3_MID;
}

void RemoteInputs::readValues() {
  int16_t valueX1 = adc.adc0->analogRead(readPinX1);
  int16_t valueY1 = adc.adc0->analogRead(readPinY1);
  int16_t valueZ1 = adc.adc0->analogRead(readPinZ1);
  x1_ = valueX1 - x1MidPoint;
  y1_ = valueY1 - y1MidPoint;
  z1_ = valueZ1 - z1MidPoint;

  int16_t valueX2 = 4083 - adc.adc0->analogRead(readPinX2);
  int16_t valueY2 = 4086 - adc.adc0->analogRead(readPinY2);
  int16_t valueZ2 = adc.adc0->analogRead(readPinZ2);
  x2_ = valueX2 - x2MidPoint;
  y2_ = valueY2 - y2MidPoint;
  z2_ = valueZ2 - z2MidPoint;

  int16_t valueSW1 = adc.adc0->analogRead(readPinSW1);
  int16_t valueSW2 = adc.adc0->analogRead(readPinSW2);
  int16_t valueSW3 = adc.adc0->analogRead(readPinSW3);
  int16_t valueSW4 = adc.adc0->analogRead(readPinSW4);
  sw1_ = sw3ValueFromAdc(valueSW1);
  sw2_ = sw3ValueFromAdc(valueSW2);
  sw3_ = sw3ValueFromAdc(valueSW3);
  sw4_ = sw3ValueFromAdc(valueSW4);

  uint8_t sw5 = digitalReadFast(readPinSW5);
  sw5_ = sw5 ? SW2_ON : SW2_OFF;

  uint8_t b1 = digitalReadFast(pinB1);
  uint8_t b2 = digitalReadFast(pinB2);
  uint8_t b3 = digitalReadFast(pinB3);
  uint8_t b4 = digitalReadFast(pinB4);
  b1_ = b1 ? SW2_OFF : SW2_ON;
  b2_ = b2 ? SW2_OFF : SW2_ON;
  b3_ = b3 ? SW2_OFF : SW2_ON;
  b4_ = b4 ? SW2_OFF : SW2_ON;
}

void RemoteInputs::setB1Led(bool value) const {
  digitalWriteFast(pinB1Led, value);
}

void RemoteInputs::setB2Led(bool value) const {
  digitalWriteFast(pinB2Led, value);
}

void RemoteInputs::setB3Led(bool value) const {
  digitalWriteFast(pinB3Led, value);
}

void RemoteInputs::setB4Led(bool value) const {
  digitalWriteFast(pinB4Led, value);
}

RemoteInputs remoteInputs;