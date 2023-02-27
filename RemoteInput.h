/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#pragma once

#include "Swithes.h"
#include "ValueWithChangeDetection.hpp"
#include <stdint.h>

class RemoteInputs {
public:
  void initPins();
  void begin();
  void readStickValues();
  void readSwitchValues();

  // Joysticks
  int16_t getX1() const { return x1_; }
  int16_t getY1() const { return y1_; }
  int16_t getZ1() const { return z1_; }
  int16_t getX2() const { return x2_; }
  int16_t getY2() const { return y2_; }
  int16_t getZ2() const { return z2_; }

  // Switches
  SW3POS getSW1() const { return sw1_; }
  SW3POS getSW2() const { return sw2_; }
  SW3POS getSW3() const { return sw3_; }
  SW3POS getSW4() const { return sw4_; }
  SW2POS getSW5() const { return sw5_; }

  bool getSW1changed() const { return sw1_.changed(); }
  bool getSW2changed() const { return sw1_.changed(); }
  bool getSW3changed() const { return sw1_.changed(); }
  bool getSW4changed() const { return sw1_.changed(); }
  bool getSW5changed() const { return sw1_.changed(); }

  void SW1reset() { sw1_.changeReset(); }
  void SW2reset() { sw1_.changeReset(); }
  void SW3reset() { sw1_.changeReset(); }
  void SW4reset() { sw1_.changeReset(); }
  void SW5reset() { sw1_.changeReset(); }

  // Buttons
  SW2POS getB1() const { return b1_; }
  SW2POS getB2() const { return b2_; }
  SW2POS getB3() const { return b3_; }
  SW2POS getB4() const { return b4_; }

  bool getB1changed() const { return b1_.changed(); }
  bool getB2changed() const { return b1_.changed(); }
  bool getB3changed() const { return b1_.changed(); }
  bool getB4changed() const { return b1_.changed(); }

  bool B1clicked() const { return b1_ == SW2_OFF && b1_.changed(); }
  bool B2clicked() const { return b2_ == SW2_OFF && b2_.changed(); }
  bool B3clicked() const { return b3_ == SW2_OFF && b3_.changed(); }
  bool B4clicked() const { return b4_ == SW2_OFF && b4_.changed(); }

  void B1reset() { b1_.changeReset(); }
  void B2reset() { b2_.changeReset(); }
  void B3reset() { b3_.changeReset(); }
  void B4reset() { b4_.changeReset(); }

  void setB1Led(bool value) const;
  void setB2Led(bool value) const;
  void setB3Led(bool value) const;
  void setB4Led(bool value) const;
private:
  SW3POS sw3ValueFromAdc(int16_t) const;

  static constexpr int16_t x1MidPoint = 2026;
  static constexpr int16_t y1MidPoint = 1982;
  static constexpr int16_t z1MidPoint = 2055;
  static constexpr int16_t x2MidPoint = 2013;
  static constexpr int16_t y2MidPoint = 2082;
  static constexpr int16_t z2MidPoint = 2022;
  static constexpr int16_t swOnLowPoint = 2000;
  static constexpr int16_t swOnHighPoint = 2100;
  static constexpr int16_t swOffPoint = 10;
  int16_t x1_;
  int16_t y1_;
  int16_t z1_;
  int16_t x2_;
  int16_t y2_;
  int16_t z2_;
  ValueWithChangeDetection<SW3POS> sw1_;
  ValueWithChangeDetection<SW3POS> sw2_;
  ValueWithChangeDetection<SW3POS> sw3_;
  ValueWithChangeDetection<SW3POS> sw4_;
  ValueWithChangeDetection<SW2POS> sw5_;
  ValueWithChangeDetection<SW2POS> b1_;
  ValueWithChangeDetection<SW2POS> b2_;
  ValueWithChangeDetection<SW2POS> b3_;
  ValueWithChangeDetection<SW2POS> b4_;
};

extern RemoteInputs remoteInputs;