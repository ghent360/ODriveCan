
/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once

#include <stdint.h>
#include  <limits>

#define NRF24_CE_PIN  4
#define NRF24_IRQ_PIN 5
#define NRF24_CS_PIN  6

class Radio {
public:
  bool initRadio();
  void powerDown();
};
