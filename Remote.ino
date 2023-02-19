/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 *
 * This is my version of the control software for OpenDog V3 remote.
 */
#include <Arduino.h>

#include "RemoteInput.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  remoteInputs.begin();
}

static const char* sw3ToStr(SW3POS value) {
  switch(value) {
  case SW3_ON: return "on ";
  case SW3_OFF: return "off";
  case SW3_MID: return "mid";
  }
  return "unk";
}

static const char* sw2ToStr(SW2POS value) {
  switch(value) {
  case SW2_ON: return "on ";
  case SW2_OFF: return "off";
  }
  return "unk";
}

void loop() {
  remoteInputs.readValues();

  Serial.print("Input: ");
  Serial.print(remoteInputs.getX1());
  Serial.print("  ");
  Serial.print(remoteInputs.getY1());
  Serial.print("  ");
  Serial.print(remoteInputs.getZ1());
  Serial.print("  ");
  Serial.print(remoteInputs.getX2());
  Serial.print("  ");
  Serial.print(remoteInputs.getY2());
  Serial.print("  ");
  Serial.print(remoteInputs.getZ2());
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW1()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW2()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW3()));
  Serial.print("  ");
  Serial.print(sw3ToStr(remoteInputs.getSW4()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getSW5()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB1()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB2()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB3()));
  Serial.print("  ");
  Serial.print(sw2ToStr(remoteInputs.getB4()));
  Serial.println();

  remoteInputs.setB1Led(remoteInputs.getB4() == SW2_ON);
  remoteInputs.setB2Led(remoteInputs.getB1() == SW2_ON);
  remoteInputs.setB3Led(remoteInputs.getB2() == SW2_ON);
  remoteInputs.setB4Led(remoteInputs.getB3() == SW2_ON);
  delay(100);
}