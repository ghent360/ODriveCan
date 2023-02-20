/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include "CanInterface.h"
#include "CanInterfaceCommon.h"
#include "ODriveCan.hpp"

#ifdef ARDUINO_TEENSY41
#include <FlexCAN_T4.h>

namespace odrive {

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_32> can3;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_32> can1;

void CanInterface::canInit() {
  // Enable CAN trasciever chips
  // CAN 1
  pinMode(2, OUTPUT);
  digitalWriteFast(2, LOW);
  // CAN 3
  pinMode(3, OUTPUT);
  digitalWriteFast(3, LOW);
  // Init can bus
  can1.begin();
  can1.setBaudRate(1000000);
  can1.enableFIFO();
  can3.begin();
  can3.setBaudRate(1000000);
  can3.enableFIFO();
}

void CanInterface::canSleep() {
  digitalWriteFast(3, HIGH);
  digitalWriteFast(2, HIGH);
}

void CanInterface::sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf) {
  CAN_message_t msg;
  int ret;
  msg.id = canId;
  if (len > 0 && len <= 8) {
    memcpy(msg.buf, buf, len);
  }
  msg.len = len;
  do {
    ret = can1.write(msg);
    if (ret == 0) {  // TX queue is full, wait a bit.
      delayMicroseconds(170); // Time to transmit 170 bits @ 1MBit/s
    }
  } while (ret == 0);
}

void CanInterface::sendCmdCh3(uint32_t canId, uint8_t len, uint8_t *buf) {
  CAN_message_t msg;
  int ret;
  msg.id = canId;
  if (len > 0 && len <= 8) {
    memcpy(msg.buf, buf, len);
  }
  msg.len = len;
  do {
    ret = can3.write(msg);
    if (ret == 0) {  // TX queue is full, wait a bit.
      delayMicroseconds(200); // Time to transmit 200 bits @ 1MBit/s
    }
  } while (ret == 0);
}

void CanInterface::readAndProcessCan() {
  CAN_message_t msg1;
  CAN_message_t msg2;
  if ( can1.read(msg1) ) {
    bool parseSuccess = ParseCanMsg(axes_, numAxes_, msg1.id, msg1.len, msg1.buf);
    if (!parseSuccess) {
      printCanMessage(msg1.id, msg1.len, msg1.buf);
    }
  }
  if ( can3.read(msg2) ) {
    bool parseSuccess = ParseCanMsg(axes_, numAxes_, msg2.id, msg2.len, msg2.buf);
    if (!parseSuccess) {
      printCanMessage(msg2.id, msg2.len, msg2.buf);
    }
  }
}

} // namespace odrive

#endif // defined ARDUINO_TEENSY41