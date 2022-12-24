/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "CanInterface.h"
#include "ODriveCan.hpp"

#ifdef ARDUINO_TEENSY41
#include <FlexCAN_T4.h>
#elif ARDUINO_CANBED_M4
#include <same51_can.h>
#else
#error Board is not supported.
#endif

namespace odrive {

static bool ParseCanMsg(
    ODriveAxis *axes,
    uint8_t numAxes,
    uint32_t msgId,
    uint8_t dataLen,
    const CanMsgData& buf) {
  uint32_t axisId = (msgId >> 5);
  uint8_t cmdId = (uint8_t)(msgId & 0x1f);
  ODriveAxis *axesEnd = axes + numAxes;
  for(ODriveAxis *axis = axes; axis < axesEnd; axis++) {
    if (axis->node_id == axisId) {
      return axis->Parse(cmdId, dataLen, buf);
    }
  }
  return false;
}

// Helper to print CAN messages we can not parse.
static void printCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
  Serial.print("Id: ");
  Serial.print(id >> 5);
  Serial.print(" ");
  Serial.print((int)(id & 0x1f));
  Serial.print(" Len: ");
  Serial.print(len);
  Serial.print(" Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(", ");
  }
  Serial.println();
}

#ifdef ARDUINO_TEENSY41
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_32> can3;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_32> can1;

void CanInterface::canInit() {
  // Enable CAN trasciever chips
  // CAN 1
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  // CAN 3
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  // Init can bus
  can1.begin();
  can1.setBaudRate(1000000);
  can1.enableFIFO();
  can3.begin();
  can3.setBaudRate(1000000);
  can3.enableFIFO();
}

void CanInterface::canSleep() {
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
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
      delayMicroseconds(200); // Time to transmit 200 bits @ 1MBit/s
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

#elif ARDUINO_CANBED_M4
SAME51_CAN can;

void CanInterface::canInit() {
  uint8_t ret;
  ret = can.begin(MCP_ANY, CAN_1000KBPS, MCAN_MODE_CAN);
  if (ret == CAN_OK) {
      Serial.println("CAN Initialized Successfully!");
  } else {
      Serial.println("Error Initializing CAN...");
      while(1);
  }
}

void CanInterface::canSleep() {
}

void CanInterface::sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf) {
  uint8_t ret;
  do {
    ret = can.sendMsgBuf(canId, 0, len, buf);
    if (ret == CAN_FAILTX) {  // TX queue is full, wait a bit.
      delayMicroseconds(200); // Time to transmit 200 bits @ 1MBit/s
    }
  } while (ret == CAN_FAILTX);

  if (ret != CAN_OK) {
    Serial.print("Failed to send message ID ");
    Serial.print(canId, HEX);
    Serial.print(" error ");
    Serial.println(ret);
  }
}

void CanInterface::sendCmdCh3(uint32_t canId, uint8_t len, uint8_t *buf) {
  sendCmdCh1(canId, len, buf);
}

// Note this should be non-blocking code, if there are no CAN
// messages in the RX queue, readMsgBuf should return some error
// but not wait for messages to appear.
void CanInterface::readAndProcessCan() {
  uint32_t id;
  uint8_t len;
  static uint8_t buf[8];

  if (can.readMsgBuf(&id, &len, buf) == CAN_OK) {
    bool parseSuccess = ParseCanMsg(axes_, numAxes_, id, len, buf);
    if (!parseSuccess) {
      printCanMessage(id, len, buf);
    }
  }
  // Add reading from other CAN busses here, process in similar fashion.
}
#endif

} // namespace odrive