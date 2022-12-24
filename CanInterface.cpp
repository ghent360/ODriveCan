/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "ODriveCan.hpp"
#include "kinematics.h"

#ifdef ARDUINO_TEENSY41
#include <FlexCAN_T4.h>
#elif ARDUINO_CANBED_M4
#include <same51_can.h>
#else
#error Board is not supported.
#endif

using odrive::CanMsgData;
using odrive::ODriveAxis;
using odrive::ParseCanMsg;

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

void canInit() {
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

void canSleep() {
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
}

static void sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf) {
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

static void sendCmdCh3(uint32_t canId, uint8_t len, uint8_t *buf) {
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

void readAndProcessCan() {
  CAN_message_t msg1;
  CAN_message_t msg2;
  if ( can1.read(msg1) ) {
    bool parseSuccess = ParseCanMsg(axes, msg1.id, msg1.len, msg1.buf);
    if (!parseSuccess) {
      printCanMessage(msg1.id, msg1.len, msg1.buf);
    }
  }
  if ( can3.read(msg2) ) {
    bool parseSuccess = ParseCanMsg(axes, msg2.id, msg2.len, msg2.buf);
    if (!parseSuccess) {
      printCanMessage(msg2.id, msg2.len, msg2.buf);
    }
  }
}

#elif ARDUINO_CANBED_M4
SAME51_CAN can;

void canInit() {
  uint8_t ret;
  ret = can.begin(MCP_ANY, CAN_1000KBPS, MCAN_MODE_CAN);
  if (ret == CAN_OK) {
      Serial.println("CAN Initialized Successfully!");
  } else {
      Serial.println("Error Initializing CAN...");
      while(1);
  }
}

void canSleep() {
}

static void sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf) {
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

static void sendCmdCh3(uint32_t canId, uint8_t len, uint8_t *buf) {
  sendCmdCh1(canId, len, buf);
}

// Note this should be non-blocking code, if there are no CAN
// messages in the RX queue, readMsgBuf should return some error
// but not wait for messages to appear.
void readAndProcessCan() {
  uint32_t id;
  uint8_t len;
  static uint8_t buf[8];

  if (can.readMsgBuf(&id, &len, buf) == CAN_OK) {
    bool parseSuccess = ParseCanMsg(axes, id, len, buf);
    if (!parseSuccess) {
      printCanMessage(id, len, buf);
    }
  }
  // Add reading from other CAN busses here, process in similar fashion.
}
#endif

// We communicate with 3 ODrive boards, each board has 2 axes. Each axis
// has to be setup separately. If you have boards that are connected to a
// different CAN bus, you would need separate CAN callback for each bus.
// For example SendCmdCh1.
//
// It is required to have unique node_id even if some axis are on a
// separate CAN bus. Otherwise the parsing code has to be modified to 
// process each CAN bus separately.
//
// On my setup the axes have even node_ids starting with 1. These have to
// be configured using the odrivetool.
ODriveAxis axes[numAxes] = {
  [FRONT_RIGHT_KNEE] = ODriveAxis( 8, sendCmdCh3),
  [FRONT_LEFT_KNEE] = ODriveAxis( 7, sendCmdCh1),
  [BACK_RIGHT_KNEE] = ODriveAxis( 6, sendCmdCh3),
  [BACK_LEFT_KNEE] = ODriveAxis( 5, sendCmdCh1),
  [FRONT_RIGHT_SHOULDER] = ODriveAxis( 4, sendCmdCh3),
  [FRONT_LEFT_SHOULDER] = ODriveAxis( 3, sendCmdCh1),
  [BACK_RIGHT_SHOULDER] = ODriveAxis(12, sendCmdCh3),
  [BACK_LEFT_SHOULDER] = ODriveAxis(11, sendCmdCh1),
  [FRONT_RIGHT_HIP] = ODriveAxis( 2, sendCmdCh3),
  [FRONT_LEFT_HIP] = ODriveAxis( 1, sendCmdCh1),
  [BACK_RIGHT_HIP] = ODriveAxis( 10, sendCmdCh3),
  [BACK_LEFT_HIP] = ODriveAxis( 9, sendCmdCh1)
};
