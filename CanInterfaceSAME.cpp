/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "CanInterface.h"
#include "CanInterfaceCommon.h"
#include "ODriveCan.hpp"

#ifdef ARDUINO_CANBED_M4
#include <same51_can.h>

namespace odrive {

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

} // namespace odrive

#endif // defined ARDUINO_CANBED_M4
