/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
 *
*/
#include "CanInterface.h"
#include "ODriveCan.hpp"
#include "Arduino.h"

namespace odrive {

bool ParseCanMsg(
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
void printCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
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

} // namespace odrive
