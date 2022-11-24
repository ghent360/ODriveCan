#include <same51_can.h>
#include "ODriveCan.hpp"

using odrive::ODriveAxis;

SAME51_CAN can;

static void SendCmdCh0(uint32_t canId, uint8_t len, uint8_t *buf) {
  can.sendMsgBuf(canId, 0, len, buf);
}

ODriveAxis axesCh0[] = {
  ODriveAxis(1, SendCmdCh0),
  ODriveAxis(3, SendCmdCh0),
  ODriveAxis(5, SendCmdCh0),
  ODriveAxis(7, SendCmdCh0),
  ODriveAxis(9, SendCmdCh0),
  ODriveAxis(11, SendCmdCh0),
};

PeriodicTimer myTimers[] = {
  PeriodicTimer(150, [](uint32_t)->void {
    for(auto& axis : axesCh0) {
      axis.hb.PeriodicCheck(axis);
      if (axis.hb.error != 0) {
        Serial.print("Error axis ");
        Serial.print(axis.node_id);
        Serial.print(" code ");
        Serial.println(axis.hb.error);
      }
    }
  }),
  PeriodicTimer(1000, [](uint32_t)->void {
    static uint8_t axis = 0;
    if (axesCh0[axis].hb.alive) {
      axesCh0[axis].RequestVbusVoltage();
    }
    axis++;
    if (axis > 5) axis = 0;
  })
};

void PrintCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
  Serial.print("Id: ");
  Serial.print(id);
  Serial.print(" Len: ");
  Serial.print(len);
  Serial.print(" Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(", ");
  }
  Serial.println();
}

void ProcessCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf) {
  bool parseSuccess = ParseCanMsg(axesCh0, id, len, buf);
  if (!parseSuccess) {
    PrintCanMessage(id, len, buf);
  }
}

uint32_t startTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  uint8_t ret;
  ret = can.begin(MCP_ANY, CAN_1000KBPS, MCAN_MODE_CAN);
  if (ret == CAN_OK) {
      Serial.println("CAN Initialized Successfully!");
  } else {
      Serial.println("Error Initializing CAN...");
      while(1);
  }
  for(auto& axis : axesCh0) {
    axis.vbus.SetCallback([](ODriveAxis& axis, VbusVoltage& v) { 
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.print(" vbus = ");
      Serial.println(v.val);
    });
    axis.hb.SetUnreachableCallback([](ODriveAxis& axis) {
      Serial.print("Axis ");
      Serial.print(axis.node_id);
      Serial.println(" unreachable");
    });
  }
  startTime = millis();
  StartAllTimers(myTimers, startTime);
}

void loop() {
  uint8_t ret;
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];
  uint8_t i;

  CheckAllTimers(myTimers, millis());
  ret = can.readMsgBuf(&id, &len, buf);
  if (ret == CAN_OK) {
    ProcessCanMessage(id, len, buf);
  }
}
