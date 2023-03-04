/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <stdint.h>

// Note: these structures are sensitive to architecture endianness.
//
// Since both the Rx and Tx MCUs are the same (Teensy 4.1) in my case,
// this is not an issue for my implementation. However if you plan to
// use different MCUs for Rx and Tx, you should verify the data is
// transmitted/received in the correct byte order.

enum CommandCodes {
  CMD_NOOP = 0,
  CMD_CLEAR_ERRORS = 1,
  CMD_SET_GAINS = 2,
  CMD_MOVE_FL_FOOT = 3,
  CMD_MOVE_FR_FOOT = 4,
  CMD_MOVE_BL_FOOT = 5,
  CMD_MOVE_BR_FOOT = 6,
  CMD_MOVE_FOOT_DONE = 7,
  CMD_EDIT_HIP_GAIN = 8,
  CMD_EDIT_TIE_GAIN = 9,
  CMD_EDIT_SHIN_GAIN = 10,
  CMD_EDIT_GAIN_DONE = 11,
};

enum ExtensionTypes {
  EXT_TYPE_NONE = 0,
  EXT_AXIS_ERROR = 1,
  EXT_GAIN_VALUES = 2,
};

struct TxDataPacket {
  union {
    struct {
      uint8_t sw4:2;
      uint8_t sw3:2;
      uint8_t sw2:2;
      uint8_t sw1:2; // msb
    };
    uint8_t data;
  } state1;
  union {
    struct {
      uint8_t reserved:7;
      bool sw5:1; // msb
    };
    uint8_t data;
  } state2;
  int16_t x1;
  int16_t y1;
  int16_t z1;
  int16_t x2;
  int16_t y2;
  int16_t z2;
  uint8_t cmd;
} __attribute__((packed));

struct RxPacketHdr {
  union {
    struct {
      uint8_t reserved:6;
      bool walk:1;
      bool control:1; // msb
    };
    uint8_t data;
  } state;
  union {
    struct {
      uint8_t reserved:4;
      bool a12error:1;
      bool a11error:1;
      bool a10error:1;
      bool a9error:1;
      bool a8error:1;
      bool a7error:1;
      bool a6error:1;
      bool a5error:1;
      bool a4error:1;
      bool a3error:1;
      bool a2error:1;
      bool a1error:1; // msb
    };
    uint16_t data;
  } errors;
  uint8_t b1_voltage; // Fixed<uint8_t, 8, 5, 18>;
  uint8_t b2_voltage; // Fixed<uint8_t, 8, 5, 18>;
  uint8_t rx_voltage; // Fixed<uint8_t, 8, 6, 6>
  uint8_t ext_type;
} __attribute__((packed));

struct RxAxisError {
  uint8_t axis_id;
  union {
    uint32_t error:24;
    uint8_t data[3];
  } axis_error;
  union {
    uint64_t error:40;
    uint8_t data[5];
  } motor_error;
  union {
    uint32_t error:16;
    uint8_t data[2];
  } encoder_error;
} __attribute__((packed));

struct RxGainValues {
  uint8_t  axis_class; // AxisClass enum value
  uint16_t pos_gain; //  Fixed<uint16_t, 16, 7>;
  uint16_t vel_gain; //  Fixed<uint16_t, 16, 9>;
  uint16_t vel_int;  //  Fixed<uint16_t, 16, 11>;
} __attribute__((packed));

struct RxFootPos {
  uint8_t foot_id; // DogLeg enum value
  int16_t x;   //  Fixed<int16_t, 16, 6>;
  int16_t y;   //  Fixed<int16_t, 16, 6, 107>;
  int16_t z;   //  Fixed<int16_t, 16, 6, 350>;
  int16_t err; //  Fixed<int16_t, 16, 10>;
} __attribute__((packed));

struct RxPacket {
  struct RxPacketHdr hdr;
  union {
    struct RxAxisError axis_err;
    struct RxGainValues gains;
    struct RxFootPos foot_pos;
  } ext;
} __attribute__((packed));
