/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#pragma once

#include <stdint.h>

struct TxDataPacket {
  union {
    struct {
      uint8_t sw4:2;
      uint8_t sw3:2;
      uint8_t sw2:2;
      uint8_t sw1:2;
    };
    uint8_t data;
  } state1;
  union {
    struct {
      uint8_t reserved:7;
      bool sw5:1;
    };
    uint8_t data;
  } state2;
  int16_t x1;
  int16_t y1;
  int16_t z1;
  int16_t x2;
  int16_t y2;
  int16_t z2;
} __attribute__((packed));

struct RxPacketHdr {
  union {
    struct {
      uint8_t reserved:6;
      bool walk:1;
      bool control:1;
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
      bool a1error:1;
    };
    uint16_t data;
  } errors;
  uint8_t b1_voltage;
  uint8_t b2_voltage;
  uint8_t rx_voltage;
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
