/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>
#include "ODriveCan.hpp"

namespace odrive {

bool ParseCanMsg(
    ODriveAxis *axes,
    uint8_t numAxes,
    uint32_t msgId,
    uint8_t dataLen,
    const CanMsgData& buf);
void printCanMessage(uint32_t id, uint8_t len, const CanMsgData& buf);

} // namespace odrive