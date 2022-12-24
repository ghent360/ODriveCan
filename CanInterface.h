/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>

namespace odrive {

class ODriveAxis;

// You may need to adapt this class to your board
class CanInterface {
public:
    CanInterface(ODriveAxis* axes, uint8_t numAxes)
        : numAxes_(numAxes), axes_(axes) {}
    void canInit();
    void readAndProcessCan();
    void canSleep();

    // Adapt these to match your board configuration. I use 2 CAN busses
    // CAN1 and CAN3 - each of these methods sends a message to the 
    // corresponding CAN 'channel'.
    //
    // It is ok to leave one unused, but if you need more can busses,
    // add more of these static methods.
    static void sendCmdCh1(uint32_t canId, uint8_t len, uint8_t *buf);
    static void sendCmdCh3(uint32_t canId, uint8_t len, uint8_t *buf);
private:
    const uint8_t numAxes_;
    ODriveAxis* axes_;
};

} // namespace odrive