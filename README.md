# ODriveCan
Arduino library to communicate with ODrive 3.x boards. This library is not tied to any particular CAN bus hardware implementation. Note: the ODrive Pro and S1 can protocol is very similar to the ODrive 3.x, some of the code may work as is, however the PRO protocol continues to evolve and compatibility is not guarranteed. I don't have PRO or S1 boards, so I can not test if it works or not use at your own risk.

The provided example Arduino code is tested on SAME51 board from seeed studio as well as Teensy 4.1.

The library can communicate over several can busses with some boards on one bus and the rest on another bus.

The main API is in the ODriveAxis class defined in ODriveCan.hpp. It implements communications with a single motor axis. Each odrive 3.x defines can drive 2 motors, therefore you can instantiate two ODriveAxis instances per board. Each ODriveAxis has associated CAN ID, this ID is configured in the board beforehand via the odrivetool. CAN IDs need to be unique even if the boards are connected on separate CAN busses.

All boards on a single CAN bus have to use the same speed. The speed is configured via odrivetool. The Arduino CAN code is abstracted inside CanInterface.cpp - currently there are two implementations one for SAME51 and another for Teensy 4.1. You can add a custom implementation for your board.

Currently the CAN message parser needs to have access to all ODriveAxis instances. These are declared in a global array called 'axes' on the bottom of CanInterface.cpp. The current code declares 12 axes - you should update that to match your robot/machine configuration as well as the CAN ID you have configured for your boards. I'm looking for a way to remove this coupling, so one does not have to modify the library code.

Example axis definition: ODriveAxis( 9, sendCmdCh1). Here 9 represents the CAN ID for the motor axis and 'sendCmdCh1' is a function declared in CanInterface.cpp that sends can messages to the can bus where the board is connected. With this API you can communicate over different busses to reduce each message bus load.

In the current code I use two CAN busses CAN1 and CAN3 on Teensy 4.1. I have defined sendCmdCh1 and sendCmdCh3 to send messages over CAN1 and CAN3. In the SAME51 implementation, the board I use has only one CAN channel so sendCmdCh3 simply calls sendCmdCh1. The only reason to have sendCmdCh3 defined for the SAME51 board is to maintain compatible axis definitions across both boards.
