## Transmitter packet format
| Byte offset | Length (bytes) |Description |
|-----:|----:|:------|
|00|1|0baabbccdd 4x2bit SW1-4 states|
|01|1|0bsxxxxxxx<br>s - SW5 state<br>xxx - 7b other states (reserved)|
|02|2|x1 value int16_t|
|04|2|y1 value int16_t|
|06|2|z1 value int16_t|
|08|2|x2 value int16_t|
|0A|2|y2 value int16_t|
|0C|2|z2 value int16_t|
|0E|1|command code - 0 noop|

SW5 - control motor idle/closed loop mode (ON - closed loop, OFF - idle); SW1,2 have no effect in idle mode;
SW1 - control walk (OFF - walk, MID - stop, ON - reset); SW1 only active of SW2 is in MID position
SW2 - park legs (OFF - static body move, MID - enable walking, ON - park legs); Switching SW2 if SW1 is not in mid pos has no effect.
SW3 - gait switch (not implemented)
SW4 - reserved for future use

## Receiver packet format

| Byte offset | Length (bytes) |Description |
|-----:|----:|:------|
|00|1|0babxxxxxx<br>a - motor state (0-idle, 1-closed loop)<br>b - walk state (0 off, 1 on)<br>xxx - reserved |
|01|2|12 bit axis error state, 4 low bits are reserved|
|03|1|Battery 1 voltage - Fixed<uint8_t, 8, 5, 18>|
|04|1|Battery 2 voltage - Fixed<uint8_t, 8, 5, 18>|
|05|1|Receiver battery voltage Fixed<uint8_t, 8, 6, 6>|
|06|1|Extended data type, 0 - no extension|
|07|?|extended data up to 24 bytes|

### Extended data type 1 sub-packed format - axis error info

Only sub-packets for axes with error bits in the main axis error packed are going to be transmitted. One sub-packet at a time. It may take op to 12 packets to receive error info for all all axes.

| Byte offset | Length (bytes) |Description |
|-----:|----:|:------|
|07|1| Axis id (1-12)|
|08|3| Axis error (uint32_t) most significant byte is always 0|
|0B|5| Motor error (uint64_t) most significant 3 bytes are always 0<br>valid only if MOTOR_FAILED bit in the Axis error is active|
|10|2| Encoder error (uint32_t) most significant 2 bytes are always 0<br>valid only if ENCODER_FAILED bit in the axis error is active|

