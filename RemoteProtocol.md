## Transmitter packet format
| Byte offset | Length (bytes) |Description |
|-----:|----:|:------|
|00|1|0baabbccdd 4x2bit SW1-4 states|
|01|1|0bsooooooo SW5 state, 7b other states|
|02|2|x1 value int16_t|
|04|2|y1 value int16_t|
|06|2|z1 value int16_t|
|08|2|x2 value int16_t|
|0A|2|y2 value int16_t|
|0C|2|z2 value int16_t|

SW5 - control motor idle/closed loop mode (ON - closed loop, OFF - idle)

## Receiver packet format

| Byte offset | Length (bytes) |Description |
|-----:|----:|:------|
|00|1|0babcdoooo<br>a - motor state (0-idle, 1-closed loop)<br>bcd - ?<br>oooo - other? |
|01|2|12 bit axis error state, 4 bit reserved|
|03|1|Battery 1 voltage - Fixed<uint8_t, 
8, 5, 18>|
|04|1|Battery 2 voltage - Fixed<uint8_t, 8, 5, 18>|
|05|1|Receiver battery voltage Fixed<uint8_t, 8, 6, 6>|
