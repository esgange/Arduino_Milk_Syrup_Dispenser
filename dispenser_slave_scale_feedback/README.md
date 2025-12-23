# Scale Module (I2C HX711)

Nano Every firmware for a 3-digit 7-seg display + HX711 load cell. Provides tare/cal UI and an I2C protocol for the dispenser controller.

## Hardware
- HX711: `DOUT=A2`, `SCK=A3`
- Button: `A1` (active-LOW, pull-up)
- 7-seg: digits 2,3,4 (active HIGH); segments 5..12 (active HIGH)
- LEDs: Green=RX(0), Red=TX(1)
- Buzzer: A0
- I2C address: set `I2C_ADDR` (0x2A or 0x2B)

## Modes
- IDLE: display blank, green LED off
- READING: live weight, green LED on
- CAL: shows `CAL`; enter via long press (~3 s)
- Overflow shows `HI` and clamps I2C to 999.0 g

## UI Actions
- Startup: auto-tare (20-sample average), enters READING
- Short press: tare (20-sample average), clears red LED and target
- Long press: enter CAL, perform silent tare, place known weight, short press to store scale to EEPROM

## I2C Protocol
- Write: `'R' + uint16 target_x10` (little endian) → performs tare then arms target, clears red LED
- Read: 3 bytes `[hit_once, weight_x10_lo, weight_x10_hi]`; `hit_once` clears after one read
- Red LED latches when target reached; cleared on tare or new target

## Notes
- Sign is inverted in firmware to match flipped sensor orientation.
- Auto-idle after 60 s without a ≥1 g change while in READING.
