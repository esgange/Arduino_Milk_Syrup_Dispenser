# Steam Wand Cleaner (Modbus RTU Slave ID 2)

Nano Every firmware for steam-wand cleaning cycles with thermocouple feedback. Non-blocking INIT/FROTH/CLEAN so ABORT can preempt at any time.

## Bus and Protocol
- RS-485 on Serial1, 19200 8N1
- `RE`: A1, `DE`: A2
- Slave ID: 2
- Function: 0x17 (Read/Write Multiple Registers)
- Command block at 0x0000, result block at 0x0100 (same layout as dispenser; tempC x10 in `0104`)

### Opcodes
- `STATUS(1)`, `CLEAR(2)`, `ABORT(3)`, `OFF(4)`
- `INIT(10)`: arg `seconds_x10`
- `FROTH(11)`: args `targetC_x10`, `timeout_s_x10`
- `CLEAN(12)`: args `tValve_x10`, `tSteam_x10`, `tStandby_x10`

### Result block fields
- `0100`: `resultCode` (`0=OK`, `1=FAIL`)
- `0101`: `errorCode` (`NONE, BUSY, TIMEOUT, BAD_ARGS, INVALID_CMD, ABORTED`)
- `0102`: `systemStatus` (`IDLE, ACTIVE, TIMEOUT_FAULT, ABORTED_FAULT, OFF`)
- `0103`: `seq` echo
- `0104`: `tempC_x10` (0 if invalid)
- `0105`: `elapsed_s_x10` (when set by operations)

## Hardware Map
- Solenoid: 2
- Motor driver (L298): `M1_IN1=3`, `M1_IN2=4`, `M2_IN1=5`, `M2_IN2=6`, `ENA=7`, `ENB=8`
- Thermocouple (bit-banged MAX6675): `SO=12`, `CS=10`, `SCK=13`
- RS-485: `RE=A1`, `DE=A2`

## Behavior
- Modes: OFF, STANDBY, INIT, FROTH, CLEAN.
- INIT: drive for requested duration.
- FROTH: heat to target A°C or timeout; caches temp to `tempC_x10`.
- CLEAN: valve → steam → standby, all non-blocking.
- ABORT: preempts, sets `ABORTED_FAULT`, drives motors to standby posture.
- OFF: true Hi-Z and `systemStatus=OFF`.

## Local Serial CLI (115200)
- `help`
- `status`, `clear`, `abort`, `off`
- `init <seconds>`
- `froth <targetC> <timeout_s>`
- `clean <tValve> <tSteam> <tStandby>`
