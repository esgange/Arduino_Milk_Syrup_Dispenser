# Dispenser Controller (Modbus RTU Slave ID 1)

Mega 2560 firmware for single-shot dispensing with dual I2C scales, rinse control, and diagnostic triggers. Uses Modbus RTU function 0x17 with a single command/result transaction model.

## Bus and Protocol
- RS-485 on Serial1, 19200 8N1
- `RE`/`DE`: pin 17 (tied)
- Slave ID: 1
- Function: 0x17 (Read/Write Multiple Registers)
- Command block: `0x0000..` (opcode + args), Result block: `0x0100..` (status + data)
- Turnaround: DE/RE is forced back to RX every loop when idle; replies include a T3.5 settle after flush to keep the bus clean.

### Opcodes
- `STATUS(1)`: no args
- `CLEAR(2)`: clears latched faults and safes outputs
- `ABORT(3)`: immediate stop, latches `ABORTED_FAULT`
- `DISPENSE(10)`: args `motorId`, `target_x10`, `slowPct_x10`, `softCutPct_x10`, `timeout_s`
- `RINSE(11)`: arg `seconds_x10`
- `TRIGGER(12)`: args `motorId`, `seconds_x10`, `flags` (bit0=high-speed)

### Result block (0x0100)
- `0100`: `resultCode` (`0=OK`, `1=FAIL`)
- `0101`: `errorCode` (`NONE, BUSY, MOTOR_FAULT, LEAK_FAULT, SCALE_FAULT, TIMEOUT, BAD_ARGS, INVALID_CMD, ABORTED`)
- `0102`: `systemStatus` (`IDLE, ACTIVE, MOTOR_FAULT, LEAK_FAULT, SCALE_FAULT, TIMEOUT_FAULT, ABORTED_FAULT`)
- `0103`: `seq` echo
- `0104`: `lastWeight_x10` (grams x10)
- `0105`: `elapsed_s_x10` (where applicable)
- `0106`: `activeScaleId` (`1=Milk`, `2=Sauce`)

## Hardware Map
- Motor pins: Milk `2..9` (IDs 11–18), Sauce `10,11,12,25..47` odd (IDs 21–35)
- Rinse solenoid: pin 49
- Speed select: pin 23 (HIGH=24 V rail, LOW=trimmed low rail)
- Motor fault input: pin 51 (active-LOW, 0.5 s debounce for dispense/rinse, 3 s for trigger)
- Leak sensors: `(A5,A6)` and `(A8,A9)` (A6/A9 supply, A5/A8 input)
- I2C scales: Milk `0x2A`, Sauce `0x2B`

## Behavior
- Dispense: starts high-speed, drops to low-speed at `slowPct%` of target, optional soft-cut up to ~1.5 g early; 3 s retry after timeout before latching `TIMEOUT_FAULT`.
- Rinse: timed solenoid run; blocked during dispense.
- Trigger: diagnostic pin drive for N seconds; rejected/aborted if motor fault holds for 3 s.
- Faults (latched until `CLEAR`): motor, leak, scale, timeout, aborted. Any fault safes outputs.

## Local Serial CLI (115200)
- `status`, `clear`, `abort`
- `rinse <seconds>`
- `trigger <motor> <seconds> [s|speed]`
- `dispense <motor> <target_g> <slowPct0-100> <softCut0-100> <timeout_s>`

## Notes
- Low-rail trim pot should stay at or below ~12 V.
- Scale selection is automatic based on motor ID.
