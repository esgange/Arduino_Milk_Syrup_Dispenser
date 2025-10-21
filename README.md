# Weigh-Scale Feedback Liquid Dispenser (+ Steam Wand Cleaner)

**Motor Controller (Mega 2560) v1.1 — 2025-10-12**  
**Steam Wand Cleaner Slave (Arduino) v1.0 — 2025-10-12**  
**Scale Slave (Nano Every + HX711) v1.0 — 2025-10-10**  
**Reference Modbus RTU Master (Nano Every) v1.0**

Authors: **Erdie Gange · ChatGPT 5** · Region: **Asia/Riyadh**

---

> One-shot liquid dispenser with weigh-scale feedback, latched leak/motor/timeout faults, and a minimalist **Modbus RTU (0x17)** slave API, plus a **Steam Wand Cleaner** slave (thermocouple-driven). A single **Nano Every master** provides a USB CLI and controls **two slaves** on one RS-485 bus.

## Contents
- [Overview](#overview)
- [Hardware](#hardware)
  - [Mega 2560 — Dispenser Controller](#mega-2560--dispenser-controller)
  - [Arduino — Steam Wand Cleaner](#arduino--steam-wand-cleaner)
  - [Nano Every — Scale Module](#nano-every--scale-module)
  - [Nano Every — Modbus Master (reference)](#nano-every--modbus-master-reference)
- [Build & Flash](#build--flash)
- [Commissioning](#commissioning)
- [Operation](#operation)
  - [Dispenser dispense profile](#dispenser-dispense-profile)
  - [Cleaner cycles](#cleaner-cycles)
  - [Fault model](#fault-model)
  - [Logging conventions](#logging-conventions)
- [Scale firmware (UI & I²C)](#scale-firmware-ui--i²c)
- [Modbus RTU API](#modbus-rtu-api)
  - [Common register map](#common-register-map)
  - [Dispenser opcodes (ID=1)](#dispenser-opcodes-id1)
  - [Cleaner opcodes (ID=2)](#cleaner-opcodes-id2)
  - [Error/status codes](#errorstatus-codes)
  - [Motor IDs & tokens](#motor-ids--tokens)
- [Reference Master CLI](#reference-master-cli)
- [Troubleshooting](#troubleshooting)

---

## Overview
Two RS-485 slaves share a simple **Modbus RTU 0x17 Read/Write Multiple Registers** interface:

- **Dispenser (ID=1)** — drives per-flavor motors and rinse solenoid; reads live mass from a dedicated I²C scale module (Nano Every + HX711).  
  Operations: `STATUS, CLEAR, ABORT, DISPENSE, RINSE, TRIGGER`.

- **Steam Wand Cleaner (ID=2)** — solenoid + dual DC drives; non-blocking **INIT/FROTH/CLEAN** cycles with thermocouple feedback and **OFF** posture.  
  Operations: `STATUS, CLEAR, ABORT, OFF, INIT, FROTH, CLEAN`.

A single **Nano Every master** provides a USB CLI to control both slaves (`d.*` for dispenser, `c.*` for cleaner). The master is **non-blocking**: start commands log a compact `[sent] ...` line, then **stay silent while waiting**, and finally print exactly one outcome line (`done.` or `ended: FAULT`).

---

## Hardware
### Mega 2560 — Dispenser Controller
- **RS-485**: `DE→A2`, `RE→A3` (RE active-LOW). **Serial3 @ 19200 8N1**, **Slave ID = 1**.
- **Motors**  
  Milk pins **2..9** (8 ch); Sauce pins **10,11,12,25..47 odd** (15 ch).
- **Speed select**: `PIN_SPEEDSEL=23` (LOW → **24 V High**, HIGH → **12 V Low**). Default after reset: **High**.
- **Rinse**: pin **49**.  
- **Motor fault in**: pin **51** (active-LOW, 1 s debounce).  
- **Leak sensors** (LMV331): `(A5/A6)`, `(A8/A9)`; `A6/A9` power, `A5/A8` logic input.  
- **I²C scales**: Milk = **0x2A**, Sauce = **0x2B**.

### Arduino — Steam Wand Cleaner
- **RS-485**: Serial1 @ **19200 8N1**, **Slave ID = 2** (RE/DE on GPIO, line-idle receive).  
- **Thermocouple**: `SO=12, CS=10, SCK=13` (bit-banged MAX6675-style).  
- **Actuators**: `SOLENOID=2`, `M1_IN1=3`, `M1_IN2=4`, `M2_IN1=5`, `M2_IN2=6`, `ENA=7`, `ENB=8`.  
- **Modes**: `OFF, STANDBY, INIT, FROTH, CLEAN`. Non-blocking phase engine; `ABORT` → safe **STANDBY** posture; `OFF` → outputs safe.

### Nano Every — Scale Module
- HX711: `DOUT=A2`, `SCK=A3`  
- Button (tare/cal): `A1` (active-LOW, PULLUP)  
- 7-segment: `DIGIT` 2,3,4 (active HIGH), `SEG` 5..12 (active HIGH)  
- LEDs: **Green=RX(0)**, **Red=TX(1)**  
- I²C address: **0x2B** (Sauce) or **0x2A** (Milk)

### Nano Every — Modbus Master (reference)
- MAX485: **RO→D0(RX1)**, **DI→D1(TX1)**, **RE+DE→D2** (HIGH=TX, LOW=RX)  
- Bus: **19200 8N1**.  
- USB CLI with `d.*` and `c.*` commands; silent background polling.

> ⚠️ Use correct RS-485 termination (120 Ω) and biasing to your panel standard.

---

## Build & Flash
> Arduino IDE 2.x (or CLI). No external HX711 library is required.

**Dispenser (Mega 2560)**
1. Select **Arduino Mega or Mega 2560**.  
2. Set `MODBUS_SERIAL=Serial3`, `MODBUS_BAUD=19200`, `MODBUS_SLAVE_ID=1`.  
3. Flash. Open Serial Monitor @ **115200**.

**Cleaner (Arduino with Serial1)**
1. Confirm `MODBUS_BAUD=19200`, **Slave ID = 2**.  
2. Flash. Open Serial Monitor @ **115200** to view events.

**Scale Module(s) (Nano Every)**
1. Set `I2C_ADDR` to **0x2B** (Sauce) or **0x2A** (Milk).  
2. Flash. On boot: auto-tare (`SEt`) → **READING**.

**Master (Nano Every)**
1. Wire MAX485 as above; flash the merged two-slave master.  
2. Open USB Serial @ **115200**.

---

## Commissioning
1. Verify RS-485 wiring and termination; power the **scales** first (auto-tare).  
2. Calibrate each scale: long-press ≥3 s → **CAL**, place known mass, short-press to save.  
3. Power Dispenser and Cleaner; watch ready banners.  
4. From the Master CLI:  
   - `d.status` and `c.status` → expect valid Modbus replies.  
   - Dispenser dry test: `d.trigger sauce2 1.0`.  
   - Cleaner dry test: `c.init 5` then `c.abort` (observe `ABORTED_FAULT`).  
5. Wet test (Dispenser): `d.dispense m1 120 10 2 45`.

---

## Operation
### Dispenser dispense profile
- Start **24 V High**, then drop to **12 V Low** at `target − slowOffset`.  
- **Soft-cut** at `target − softCutOffset`, then brief settle/coast.  
- Hard-cut if target reached earlier.  
- Per-dispense **timeout**; on timeout → safe outputs + latched fault.

### Cleaner cycles
- **INIT**: drive/prime for a fixed time.  
- **FROTH**: heat until `targetC` or timeout.  
- **CLEAN**: Valve phase → Steam phase → Standby phase; all non-blocking.  
- `ABORT` transitions to safe **STANDBY** posture; `OFF` powers outputs down.

### Fault model
Latched until `CLEAR`:
- `MOTOR_FAULT`, `LEAK_FAULT`, `SCALE_FAULT`, `TIMEOUT_FAULT`, `ABORTED_FAULT`.  
Cleaner also reports `OFF` as a distinct **systemStatus**.

### Logging conventions
- When a command is issued: **one** concise line, e.g.  
  `[sent] DISPENSE motor=Sauce2 target_g=8.0 slowOffset_g=15.0 softCutOffset_g=1.4 timeout_s=20.`  
  `[sent] CLEAN tValve_s=4.0 tSteam_s=6.0 tStandby_s=5.0.`
- While waiting: **silent** background polling.  
- On completion: **one** final line, e.g.  
  `[dispense] done.` or `[froth] ended: TIMEOUT_FAULT`.

---

## Scale firmware (UI & I²C)
**Modes**: `IDLE`, `READING`, `CAL`, overflow `HI`.  
**Tare/Cal**: startup auto-tare (`SEt`), short press = tare, long press = enter CAL (place mass, short press to save).  
**I²C**:
- **Write**: `'R' + uint16 target_x10` (little-endian) → tare + arm target.  
- **Read**: `[hit_once, weight_x10_lo, weight_x10_hi]`. Red LED latches when target hit; cleared by next tare.

---

## Modbus RTU API
Link: **19200 8N1**. Only **Function 0x17** is accepted. Both slaves share the same block layout (`0x0100..0x010A` result).

### Common register map
**Command Block @ `0x0000`**

| Addr  | Field   | Notes |
|:-----:|:--------|:------|
| `0000` | `opcode` | Per device (see below) |
| `0001` | `seq` | Echo (master may write `0`) |
| `0002..0007` | `args` | Per-opcode |

**Result Block @ `0x0100`**

| Addr  | Field | Meaning |
|:-----:|:------|:--------|
| `0100` | `resultCode` | `0=OK`, `1=FAIL` |
| `0101` | `errorCode` | See table below |
| `0102` | `systemStatus` | See table below |
| `0103` | `seq` | Echo |
| `0104` | aux0 | Dispenser: `lastWeight_x10` (g×10) · Cleaner: **tempC_x10** |
| `0105` | aux1 | `elapsed_s_x10` |
| `0106..010A` | reserved | `0` |

### Dispenser opcodes (ID=1)
- `STATUS(1)` — no args.  
- `CLEAR(2)` — clears latched faults.  
- `ABORT(3)` — immediate safe-stop → `ABORTED_FAULT`.  
- `DISPENSE(10)` — args: `motorId`, `target_x10`, `slowOffset_x10`, `softCutOffset_x10`, `timeout_s`.  
- `RINSE(11)` — args: `seconds_x10`.  
- `TRIGGER(12)` — args: `motorId`, `seconds_x10`.

### Cleaner opcodes (ID=2)
- `STATUS(1)` — no args.  
- `CLEAR(2)` — clears latched faults (returns to `IDLE`/`STANDBY`).  
- `ABORT(3)` — aborts current phase → `ABORTED_FAULT` + **STANDBY** posture.  
- `OFF(4)` — outputs safe; `systemStatus=OFF`.  
- `INIT(10)` — args: `seconds_x10`.  
- `FROTH(11)` — args: `targetC_x10`, `timeout_s_x10`.  
- `CLEAN(12)` — args: `tValve_x10`, `tSteam_x10`, `tStandby_x10`.

### Error/status codes
**errorCode (`0x0101`)**  
`NONE(0), BUSY(1), MOTOR_FAULT(2), LEAK_FAULT(3), SCALE_FAULT(4), TIMEOUT(5), BAD_ARGS(6), INVALID_CMD(7), ABORTED(8)`

**systemStatus (`0x0102`)**  
`IDLE(0), ACTIVE(1), MOTOR_FAULT(2), LEAK_FAULT(3), SCALE_FAULT(4), TIMEOUT_FAULT(5), ABORTED_FAULT(6), OFF(7*)`  
\*`OFF` is reported by the Cleaner.

### Motor IDs & tokens
- **Milk**: `1..8` — tokens: `Milk1..Milk8`, `m1..m8`  
- **Sauce**: `101..115` — tokens: `Sauce1..Sauce15`, `s1..s15`  
Numeric IDs are accepted.

---

## Reference Master CLI
USB Serial @ **115200**.  
**Dispenser (`d.*`)**
```
d.status
d.clear
d.abort
d.rinse 2.0
d.trigger Sauce2 1.0
# d.dispense <motor|name> <target_g> <slow_g> <soft_g> <timeout_s>
d.dispense Sauce2 8 15 1.4 20
d.dispense Milk1 5 5 0.1 20
```

**Cleaner (`c.*`)**
```
c.status
c.clear
c.abort
c.off
c.init 5
c.froth 65 120
c.clean 4 6 5
```

Example responses are brief:
```
result=OK  error=NONE  status=ACTIVE
...
[dispense] done.
[froth] ended: TIMEOUT_FAULT
```

---

## Troubleshooting
**No Modbus response / CRC** — verify RE/DE direction (D2), **19200 8N1**, correct Slave ID, and 120 Ω termination with bias.  
**BUSY** — an op is running. Use `...abort`, then `...clear`.  
**SCALE_FAULT** — I²C wiring/address or HX711 power on the scale module.  
**LEAK/MOTOR faults** — inspect sensors/driver; faults are **latched** until `CLEAR`.  
**Cleaner OFF** — use `c.status` to confirm; `c.init`/`c.froth`/`c.clean` will set `ACTIVE` on acceptance.
