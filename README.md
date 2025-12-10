# Weigh-Scale Feedback Liquid Dispenser (+ Steam Wand Cleaner)

**Motor Controller (Mega 2560) v1.1 — 2025-10-12**  
**Steam Wand Cleaner Slave (Nano Every) v2.4 — 2025-10-21**  
**Scale Slave (Nano Every + HX711) v1.0 — 2025-10-10**  
**Reference Modbus RTU Master (Nano Every) v1.0**

Authors: **Erdie Gange · ChatGPT 5** · Region: **Asia/Riyadh**

---

> Single-shot liquid dispenser with weigh-scale feedback, latched leak/motor/timeout faults, and a minimalist **Modbus RTU (0x17)** slave API, plus a thermocouple-driven **Steam Wand Cleaner** slave. A single **Nano Every master** provides a USB CLI and drives **two slaves on one RS‑485 bus**.

## Contents
- [Overview](#overview)
- [Hardware](#hardware)
  - [Mega 2560 — Dispenser Controller (ID=1)](#mega-2560--dispenser-controller-id1)
  - [Nano Every — Steam Wand Cleaner (ID=2)](#nano-every--steam-wand-cleaner-id2)
  - [Nano Every — Scale Module](#nano-every--scale-module)
  - [Nano Every — Modbus Master (reference)](#nano-every--modbus-master-reference)
- [Build & Flash](#build--flash)
- [Commissioning](#commissioning)
- [Operation](#operation)
  - [Dispenser behavior](#dispenser-behavior)
  - [Cleaner behavior](#cleaner-behavior)
  - [Fault model](#fault-model)
  - [Logging conventions](#logging-conventions)
- [Scale firmware (UI & I²C)](#scale-firmware-ui--i%C2%B2c)
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
- Two RS-485 slaves share **Modbus RTU 0x17 Read/Write Multiple Registers**.
  - **Dispenser (ID=1)** — drives per-flavor motors, rinse solenoid, and reads live mass from dedicated I²C scales (Milk @ **0x2A**, Sauce @ **0x2B**). Non-blocking `DISPENSE/RINSE/TRIGGER` with latched fault handling.
  - **Steam Wand Cleaner (ID=2)** — solenoid + dual DC drives; non-blocking **INIT/FROTH/CLEAN** cycles with thermocouple feedback and a distinct **OFF** posture.
- A **Nano Every master** offers a USB CLI (`d.*` / `c.*`) and silently polls STATUS while operations run.
- All timeouts/leak/motor/scale faults are **latched until CLEAR**. ABORT is always available and latches `ABORTED_FAULT`.

---

## Hardware
### Mega 2560 — Dispenser Controller (ID=1)
- **RS-485**: `RE+DE → 17`, **Serial1 @ 19200 8N1**.  
- **Motors**: Milk pins **2..9** (8 ch); Sauce pins **10,11,12,25..47 odd** (15 ch).  
- **Speed select**: `PIN_SPEEDSEL=23` (**HIGH = 24 V High**, LOW = 12 V Low). Boot default: High.  
- **Rinse**: pin **49**.  
- **Motor fault in**: pin **51** (active-LOW, 1 s debounce).  
- **Leak sensors** (LMV331): `(A5/A6)` and `(A8/A9)`; `A6/A9` = sensor Vcc, `A5/A8` = logic input.  
- **I²C scales**: Milk = **0x2A**, Sauce = **0x2B**.

### Nano Every — Steam Wand Cleaner (ID=2)
- **RS-485**: `RE=A1`, `DE=A2`, **Serial1 @ 19200 8N1**.  
- **Thermocouple**: `SO=12`, `CS=10`, `SCK=13` (bit-banged MAX6675 style).  
- **Actuators**: `SOLENOID=2`, `M1_IN1=3`, `M1_IN2=4`, `M2_IN1=5`, `M2_IN2=6`, `ENA=7`, `ENB=8`.  
- Modes: `OFF, STANDBY, INIT, FROTH, CLEAN`. ABORT → **STANDBY drive posture**; OFF → Hi-Z outputs and status=OFF.

### Nano Every — Scale Module
- HX711: `DOUT=A2`, `SCK=A3`  
- Button (tare/cal): `A1` (active-LOW, PULLUP)  
- 7-segment: `DIGIT` 2,3,4 (active HIGH), `SEG` 5..12 (active HIGH)  
- LEDs: **Green=RX(0)**, **Red=TX(1)**  
- I²C address: **0x2A** (Milk) or **0x2B** (Sauce) — set `I2C_ADDR` accordingly.

### Nano Every — Modbus Master (reference)
- MAX485: **RO→D0(RX1)**, **DI→D1(TX1)**, **RE+DE→D2** (HIGH=TX, LOW=RX)  
- Bus: **19200 8N1**.  
- USB CLI @ **115200** with non-blocking background polling.

> ⚠️ Use correct RS-485 termination (120 Ω) and biasing per your panel standard.

---

## Build & Flash
> Arduino IDE 2.x (or CLI). No external HX711 library is required.

**Dispenser (Mega 2560)**  
1) Board: **Arduino Mega or Mega 2560**.  
2) Verify `MODBUS_SERIAL=Serial1`, `MODBUS_BAUD=19200`, `MODBUS_SLAVE_ID=1`.  
3) Flash. Open Serial Monitor @ **115200** for CLI/events.

**Cleaner (Nano Every)**  
1) Confirm `MODBUS_BAUD=19200`, `MODBUS_SLAVE_ID=2`.  
2) Flash. Open Serial Monitor @ **115200** to watch events.

**Scale Module(s) (Nano Every)**  
1) Set `I2C_ADDR` to **0x2A** (Milk) or **0x2B** (Sauce).  
2) Flash. On boot: auto-tare (`SEt`) → **READING**.

**Master (Nano Every)**  
1) Wire MAX485 as above; flash the provided master sketch.  
2) Open USB Serial @ **115200**.

---

## Commissioning
1. Power the **scale modules first** so they auto-tare, then power slaves.  
2. Calibrate each scale: long-press ≥3 s → **CAL**, place known mass (default 500 g), short-press to save (32-sample average).  
3. Verify RS-485 wiring/termination; watch each slave’s ready banner.  
4. From the master CLI:  
   - `d.status` and `c.status` → expect valid Modbus replies.  
   - Dispenser dry test: `d.trigger sauce2 1`.  
   - Cleaner dry test: `c.init 5` then `c.abort` (expect `ABORTED_FAULT`).  
5. Wet test (Dispenser): e.g. `d.dispense m1 120 50 20 45`.

---

## Operation
### Dispenser behavior
- **Dispense profile**: starts at **24 V (High)**, auto-drops to **12 V (Low)** when weight reaches `slowPct%` of target. Optional soft-cut stops the motor early, lets the stream coast, then checks target. Hard-cut if target is reached first.  
- **Parameters (DISPENSE)**:  
  - `target_g` (0 < g ≤ 999).  
  - `slowPct` (`0..100`): % of target when to switch to Low.  
  - `softCutPct` (`0..100`): maps to up to **1.5 g** early stop. `0` = ~1.5 g before target; `100` = cut at target (no soft-cut).  
  - `timeout_s`: main timeout; if hit, the controller forces Low and gives an extra **3 s retry** before latching `TIMEOUT_FAULT`.  
- **Scale selection**: automatically routes to **0x2A (Milk)** or **0x2B (Sauce)** based on motor ID.  
- **Rinse**: solenoid on for N seconds (blocked during dispense).  
- **Trigger**: diagnostic pin drive for N seconds.  
- **Startup delay**: 2 s motor-start guard is applied before the motor energizes.

### Cleaner behavior
- **INIT**: drive/prime for `seconds_x10`.  
- **FROTH**: heat until `targetC` (°C) or timeout; uses MAX6675-style thermocouple.  
- **CLEAN**: Phase0 Valve → Phase1 Steam → Phase2 Standby; all non-blocking.  
- **ABORT**: immediate preemption; sets **ABORTED_FAULT** but posture is **STANDBY drive** (not OFF).  
- **OFF**: true Hi-Z posture with `systemStatus=OFF`.

### Fault model
- Latched until `CLEAR`: `MOTOR_FAULT`, `LEAK_FAULT`, `SCALE_FAULT`, `TIMEOUT_FAULT`, `ABORTED_FAULT`.  
- Cleaner only produces `TIMEOUT_FAULT` and `ABORTED_FAULT`, plus `OFF` as a distinct status.  
- Dispenser forces outputs safe on any fault or abort.

### Logging conventions
- Start commands log exactly one `[sent] ...` line.  
- While waiting: **silent** background polling.  
- Final outcome: one line, e.g. `[dispense] done.` or `[froth] ended: TIMEOUT_FAULT`.

---

## Scale firmware (UI & I²C)
- **Modes**: `IDLE` (blank), `READING`, `CAL`, overflow `HI`. Auto-IDLE after 60 s without ≥1 g movement.  
- **Tare/Cal**:  
  - Auto-tare at startup (`SEt`).  
  - Short press = tare (`SEt`).  
  - Long press (≥3 s) = enter **CAL** with a silent tare; short press stores new scale factor to EEPROM.  
- **I²C**:  
  - **Write**: `'R' + uint16 target_x10` (LE) → tare + arm target + clear Red LED.  
  - **Read**: `[hit_once, weight_x10_lo, weight_x10_hi]`; `hit_once` clears after one read.  
- Red LED latches when target is reached; Green LED indicates READING.

---

## Modbus RTU API
Link: **19200 8N1**, **Function 0x17 only**. Shared result layout `0x0100..0x010A`.

### Common register map
**Command Block @ `0x0000`**

| Addr  | Field   | Notes |
|:-----:|:--------|:------|
| `0000` | `opcode` | Per device (below) |
| `0001` | `seq` | Echo (master may write `0`) |
| `0002..0007` | `args` | Per-opcode |

**Result Block @ `0x0100`**

| Addr  | Field | Meaning |
|:-----:|:------|:--------|
| `0100` | `resultCode` | `0=OK`, `1=FAIL` |
| `0101` | `errorCode` | See table below |
| `0102` | `systemStatus` | See table below |
| `0103` | `seq` | Echo |
| `0104` | aux0 | Dispenser: `lastWeight_x10` (g×10) · Cleaner: `tempC_x10` |
| `0105` | aux1 | `elapsed_s_x10` (set by ops that care) |
| `0106` | aux2 | Dispenser: `activeScaleId` (1=Milk, 2=Sauce); Cleaner: `0` |
| `0107..010A` | reserved | `0` |

### Dispenser opcodes (ID=1)
- `STATUS(1)` — no args.  
- `CLEAR(2)` — clears latched faults and resets outputs.  
- `ABORT(3)` — immediate safe-stop → `ABORTED_FAULT`.  
- `DISPENSE(10)` — args: `motorId`, `target_x10`, `slowPct_x10`, `softCutPct_x10`, `timeout_s`.  
- `RINSE(11)` — args: `seconds_x10` (blocked during dispense).  
- `TRIGGER(12)` — args: `motorId`, `seconds_x10` (diagnostic pin drive).

### Cleaner opcodes (ID=2)
- `STATUS(1)` — no args.  
- `CLEAR(2)` — clears latched faults, returns to `IDLE/STANDBY`.  
- `ABORT(3)` — aborts current phase → `ABORTED_FAULT` + STANDBY posture.  
- `OFF(4)` — Hi-Z outputs; `systemStatus=OFF`.  
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
- **Milk**: IDs `11..18` — tokens: `Milk1..Milk8`, `m1..m8`.  
- **Sauce**: IDs `21..35` — tokens: `Sauce1..Sauce15`, `s1..s15`.  
- Numeric IDs are accepted by Modbus and the master CLI.

---

## Reference Master CLI
USB Serial @ **115200**. The master is non-blocking: start commands log `[sent]`, background-polls STATUS, and prints one final outcome.

**Dispenser (`d.*`)**
```
d.status
d.clear
d.abort
d.rinse 2.0
d.trigger Sauce2 1.0
# d.dispense <motor|name> <target_g> <slowPct0-100> <softCutPct0-100> <timeout_s>
d.dispense Sauce2 8 50 20 20
d.dispense Milk1 5 30 0 20
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

Example outcomes:
```
result=OK  error=NONE  status=ACTIVE
[dispense] done.
[froth] ended: TIMEOUT_FAULT
```

---

## Troubleshooting
- **No Modbus response / CRC** — verify RE/DE wiring (pin 17 on the Dispenser, A1/A2 on the Cleaner), **19200 8N1**, correct Slave ID, and 120 Ω termination with bias.  
- **BUSY** — an op is running. Use `...abort`, then `...clear`.  
- **SCALE_FAULT** — check I²C wiring/address or HX711 power on the selected scale module.  
- **LEAK/MOTOR faults** — inspect sensors/driver; faults are **latched** until `CLEAR`.  
- **Cleaner OFF** — use `c.status` to confirm; `c.init`/`c.froth`/`c.clean` drive it back to `ACTIVE` when accepted.
