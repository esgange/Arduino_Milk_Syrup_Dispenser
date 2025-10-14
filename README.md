# Weigh‑Scale Feedback Liquid Dispenser

**Motor Controller (Mega 2560) v1.1 — 2025‑10‑12**  \
**Scale Slave (Nano Every + HX711) v1.0 — 2025‑10‑10**  \
**Reference Modbus RTU Master (Nano Every)**

Authors: **Erdie Gange · ChatGPT 5** · Region: **Asia/Riyadh**

---

> One‑shot liquid dispenser with weigh‑scale feedback, latched leak/motor/timeout faults, and a minimalist **Modbus RTU (0x17)** slave API. Includes a ready‑to‑use Nano Every master sketch as a reference client/CLI.

## Contents
- [Overview](#overview)
- [Hardware](#hardware)
  - [Mega 2560 — Controller](#mega-2560--controller)
  - [Nano Every — Scale Module](#nano-every--scale-module)
  - [Nano Every — Modbus Master (reference)](#nano-every--modbus-master-reference)
- [Build & Flash](#build--flash)
- [Commissioning](#commissioning)
- [Operation](#operation)
  - [Dispense profile](#dispense-profile)
  - [Fault model](#fault-model)
  - [Active scale selection](#active-scale-selection)
- [Scale firmware (UI & I²C)](#scale-firmware-ui--i²c)
- [Modbus RTU API](#modbus-rtu-api)
  - [Register map](#register-map)
  - [Opcodes](#opcodes)
  - [Motor IDs](#motor-ids)
- [Reference Master CLI](#reference-master-cli)
- [Troubleshooting](#troubleshooting)
- [Version history](#version-history)

---

## Overview
The Mega‑based controller drives per‑flavor motors and the rinse solenoid while reading live mass from a dedicated I²C scale (Nano Every + HX711). It exposes a **synchronous Modbus RTU 0x17** API where each command is **executed to completion within a single transaction**. All runtime faults are **latched** until a `CLEAR`.

**Key operations**: `STATUS`, `CLEAR`, `ABORT`, `DISPENSE`, `RINSE`, `TRIGGER`.

**Speed stages**: start **High (24 V)** then drop to **Low (12 V)** before soft‑cut for accuracy.

---

## Hardware
### Mega 2560 — Controller
- **RS‑485**: `DE→A2`, `RE→A3` (RE is **active‑LOW**). Bus: **Serial3 @ 19200 8N1**, **Slave ID = 1**.
- **Motors**  
  Milk pins **2..9** (8 ch); Sauce pins **10,11,12,25..47 odd** (15 ch).
- **Speed select**: `PIN_SPEEDSEL=23` (LOW → **24 V High**, HIGH → **12 V Low**). Default after reset: **High**.
- **Rinse**: pin **49**.  
- **Motor fault in**: pin **51** (**active‑LOW**, 1 s debounce).  
- **Leak sensors** (LMV331): `(A5/A6)`, `(A8/A9)`; `A6/A9` power, `A5/A8` logic input.  
- **I²C scales**: Milk = **0x2A**, Sauce = **0x2B** *(shop standard)*.

### Nano Every — Scale Module
- HX711: `DOUT=A2`, `SCK=A3`  
- Button (tare/cal): `A1` (active‑LOW, PULLUP)  
- 7‑segment: `DIGIT` 2,3,4 (ACTIVE HIGH), `SEG` 5..12 (ACTIVE HIGH)  
- LEDs: **Green=RX(0)**, **Red=TX(1)**  
- I²C address: **0x2B** (Sauce) or **0x2A** (Milk)

### Nano Every — Modbus Master (reference)
- MAX485: **RO→D0(RX1)**, **DI→D1(TX1)**, **RE+DE→D2** (HIGH=TX, LOW=RX)  
- Bus: **19200 8N1**. Provides a USB CLI to drive the dispenser via Modbus.

> ⚠️ Use proper RS‑485 termination (120 Ω) and biasing as per your panel standard.

---

## Build & Flash
> Arduino IDE 2.x (or CLI). No external HX711 library is required.

**Controller (Mega 2560)**
1. Select **Arduino Mega or Mega 2560**.  
2. Verify `MODBUS_BAUD=19200`, `MODBUS_SERIAL=Serial3`, `MODBUS_SLAVE_ID=1`.  
3. Flash. Open Serial Monitor @ **115200** for bench commands (`help`, `status`, etc.).

**Scale(s) (Nano Every)**
1. Set `I2C_ADDR` to **0x2B** (Sauce) or **0x2A** (Milk). Optionally set `CAL_WEIGHT_G` to your reference (e.g., 500.0 g).  
2. Flash. On boot, module shows **SEt** (startup tare) then enters **READING**.

**Master (Nano Every)**
1. Wire MAX485 per above; flash the provided master sketch.  
2. Open USB Serial @ **115200**.

---

## Commissioning
1. Verify wiring: motors, solenoid (49), speed select (23), leak pairs (A5/A6 & A8/A9), RS‑485, I²C to scales.  
2. Power **scales** first → auto‑tare (display **SEt**), then live weight.
3. **Calibrate each scale**: long‑press ≥3 s → **CAL** → place known mass (default 500.0 g) → short‑press to save (EEPROM) → returns to **READING**.
4. Power controller (Mega); console prints ready banner.  
5. From Master, run `status` → expect valid Modbus response.  
6. Dry test: `trigger sauce2 1.0` to pulse a channel.  
7. Wet test: `dispense m1 120 10 2 45` and observe slow‑down/soft‑cut and final mass.

---

## Operation
### Dispense profile
- Start at **24 V High**, drop to **12 V Low** when `weight ≥ target − slowOffset`.  
- **Soft‑cut**: motor **OFF** at `target − softCutOffset`, then **settle 150 ms** and **coast**.  
- **Hard‑cut**: if target reached before soft‑cut, stop immediately.  
- No top‑off pulses.  
- **Timeout** per dispense (cap 10 min). On timeout: outputs safe + **latched** fault.

### Fault model (latched until `CLEAR`)
- `MOTOR_FAULT` (pin 51 low > 1 s)  
- `LEAK_FAULT` (binary sensors, 1 s debounce)  
- `SCALE_FAULT` (I²C failure/arm fail)  
- `TIMEOUT_FAULT`  
- `ABORTED` (software E‑stop)

### Active scale selection
Motor pin decides scale automatically: **Milk pins → 0x2A**, **Sauce pins → 0x2B**. Final mass exposed as `lastWeight_x10` (g×10).

---

## Scale firmware (UI & I²C)
**Modes**:  
- **IDLE** (blank, Green OFF after 60 s inactivity)  
- **READING** (<100 g → `XX.X`, ≥100 g → `XXX`; Green ON)  
- **CAL** (shows `CAL`)  
- **Overflow**: if >999 g actual → shows `HI` (I²C clamps to 999.0)

**Tare/Cal**:  
- Startup auto‑tare (shows `SEt`); short press = tare (also clears Red + hit flag); long press = enter **CAL** (silent tare at entry), short press to save scale to EEPROM.

**I²C API**:  
- **Write**: `'R' + uint16 target_x10` (little‑endian) → tare (`SEt`) + arm target.  
- **Read**: 3 bytes `[hit_once, weight_x10_lo, weight_x10_hi]`. Red LED **latches** when target is hit; cleared by next tare.

---

## Modbus RTU API
**Link**: Serial3, **19200 8N1**, **Slave ID = 1**. Only **Function 0x17 Read/Write Multiple Registers** is accepted. Execution is **synchronous**: command runs to completion within the 0x17 transaction, then the reply returns the **Result Block**.

### Register map
**Command Block @ `0x0000`**

| Addr  | Field   | Notes |
|:-----:|:--------|:------|
| `0000` | `opcode` | `1=STATUS`, `2=CLEAR`, `3=ABORT`, `10=DISPENSE`, `11=RINSE`, `12=TRIGGER` |
| `0001` | `seq` | Echo; master may write `0` |
| `0002..0007` | `args` | Per‑opcode (see below) |

**Result Block @ `0x0100`**

| Addr  | Field | Meaning |
|:-----:|:------|:--------|
| `0100` | `resultCode` | `0=OK`, `1=FAIL` |
| `0101` | `errorCode` | `0=NONE,1=BUSY,2=MOTOR,3=LEAK,4=SCALE,5=TIMEOUT,6=BAD_ARGS,7=INVALID_CMD,8=ABORTED` |
| `0102` | `systemStatus` | `0=IDLE,1=ACTIVE,2=MOTOR_FAULT,3=LEAK_FAULT,4=SCALE_FAULT,5=TIMEOUT_FAULT,6=ABORTED_FAULT` |
| `0103` | `seq` | Echo |
| `0104` | `lastWeight_x10` | grams×10 |
| `0105` | `elapsed_s_x10` | 0.1 s units |
| `0106` | `activeScaleId` | `1=MILK`, `2=SAUCE` |
| `0107..010A` | reserved | `0` |

### Opcodes
- **STATUS (1)** — no args. Always `OK`; inspect `systemStatus`, `lastWeight_x10`.
- **CLEAR (2)** — no args. Resets **latched** faults → `IDLE`.
- **ABORT (3)** — no args. Immediately safe‑stops and **latches** `ABORTED`; reply is `FAIL/ABORTED`. Use `CLEAR` to resume.
- **RINSE (11)** — args: `0x0002 = seconds_x10`. Busy if dispensing/triggering.
- **TRIGGER (12)** — args: `0x0002 = motorId`, `0x0003 = seconds_x10`. Busy if dispensing or rinse active.
- **DISPENSE (10)** — args:  
  `0x0002 = motorId`  
  `0x0003 = target_x10` (g×10)  
  `0x0004 = slowOffset_x10` (g×10)  
  `0x0005 = softCutOffset_x10` (g×10)  
  `0x0006 = timeout_s`

> **Busy/Abort caveat**: Because 0x17 is synchronous, the master cannot send a separate `ABORT` frame until the current transaction completes. `ABORT` preempts only when used **as the current opcode**, and it blocks future commands until `CLEAR`.

### Motor IDs
- **Milk**: `1..8` (tokens: `milk1..milk8`, `m1..m8`)  
- **Sauce**: `101..115` (tokens: `sauce1..sauce15`, `s1..s15`)

---

## Reference Master CLI
Open the master’s USB serial @ **115200** and type commands. These send 0x17 frames to the controller and print the parsed **Result Block**.

```
status
clear
rinse 2.0
trigger sauce2 1.0
# dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>
dispense m1 120 10 2 45
```

Example response line:
```
result=OK  error=NONE
```
Or, on error:
```
result=FAIL  error=LEAK_FAULT
```

---

## Troubleshooting
**No Modbus response / CRC mismatch**  
- Check RS‑485 wiring and RE/DE direction control on the master (D2).  
- Settings: **19200 8N1**, **Slave ID = 1**.  
- Termination/biasing present; idle gap ≈ **t3.5 ≈ 2.0 ms**.

**BUSY**  
- Another command is running (`DISPENSE`, `RINSE`, `TRIGGER`). Use **`ABORT`** as the current command to latch `ABORTED`, then `CLEAR`.

**SCALE_FAULT**  
- I²C wiring/address or HX711 power. Each dispense *arms* the active scale; if the initial write fails, the controller latches `SCALE_FAULT`.

**LEAK_FAULT**  
- Moisture detected or input stuck HIGH; dry/repair, then `CLEAR`.

**MOTOR_FAULT**  
- Pin 51 low >1 s; check driver/load/wiring.

**TIMEOUT_FAULT**  
- Flow too slow/obstruction; revisit offsets or increase `timeout_s`.

**Scale shows `HI`**  
- Actual >999 g; for drift/container change, perform a **tare** (short press or issue I²C `'R'`).

---

## Version history
- **Controller v1.1** — All faults **latched**; new **ABORT** command (`ABORTED` state); clarified synchronous 0x17 execution.  
- **Scale v1.0** — I²C `'R'+target_x10` with auto‑tare; `HI` overflow; auto‑idle; EEPROM‑persisted scale.

