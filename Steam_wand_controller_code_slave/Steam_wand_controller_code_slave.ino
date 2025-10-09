/*
========================================
README — Steam Wand Module (Modbus RTU Slave, no libraries)
Target MCU: Arduino Nano Every (megaAVR 4809)
========================================

Overview
--------
This sketch turns an Arduino Nano Every into a Modbus RTU **slave** that controls a
steam-wand subsystem over RS-485 (MAX485). It implements the Modbus frame parsing,
timing, and CRC **manually** (no Modbus libraries), plus a bit-banged MAX6675
thermocouple reader. A small set of holding registers exposes commands and status so
an external PLC/HMI/PC can orchestrate the wand.

Key features
- Modbus RTU on `Serial1` @ 9600 baud, **8E1** (configurable).
- Function codes: **0x03 (Read Holding Registers)**, **0x06 (Write Single Reg)**, **0x10 (Write Multiple Regs)**.
- Respect for broadcast address **0** (write executes, **no** response).
- Implements **Slave Device Busy** exception (0x06) when a non-OFF command arrives mid-run.
- Temperature via **MAX6675**, paced about every 220 ms, with cached value for responsiveness.
- Local USB debug (optional): `off`, `status`, `standby`, `init s`, `froth T timeout`, `clean a b c`.

Hardware & Wiring
-----------------
Board: Arduino Nano Every (ATmega4809)

RS-485 Transceiver (MAX485):
- RE  -> **A1** (active LOW: 0 = RX enabled)
- DE  -> **A2** (active HIGH: 1 = TX enabled)
- RO  -> **Serial1 RX**
- DI  -> **Serial1 TX**

Actuators / Sensors:
- Solenoid        : **D2**  (USED ONLY IN CLEAN MODE)
- Motor M1 Cleaner: **IN1=D3**, **IN2=D4**   (STANDBY = REVERSE)
- Motor M2 Control: **IN1=D5**, **IN2=D6**   (STANDBY = FORWARD)
- MAX6675 (bit-banged): **SO=D12**, **CS#=D10**, **SCK=D13**

RS-485 notes:
- Add a 120 Ω termination resistor across A/B at the bus ends if needed.
- Tie RE/DE as shown so the sketch can switch TX/RX direction.
- Use common ground between MCU and transceiver.

Build & Configuration
---------------------
Edit these as needed near the top of the file:
- `SLAVE_ID`      : Modbus slave address (default **1**).
- `MODBUS_BAUD`   : default **9600**.
- `MODBUS_CONFIG` : default **SERIAL_8E1** (8 data bits, even parity, 1 stop).

Runtime Modes / Rules
---------------------
- `OFF` is the only mid-run interrupt: writing CMD=0 sets an abort flag and stops.
- `STANDBY`: M1=Reverse, M2=Forward, Solenoid=Off.
- `INIT(seconds)`: drives M2 reverse for the given time, then goes to STANDBY.
- `FROTH(target °C, timeout s)`: M2 reverse until target reached, timeout, or TC fault; ends in STANDBY.

- **CLEAN(tValve, tSteam, tStandby) — updated logic**
  1) Start **M1 forward** immediately.
  2) After **tValve**: **Solenoid ON**.
  3) After **tSteam**: **M2 reverse ON** **and simultaneously Solenoid OFF** (new requirement).
  4) After **tStandby**: transition to **STANDBY** (M1=Rev, M2=Fwd, Solenoid=Off).

Error Mask bits (READ register `ERROR_MASK`)
- bit0 = thermocouple fault (open)
- bit1 = aborted (OFF)
- bit2 = timeout (FROTH only)

Modbus Register Map (Holding Registers, 16-bit)
-----------------------------------------------
WRITE (master → slave)
- `0000` **CMD**: 0=OFF, 1=STANDBY, 2=INIT, 3=FROTH, 4=CLEAN
- `0001` **INIT_SECSx100**
- `0002` **FROTH_TARGETCx100**
- `0003` **FROTH_TIMEOUT_SECSx100**
- `0004` **CLEAN_T_VALVE_SECSx100**   (delay before Solenoid ON)
- `0005` **CLEAN_T_STEAM_SECSx100**   (delay before M2 Reverse; also when Solenoid turns OFF)
- `0006` **CLEAN_T_STANDBY_SECSx100** (delay before returning to STANDBY)

READ (slave → master)
- `0010` **MODE**: 0=OFF,1=STANDBY,2=INIT,3=FROTH,4=CLEAN
- `0011` **BUSY**: 0/1
- `0012` **TEMP_Cx100**: signed; **-32768** if invalid
- `0013` **ERROR_MASK**
- `0014` RESERVED

Data scaling:
- Times are in seconds with 2 decimals → **x100** integer.
- Temperature in °C with 2 decimals → **x100** signed integer.

Supported Function Codes & Exceptions
-------------------------------------
- **0x03** Read Holding Registers
- **0x06** Write Single Register
- **0x10** Write Multiple Registers
- Exceptions used:
  - 0x01 Illegal Function
  - 0x02 Illegal Data Address
  - 0x03 Illegal Data Value
  - 0x06 Slave Device Busy (non-OFF CMD while running)

Busy & Broadcast Behavior
-------------------------
- If the module is executing a mode (BUSY=1), any non-OFF `CMD` write returns exception **0x06**.
- Broadcast (address **0**): writes are executed but **no response** is sent (per Modbus spec).

Timing & MAX6675
----------------
- MAX6675 conversion ~220 ms; the sketch paces reads and caches the most recent value.
- The status registers are refreshed regularly for responsive polling.
- Modbus inter-frame gap detection uses **3.5 character times** at the configured baud.

USB Debug (optional)
--------------------
Open the USB serial monitor at **115200** baud:
- `off`          → aborts and goes OFF
- `status`       → prints Mode/Busy/Temp/ErrorMask
- `standby`
- `init s`
- `froth T timeout`
- `clean tValve tSteam tStandby`

Quick Test (example with 5.00 s each)
-------------------------------------
Write 500 to regs 4, 5, and 6 (x100 scaling). Sequence:
1) t=0s: M1 forward
2) t=5s: Solenoid ON
3) t=10s: M2 reverse ON **and** Solenoid OFF
4) t=15s: STANDBY

Safety
------
Steam and hot surfaces can cause burns. Validate wiring and mechanics with
LOW-RISK dry runs (solenoid off, motors disconnected) before introducing heat/steam.
Use proper fusing and isolation. You are responsible for safety-critical interlocks.

Notes
-----
- Defaults are set in `setup()` for convenience.
- Adjust `SLAVE_ID`, baud, and parity to match your master.
- The implementation avoids external libraries to keep timing/behavior explicit.
*/


/*
  Steam Wand Module — Modbus RTU Slave (NO LIBRARIES)
  Board: Arduino Nano Every (megaAVR 4809)

  RS-485 Transceiver (MAX485):
    - RE  -> A1 (active LOW: 0 = RX enabled)
    - DE  -> A2 (active HIGH: 1 = TX enabled)
    - RO  -> Serial1 RX
    - DI  -> Serial1 TX
  Modbus: Serial1 @ 9600 baud, 8E1 (standard RTU), slave ID configurable.

  Local hardware (from previous sketches):
    - Solenoid  : D2 (USED ONLY IN CLEAN MODE)
    - M1 Cleaner: IN1=D3, IN2=D4  (STANDBY = REVERSE)
    - M2 Control: IN1=D5, IN2=D6  (STANDBY = FORWARD)
    - MAX6675   : SO=D12, CS#=D10, SCK=D13 (bit-banged, no libs)

  Modes / Rules:
    - 'off' is the only mid-run interrupt (sets abort).
    - 'status' available locally via USB Serial (optional).
    - Froth: on target or timeout or TC fault -> STANDBY.
    - Clean: ends in STANDBY (M1=Rev, M2=Fwd, Solenoid=Off).

  Modbus map (Holding Registers, 16-bit):
    WRITE (master -> slave)
      0000  CMD            0=OFF, 1=STANDBY, 2=INIT, 3=FROTH, 4=CLEAN
      0001  INIT_SECSx100
      0002  FROTH_TARGETCx100
      0003  FROTH_TIMEOUT_SECSx100
      0004  CLEAN_T_VALVE_SECSx100
      0005  CLEAN_T_STEAM_SECSx100
      0006  CLEAN_T_STANDBY_SECSx100

    READ (slave -> master)
      0010  MODE           0=OFF,1=STANDBY,2=INIT,3=FROTH,4=CLEAN
      0011  BUSY           0/1
      0012  TEMP_Cx100     (signed; -32768 if invalid)
      0013  ERROR_MASK     bit0=TC fault, bit1=aborted(off), bit2=timeout
      0014  RESERVED

  Supported function codes:
    03 Read Holding Registers
    06 Write Single Register
    10 Write Multiple Registers
  Busy behavior:
    - Non-off CMD while running -> Modbus exception 06 (Slave Device Busy)
    - Illegal address/value -> exception 02/03 respectively
    - Broadcast (address 0): writes execute, no response frame (per Modbus).

  Notes:
    - Times are in seconds (2 decimals); stored as x100 integers.
    - MAX6675 paced ~220 ms; cached temp used for responsiveness.
*/

#include <Arduino.h>

// ====== USER CONFIG ======
#define SLAVE_ID        1
#define MODBUS_BAUD     9600
#define MODBUS_CONFIG   SERIAL_8E1   // typical RTU: 8E1 or 8N2 also common

// RS-485 direction control pins
const uint8_t RS485_RE_PIN = A1;  // RE: LOW = receive enable
const uint8_t RS485_DE_PIN = A2;  // DE: HIGH = driver enable

// Use Serial1 for RS-485 Modbus
#define MODBUS_PORT Serial1

// ====== Application Pins (actuators / sensors) ======
const uint8_t SOLENOID_PIN = 2;
const uint8_t M1_IN1 = 3;
const uint8_t M1_IN2 = 4;
const uint8_t M2_IN1 = 5;
const uint8_t M2_IN2 = 6;
// MAX6675
const uint8_t TC_SO  = 12;
const uint8_t TC_CS  = 10;
const uint8_t TC_SCK = 13;

// ====== App State ======
enum Mode : uint8_t { MODE_OFF, MODE_STANDBY, MODE_INIT, MODE_FROTH, MODE_CLEAN };
volatile bool g_isBusy = false;
volatile bool g_abort  = false;
Mode g_currentMode     = MODE_OFF;

static unsigned long g_lastTcReadMs = 0;
static float g_tempC   = NAN;
static bool  g_tempOk  = false;
static uint16_t g_errorMask = 0;   // bit0=TCfault, bit1=aborted, bit2=timeout

// ====== L298 helpers ======
inline void mStop(uint8_t in1, uint8_t in2){ digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
inline void mFwd (uint8_t in1, uint8_t in2){ digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
inline void mRev (uint8_t in1, uint8_t in2){ digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
inline void M1_Stop(){ mStop(M1_IN1, M1_IN2); }
inline void M1_Fwd (){ mFwd (M1_IN1, M1_IN2); }
inline void M1_Rev (){ mRev (M1_IN1, M1_IN2); }
inline void M2_Stop(){ mStop(M2_IN1, M2_IN2); }
inline void M2_Fwd (){ mFwd (M2_IN1, M2_IN2); }
inline void M2_Rev (){ mRev (M2_IN1, M2_IN2); }

inline void solenoidOn()  { digitalWrite(SOLENOID_PIN, HIGH); }
inline void solenoidOff() { digitalWrite(SOLENOID_PIN, LOW);  }

unsigned long secToMs(float s){ if (s < 0) s = 0; return (unsigned long)(s*1000.0f + 0.5f); }

// ====== MAX6675 (bit-banged) ======
uint16_t tc_readRaw(){
  uint16_t v=0;
  digitalWrite(TC_CS, LOW);
  delayMicroseconds(2);
  digitalWrite(TC_SCK, LOW);
  delayMicroseconds(1);
  for(int i=0;i<16;++i){
    digitalWrite(TC_SCK, HIGH);
    delayMicroseconds(1);
    v <<= 1;
    if (digitalRead(TC_SO)) v |= 1;
    digitalWrite(TC_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(TC_CS, HIGH);
  return v;
}
bool tc_openFault(uint16_t raw){ return (raw & 0x0004); }
bool tc_readC_blocking(float &c){
  unsigned long now = millis();
  if (now - g_lastTcReadMs < 220UL) delay(220UL - (now - g_lastTcReadMs));
  g_lastTcReadMs = millis();
  uint16_t raw = tc_readRaw();
  if (tc_openFault(raw)){
    g_tempOk=false; g_tempC=NAN; return false;
  }
  uint16_t data = (raw>>3)&0x0FFF;
  c = data * 0.25f;
  g_tempOk=true; g_tempC=c;
  return true;
}
bool tc_tryReadC_nonblocking(float &c){
  unsigned long now = millis();
  if (now - g_lastTcReadMs >= 220UL){
    g_lastTcReadMs = now;
    uint16_t raw = tc_readRaw();
    if (!tc_openFault(raw)){
      g_tempC = ((raw>>3)&0x0FFF) * 0.25f;
      g_tempOk = true;
    } else { g_tempOk=false; g_tempC=NAN; }
  }
  c = g_tempC;
  return g_tempOk;
}

// ====== State helpers ======
void toOff(){ M1_Stop(); M2_Stop(); solenoidOff(); g_currentMode=MODE_OFF; }
void toStandby(){ M1_Rev(); M2_Fwd(); solenoidOff(); g_currentMode=MODE_STANDBY; }

// ====== Modbus Register Space ======
// We'll keep a small holding register array. Only addresses shown in map are used.
#define HR_COUNT 32
uint16_t HR[HR_COUNT]; // 0..31

// HR address define helpers
#define HR_CMD                 0
#define HR_INIT_SECSX100       1
#define HR_FROTH_TARGETX100    2
#define HR_FROTH_TOUTX100      3
#define HR_CLEAN_TVALVEX100    4
#define HR_CLEAN_TSTEAMX100    5
#define HR_CLEAN_TSTDBYX100    6

#define HR_MODE                10
#define HR_BUSY                11
#define HR_TEMPX100            12  // signed (casted)
#define HR_ERRORMASK           13

// ====== Modbus low-level ======
inline void rs485ToRX(){ digitalWrite(RS485_DE_PIN, LOW); digitalWrite(RS485_RE_PIN, LOW); }   // listen
inline void rs485ToTX(){ digitalWrite(RS485_RE_PIN, HIGH); digitalWrite(RS485_DE_PIN, HIGH); } // drive

uint16_t mb_crc16(const uint8_t* buf, uint16_t len){
  uint16_t crc=0xFFFF;
  for(uint16_t pos=0; pos<len; pos++){
    crc ^= buf[pos];
    for(uint8_t i=0;i<8;i++){
      if (crc & 0x0001) { crc >>=1; crc ^= 0xA001; }
      else crc >>=1;
    }
  }
  return crc;
}

void mb_send(const uint8_t* data, uint16_t len){
  rs485ToTX();
  MODBUS_PORT.write(data, len);
  MODBUS_PORT.flush();     // wait for full transmission
  rs485ToRX();
}

// Build & send normal response
void mb_send_ok(const uint8_t addr, const uint8_t* pdu, uint16_t pdulen){
  // RTU: [addr][pdu...][CRCLo][CRCHi]
  uint8_t frame[260];
  frame[0]=addr;
  memcpy(frame+1, pdu, pdulen);
  uint16_t crc = mb_crc16(frame, pdulen+1);
  frame[pdulen+1] = crc & 0xFF;
  frame[pdulen+2] = (crc>>8) & 0xFF;
  mb_send(frame, pdulen+3);
}

// Build & send exception
void mb_send_exc(const uint8_t addr, uint8_t func, uint8_t code){
  uint8_t resp[3];
  resp[0] = func | 0x80;
  resp[1] = code;
  mb_send_ok(addr, resp, 2);
}

// Addresses and timing
unsigned long charUs(){
  // 8E1 => 11 bits per char; 8N1 is ~10 bits; use 11 to be safe
  return (unsigned long)( (1000000.0 * 11.0) / MODBUS_BAUD );
}
unsigned long T3_5_us(){ return (unsigned long)(charUs() * 3.5 + 0.5); }

// ====== Modbus functional handlers ======
bool app_isBusy(){ return g_isBusy; }

// convert scaled regs to float seconds/C
float regToSec(uint16_t x){ return ((int32_t)x)/100.0f; }
float regToC  (uint16_t x){ return ((int32_t)x)/100.0f; }
int16_t floatC_to_i16x100(float c){
  long v = lroundf(c*100.0f);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

// Update read-only status registers
void updateStatusRegs(){
  HR[HR_MODE] = (uint16_t)g_currentMode;
  HR[HR_BUSY] = g_isBusy ? 1 : 0;

  float c;
  bool ok = tc_tryReadC_nonblocking(c);
  HR[HR_TEMPX100] = ok ? (uint16_t)(int16_t)floatC_to_i16x100(c) : (uint16_t)(int16_t)-32768;
  HR[HR_ERRORMASK] = g_errorMask;
}

// Execute command written to HR_CMD
// Returns true if accepted/executed (or queued), false if rejected due to busy (non-off)
bool executeCommandFromReg(){
  uint16_t cmd = HR[HR_CMD];
  switch (cmd){
    case 0: // OFF (always accepted)
      g_abort = true;
      toOff();
      return true;
    case 1: // STANDBY
      if (app_isBusy()) return false;
      toStandby();
      return true;
    case 2: // INIT
      if (app_isBusy()) return false;
      // will be started by dispatcher (loop) after we exit this handler
      return true;
    case 3: // FROTH
      if (app_isBusy()) return false;
      return true;
    case 4: // CLEAN
      if (app_isBusy()) return false;
      return true;
    default:
      return false; // illegal value
  }
}

// Minimal address-range check helper for our HR[]
bool hr_inRange(uint16_t addr, uint16_t qty){
  return (addr + qty) <= HR_COUNT;
}

// Process a complete RTU frame in rxBuf[rxLen]
void modbus_process(uint8_t* rxBuf, uint16_t rxLen){
  if (rxLen < 5) return; // too short
  uint16_t crcCalc = mb_crc16(rxBuf, rxLen-2);
  uint16_t crcRx   = rxBuf[rxLen-2] | (rxBuf[rxLen-1]<<8);
  if (crcCalc != crcRx) return; // bad CRC, ignore

  uint8_t addr = rxBuf[0];
  bool isBroadcast = (addr == 0);
  if (!(isBroadcast || addr == SLAVE_ID)) return; // not for us

  uint8_t func = rxBuf[1];

  // ---- FC=0x03: Read Holding Registers ----
  if (func == 0x03){
    if (rxLen != 8) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
    uint16_t start = (rxBuf[2]<<8) | rxBuf[3];
    uint16_t qty   = (rxBuf[4]<<8) | rxBuf[5];
    if (qty == 0 || qty > 125) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
    if (!hr_inRange(start, qty)) { if(!isBroadcast) mb_send_exc(addr, func, 0x02); return; }

    updateStatusRegs();

    if (!isBroadcast){
      uint8_t pdu[255];
      pdu[0] = func;
      pdu[1] = qty*2; // byte count
      uint8_t* p = pdu+2;
      for(uint16_t i=0;i<qty;i++){
        uint16_t v = HR[start+i];
        *p++ = (v>>8)&0xFF;
        *p++ = (v   )&0xFF;
      }
      mb_send_ok(addr, pdu, 2 + qty*2);
    }
    return;
  }

  // ---- FC=0x06: Write Single Register ----
  if (func == 0x06){
    if (rxLen != 8) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
    uint16_t reg = (rxBuf[2]<<8) | rxBuf[3];
    uint16_t val = (rxBuf[4]<<8) | rxBuf[5];

    if (reg >= HR_COUNT) { if(!isBroadcast) mb_send_exc(addr, func, 0x02); return; }

    // Write
    uint16_t old = HR[reg];
    HR[reg] = val;

    // Command register special handling
    if (reg == HR_CMD){
      if (val > 4){ // illegal data value
        HR[reg] = old;
        if(!isBroadcast) mb_send_exc(addr, func, 0x03);
        return;
      }
      bool ok = executeCommandFromReg();
      if (!ok){
        // Busy or illegal
        HR[reg] = old;
        if(!isBroadcast) mb_send_exc(addr, func, app_isBusy() && val!=0 ? 0x06 : 0x03);
        return;
      }
    }

    if (!isBroadcast){
      // Echo write request back as per Modbus 0x06
      uint8_t pdu[5];
      pdu[0]=func; pdu[1]= (reg>>8)&0xFF; pdu[2]=reg&0xFF; pdu[3]=(val>>8)&0xFF; pdu[4]=val&0xFF;
      mb_send_ok(addr, pdu, 5);
    }
    return;
  }

  // ---- FC=0x10: Write Multiple Registers ----
  if (func == 0x10){
    if (rxLen < 9) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
    uint16_t start = (rxBuf[2]<<8) | rxBuf[3];
    uint16_t qty   = (rxBuf[4]<<8) | rxBuf[5];
    uint8_t  bc    = rxBuf[6];
    if (qty == 0 || qty > 123 || bc != qty*2) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
    if (!hr_inRange(start, qty)) { if(!isBroadcast) mb_send_exc(addr, func, 0x02); return; }
    if (rxLen != (7 + 1 + bc + 2)) { if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }

    // Stash old values if we need rollback on error
    uint16_t oldCmd = HR[HR_CMD];
    bool wroteCmd = false;

    // Write payload
    const uint8_t* p = rxBuf + 7;
    for(uint16_t i=0;i<qty;i++){
      uint16_t v = (p[2*i]<<8) | p[2*i+1];
      uint16_t reg = start + i;
      if (reg >= HR_COUNT){ if(!isBroadcast) mb_send_exc(addr, func, 0x02); return; }
      if (reg == HR_CMD){
        if (v > 4){ if(!isBroadcast) mb_send_exc(addr, func, 0x03); return; }
        wroteCmd = true;
      }
      HR[reg] = v;
    }

    // Execute CMD if included
    if (wroteCmd){
      bool ok = executeCommandFromReg();
      if (!ok){
        // restore CMD on failure (busy / illegal)
        HR[HR_CMD] = oldCmd;
        if(!isBroadcast) mb_send_exc(addr, func, app_isBusy() && HR[HR_CMD]!=0 ? 0x06 : 0x03);
        return;
      }
    }

    if (!isBroadcast){
      // Response: echo start & qty
      uint8_t pdu[5];
      pdu[0]=func; pdu[1]=(start>>8)&0xFF; pdu[2]=start&0xFF; pdu[3]=(qty>>8)&0xFF; pdu[4]=qty&0xFF;
      mb_send_ok(addr, pdu, 5);
    }
    return;
  }

  // Unsupported function
  if (!isBroadcast) mb_send_exc(addr, func, 0x01); // Illegal function
}

// ====== Modbus poller (gap-based frame detection) ======
void modbusPoll(){
  static uint8_t  rxBuf[260];
  static uint16_t rxLen = 0;
  static unsigned long lastByteUs = 0;

  // Read any available bytes
  while (MODBUS_PORT.available()){
    int b = MODBUS_PORT.read();
    if (b < 0) break;
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = (uint8_t)b;
    lastByteUs = micros();
  }

  // If we have data and enough silence (3.5 chars), process as a frame
  if (rxLen > 0){
    unsigned long gap = micros() - lastByteUs;
    if (gap >= T3_5_us()){
      modbus_process(rxBuf, rxLen);
      rxLen = 0;
    }
  }
}

// ====== Mode engines (call modbusPoll inside loops for responsiveness) ======
bool pollDuringRun(uint16_t pollMs = 5){
  unsigned long start = millis();
  while (millis() - start < pollMs){
    modbusPoll(); // keep field bus responsive
    // (We no longer process other local commands mid-run; only OFF via Modbus or USB)
    delay(1);
  }
  return g_abort;
}

void modeInit(float seconds){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_INIT;
  g_errorMask = 0;
  solenoidOff();
  M2_Rev();
  unsigned long dur = secToMs(seconds), t0 = millis();
  while (!g_abort && (millis() - t0 < dur)) pollDuringRun(10);
  if (g_abort){ g_errorMask |= (1<<1); toOff(); }
  else { toStandby(); }
  g_isBusy = false;
}

void modeFroth(float targetC, float timeoutSec){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_FROTH;
  g_errorMask = 0;
  solenoidOff();
  M2_Rev();
  unsigned long t0 = millis(), tout = secToMs(timeoutSec);
  while (!g_abort){
    float c;
    bool ok = tc_readC_blocking(c); // paced read, also updates cache
    if (!ok){ g_errorMask |= (1<<0); break; }
    if (c >= targetC) break;
    if (millis() - t0 > tout){ g_errorMask |= (1<<2); break; }
    pollDuringRun(5);
  }
  if (g_abort){ g_errorMask |= (1<<1); toOff(); }
  else { toStandby(); }
  g_isBusy = false;
}

// ====== UPDATED CLEAN MODE ======
// Sequence:
// 0) M1 forward immediately
// 1) wait tValve  -> Solenoid ON
// 2) wait tSteam  -> M2 reverse ON AND Solenoid OFF (simultaneous)
// 3) wait tStandby-> STANDBY (M1=Rev, M2=Fwd, Solenoid=Off)
void modeClean(float tValve, float tSteam, float tStandby){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_CLEAN;
  g_errorMask = 0;

  // 0) start cleaner feed immediately
  M1_Fwd();

  // 1) wait then open solenoid
  { unsigned long t0=millis(), dur=secToMs(tValve);
    while(!g_abort && (millis()-t0<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); toOff(); g_isBusy=false; return; }
  solenoidOn();

  // 2) wait then start steam AND turn solenoid OFF at the same moment
  { unsigned long t0=millis(), dur=secToMs(tSteam);
    while(!g_abort && (millis()-t0<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); toOff(); g_isBusy=false; return; }
  solenoidOff();   // NEW: turn off solenoid when M2 activates
  M2_Rev();

  // 3) wait then return to STANDBY
  { unsigned long t0=millis(), dur=secToMs(tStandby);
    while(!g_abort && (millis()-t0<dur)) pollDuringRun(5); }

  if (g_abort){ g_errorMask |= (1<<1); toOff(); }
  else { toStandby(); }
  g_isBusy = false;
}

// ====== Optional USB Serial command-line (kept for debug) ======
String readLineNonBlocking_USB(){
  static String buf;
  while (Serial.available()){
    char ch=(char)Serial.read();
    if (ch=='\r') continue;
    if (ch=='\n'){ String out=buf; buf=""; out.trim(); out.toLowerCase(); return out; }
    buf += ch;
  }
  return String();
}
void printStatusBlocking_USB(){
  Serial.print(F("Mode="));
  switch(g_currentMode){
    case MODE_OFF: Serial.print(F("OFF")); break;
    case MODE_STANDBY: Serial.print(F("STANDBY")); break;
    case MODE_INIT: Serial.print(F("INIT")); break;
    case MODE_FROTH: Serial.print(F("FROTH")); break;
    case MODE_CLEAN: Serial.print(F("CLEAN")); break;
  }
  Serial.print(F(" | Busy=")); Serial.print(g_isBusy?F("YES"):F("NO"));
  float c; bool ok = tc_readC_blocking(c);
  Serial.print(F(" | Temp=")); if(ok){ Serial.print(c,2); Serial.print(F(" C")); } else Serial.print(F("fault/open"));
  Serial.print(F(" | ErrMask=")); Serial.println(g_errorMask);
}

// Simple dispatcher for local debug (ignored mid-run except 'off'/'status')
void handleUSBCommand(const String &line){
  if (!line.length()) return;
  if (line=="off"){ g_abort=true; toOff(); Serial.println(F("[USB] OFF")); return; }
  if (line=="status"){ printStatusBlocking_USB(); return; }
  if (line=="help"){
    Serial.println(F("USB cmds: off | status | standby | init s | froth T timeout | clean tValve tSteam tStandby"));
    return;
  }
  if (g_isBusy){ Serial.println(F("[USB] Busy; ignoring")); return; }

  // Simple parse
  String tok[5]; int n=0, start=0;
  while (n<5){ int sp=line.indexOf(' ', start); String part=(sp==-1)?line.substring(start):line.substring(start,sp);
    part.trim(); if(part.length()) tok[n++]=part; if(sp==-1) break; start=sp+1; }

  if (n>=1 && tok[0]=="standby"){ toStandby(); return; }
  if (n>=2 && tok[0]=="init"){ float s=tok[1].toFloat(); modeInit(s); return; }
  if (n>=3 && tok[0]=="froth"){ float T=tok[1].toFloat(); float to=tok[2].toFloat(); modeFroth(T,to); return; }
  if (n>=4 && tok[0]=="clean"){ float a=tok[1].toFloat(); float b=tok[2].toFloat(); float c=tok[3].toFloat(); modeClean(a,b,c); return; }
  Serial.println(F("[USB] Unknown. Type 'help'."));
}

// ====== Arduino setup/loop ======
void setup(){
  // Actuators
  pinMode(SOLENOID_PIN, OUTPUT); solenoidOff();
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  toOff();

  // MAX6675
  pinMode(TC_CS, OUTPUT); pinMode(TC_SCK, OUTPUT); pinMode(TC_SO, INPUT);
  digitalWrite(TC_CS, HIGH); digitalWrite(TC_SCK, LOW);

  // RS-485 direction ctrl
  pinMode(RS485_RE_PIN, OUTPUT);
  pinMode(RS485_DE_PIN, OUTPUT);
  rs485ToRX();

  // Ports
  Serial.begin(115200);         // USB debug (optional)
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);

  // Init HR defaults
  memset(HR, 0, sizeof(HR));
  HR[HR_INIT_SECSX100]    = 250;   // 2.50 s (example defaults)
  HR[HR_FROTH_TARGETX100] = 6000;  // 60.00 C
  HR[HR_FROTH_TOUTX100]   = 18000; // 180.00 s
  HR[HR_CLEAN_TVALVEX100] = 500;   // 5.00 s  (example for new routine)
  HR[HR_CLEAN_TSTEAMX100] = 500;   // 5.00 s
  HR[HR_CLEAN_TSTDBYX100] = 500;   // 5.00 s
  updateStatusRegs();

  Serial.println(F("\n=== Steam Wand Modbus RTU Slave (no libs) ==="));
  Serial.println(F("Modbus: Serial1 @ 9600 8E1, Slave ID=1"));
  Serial.println(F("USB debug: 'off' | 'status' | 'standby' | 'init s' | 'froth T timeout' | 'clean a b c'"));
}

void loop(){
  // Always service the bus
  modbusPoll();

  // Update status registers (temp/flags) regularly
  updateStatusRegs();

  // USB debug (optional)
  String cmd = readLineNonBlocking_USB();
  if (cmd.length()) handleUSBCommand(cmd);

  // If not busy, check HR_CMD to start modes requested via Modbus
  if (!g_isBusy){
    uint16_t cmdv = HR[HR_CMD];
    switch (cmdv){
      case 0: /* off already applied when written */ break;
      case 1: toStandby(); HR[HR_CMD]=0; break;
      case 2: { float s = regToSec(HR[HR_INIT_SECSX100]);   HR[HR_CMD]=0; modeInit(s); } break;
      case 3: { float T = regToC  (HR[HR_FROTH_TARGETX100]);
                float to= regToSec(HR[HR_FROTH_TOUTX100]);  HR[HR_CMD]=0; modeFroth(T,to); } break;
      case 4: { float a = regToSec(HR[HR_CLEAN_TVALVEX100]);
                float b = regToSec(HR[HR_CLEAN_TSTEAMX100]);
                float c = regToSec(HR[HR_CLEAN_TSTDBYX100]); HR[HR_CMD]=0; modeClean(a,b,c); } break;
      default: /* ignore */ break;
    }
  }
}
