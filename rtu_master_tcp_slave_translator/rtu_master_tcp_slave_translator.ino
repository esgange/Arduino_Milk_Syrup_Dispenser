/*
  Unified Modbus RTU Master (0x17) for two slaves on one RS-485 line
  Hardware: Arduino Nano Every + MAX485
    RO -> D0 (RX1), DI -> D1 (TX1), RE+DE -> D2 (HIGH=TX, LOW=RX)
  Bus: 19200 8N1

  Slaves:
    - DEV_DISP   (Dispenser)   Slave ID = 1
    - DEV_CLEAN  (Cleaner)     Slave ID = 2

  Key behavior:
    • Start-command acceptance timeout = 2.5 s (all start ops).
    • If the first reply CRC/timeout fails, probe STATUS; if ACTIVE, proceed (salvage).
    • RS-485 turnaround tuned for Dispenser compatibility:
        - After TX flush: drop DE immediately, then wait ~3 char-times before RX.
    • Non-blocking per-device pending loops; STATUS poll ~150 ms.
    • Logs are device-prefixed: [D] and [C].
    • Anti-collision: waitBusIdle + drainLate around errors and between startup probes.
    • ABORT: avoid double-sending (only schedule background ABORT if immediate one fails).

  Unified CLI (device FIRST):
    Common:
      status <d|c>
      clear  <d|c>
      abort  <d|c|*>
    Dispenser-only:
      rinse     <d> <seconds>
      trigger   <d> <motorId|name> <seconds>
      dispense  <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>
    Cleaner-only:
      off    <c>
      init   <c> <seconds>
      froth  <c> <targetC> <timeout_s>
      clean  <c> <tValve_s> <tSteam_s> <tStandby_s>
*/

#include <Arduino.h>
#include <string.h>

// ---------------- RS485 link ----------------
#define RS485_EN_PIN 2
#define RS485       Serial1
#define BUS_BAUD    19200

// ---------------- Devices / Slave IDs ----------------
enum Device : uint8_t { DEV_NONE=0, DEV_DISP=1, DEV_CLEAN=2 };
constexpr uint8_t SLAVE_ID_DISP  = 1;
constexpr uint8_t SLAVE_ID_CLEAN = 2;
inline uint8_t slaveId(Device d){ return (d==DEV_DISP)?SLAVE_ID_DISP : (d==DEV_CLEAN)?SLAVE_ID_CLEAN : 0; }

// ---------------- Modbus constants ----------------
#define MB_FUNC_RW_MREGS 0x17
#define REG_CMD_BASE  0x0000
#define REG_RES_BASE  0x0100
const uint16_t RES_QTY = 0x000B;      // read 11 regs

// Common opcodes
constexpr uint16_t OP_STATUS = 1;
constexpr uint16_t OP_CLEAR  = 2;
constexpr uint16_t OP_ABORT  = 3;

// Dispenser-only opcodes
constexpr uint16_t OPD_DISPENSE = 10;
constexpr uint16_t OPD_RINSE    = 11;
constexpr uint16_t OPD_TRIGGER  = 12;

// Cleaner-only opcodes
constexpr uint16_t OPC_OFF   = 4;
constexpr uint16_t OPC_INIT  = 10;
constexpr uint16_t OPC_FROTH = 11;
constexpr uint16_t OPC_CLEAN = 12;

// Result decode
enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };
enum ApiError   : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};

// STATUS poll cadence
const uint16_t STATUS_POLL_MS = 150;

// ---- Timing: match dispenser (11 bits/char used in original) ----
constexpr float   BITS_PER_CHAR = 11.0f; // keep conservative timing like your dispenser
const uint16_t    TCHAR_US      = (uint16_t)((BITS_PER_CHAR / BUS_BAUD) * 1e6 + 0.5f); // ≈573 µs @19200
const uint16_t    T3P5_US       = (uint16_t)(3.5f * TCHAR_US);   // spec gap ~2.0 ms

// Acceptance timeout for start commands
const uint32_t ACCEPT_MS = 2500;

// ---------------------------------------------
// CRC16 (Modbus) poly 0xA001, init 0xFFFF
// ---------------------------------------------
uint16_t mb_crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

// ---------------------------------------------
// RS485 direction control
// ---------------------------------------------
inline void rs485Rx() { digitalWrite(RS485_EN_PIN, LOW); }
inline void rs485Tx() { digitalWrite(RS485_EN_PIN, HIGH); }

// ---------------------------------------------
// Utils
// ---------------------------------------------
void purgeRx() { while (RS485.available()) (void)RS485.read(); }

// Gentle read: tiny yield to avoid busy-spin
bool readExact(uint8_t* buf, size_t n, uint32_t deadlineMs) {
  size_t got = 0;
  while ((long)(deadlineMs - millis()) >= 0 && got < n) {
    int b = RS485.read();
    if (b >= 0) buf[got++] = (uint8_t)b;
    else delayMicroseconds(10);
  }
  return (got == n);
}

void hexDump(const uint8_t* b, size_t n) {
  for (size_t i=0;i<n;i++) {
    if (i && (i%16)==0) Serial.println();
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

const char* devPrefix(Device d){ return (d==DEV_DISP)?"[D] ":"[C] "; }

// ---- NEW: wait for a real idle window (no incoming bytes) ----
bool waitBusIdle(uint32_t silent_us, uint32_t max_wait_ms = 20) {
  uint32_t deadline = millis() + max_wait_ms;
  uint32_t lastActivity = micros();
  while ((long)(deadline - millis()) >= 0) {
    if (RS485.available()) {
      while (RS485.available()) (void)RS485.read();
      lastActivity = micros();
    } else {
      if ((uint32_t)(micros() - lastActivity) >= silent_us) return true;
      delayMicroseconds(20);
    }
  }
  return false; // couldn't get quiet in time; still safe to proceed
}

// ---- NEW: give late replies a moment to finish, then drain ----
void drainLate(uint32_t ms) {
  uint32_t until = millis() + ms;
  while ((long)(until - millis()) >= 0) {
    bool saw = false;
    while (RS485.available()) { (void)RS485.read(); saw = true; }
    if (saw) until = millis() + 2; // extend a touch if bytes kept flowing
    delayMicroseconds(100);
  }
}

// ---------------------------------------------
// Core 0x17 transaction
//  - T3.5 quiet before TX (real idle, not just a delay)
//  - TX -> drop DE immediately after flush()
//  - then brief settle before RX
//  - after any timeout/CRC error: drainLate + waitBusIdle to avoid collisions
// ---------------------------------------------
bool mbReadWriteMultiple(uint8_t slaveId,
                         uint16_t readStart, uint16_t readQty,
                         uint16_t writeStart, const uint16_t* writeRegs, uint16_t writeQty,
                         uint16_t* respRegs, uint16_t respRegsLen,
                         uint32_t overallTimeoutMs,
                         bool verboseOnError = true)
{
  const uint8_t func = MB_FUNC_RW_MREGS;
  const uint8_t writeByteCount = (uint8_t)(writeQty * 2);
  const uint16_t pduLen  = 1+1 +2+2 +2+2 +1 + writeByteCount; // no CRC
  const uint16_t frameLen = pduLen + 2;
  uint8_t frame[256];
  if (frameLen > sizeof(frame)) return false;

  uint16_t idx = 0;
  frame[idx++] = slaveId;
  frame[idx++] = func;
  frame[idx++] = (uint8_t)(readStart >> 8);
  frame[idx++] = (uint8_t)(readStart & 0xFF);
  frame[idx++] = (uint8_t)(readQty >> 8);
  frame[idx++] = (uint8_t)(readQty & 0xFF);
  frame[idx++] = (uint8_t)(writeStart >> 8);
  frame[idx++] = (uint8_t)(writeStart & 0xFF);
  frame[idx++] = (uint8_t)(writeQty >> 8);
  frame[idx++] = (uint8_t)(writeQty & 0xFF);
  frame[idx++] = writeByteCount;
  for (uint16_t i=0;i<writeQty;i++) {
    frame[idx++] = (uint8_t)(writeRegs[i] >> 8);
    frame[idx++] = (uint8_t)(writeRegs[i] & 0xFF);
  }
  uint16_t crc = mb_crc16(frame, idx);
  frame[idx++] = (uint8_t)(crc & 0xFF);   // CRC lo
  frame[idx++] = (uint8_t)(crc >> 8);     // CRC hi

  // Ensure real idle before we drive the bus
  rs485Rx();
  purgeRx();                     // clean any stale
  waitBusIdle(T3P5_US, 20);      // enforce ≥3.5 char-times of silence

  rs485Tx();
  RS485.write(frame, idx);
  RS485.flush();                 // TX fully out (buffer + shift reg)
  rs485Rx();                     // release the bus immediately
  delayMicroseconds(50);         // brief settle (40–60 µs is fine)

  // Expect: addr, func, byteCount, data(byteCount), crcLo, crcHi
  const uint16_t expectDataBytes = (uint16_t)(readQty * 2);

  uint8_t hdr[3];
  uint32_t deadline = millis() + overallTimeoutMs;
  if (!readExact(hdr, 3, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on header"));
    drainLate(8);
    waitBusIdle(T3P5_US, 20);
    return false;
  }

  // Exception?
  if ((hdr[1] & 0x80) && hdr[0] == slaveId) {
    uint8_t exCrc[2];
    (void)readExact(exCrc, 2, deadline);
    if (verboseOnError) {
      Serial.print(F("[modbus] exception code=0x")); Serial.println(hdr[2], HEX);
    }
    drainLate(4);
    waitBusIdle(T3P5_US, 20);
    return false;
  }

  if (hdr[0] != slaveId || hdr[1] != func || hdr[2] != expectDataBytes) {
    uint8_t dump[96]; size_t dn = 0;
    while (RS485.available() && dn < sizeof(dump)) dump[dn++] = (uint8_t)RS485.read();
    if (verboseOnError) {
      Serial.print(F("[modbus] bad header: "));
      Serial.print(hdr[0],HEX); Serial.print(' ');
      Serial.print(hdr[1],HEX); Serial.print(' ');
      Serial.print(hdr[2],HEX); Serial.println();
      if (dn) { Serial.println(F("[modbus] tail dump:")); hexDump(dump, dn); }
    }
    drainLate(8);
    waitBusIdle(T3P5_US, 20);
    return false;
  }

  // Read payload + CRC
  const uint16_t rem = expectDataBytes + 2;
  uint8_t rest[300];
  if (!readExact(rest, rem, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on payload"));
    drainLate(8);
    waitBusIdle(T3P5_US, 20);
    return false;
  }

  // CRC check (over addr..byteCount+data)
  uint8_t full[3 + 300];
  memcpy(full, hdr, 3);
  memcpy(full+3, rest, expectDataBytes);
  uint16_t gotCrc = (uint16_t)(rest[rem-2] | (rest[rem-1] << 8));
  uint16_t calcCrc = mb_crc16(full, (uint16_t)(3 + expectDataBytes));
  if (gotCrc != calcCrc) {
    if (verboseOnError) {
      Serial.println(F("[modbus] CRC mismatch; header+data dump:"));
      hexDump(full, 3 + expectDataBytes);
      Serial.print(F("gotCrc=0x")); Serial.println(gotCrc, HEX);
      Serial.print(F("calculated=0x")); Serial.println(calcCrc, HEX);
    }
    drainLate(8);
    waitBusIdle(T3P5_US, 20);
    return false;
  }

  // Unpack data to regs
  if (respRegsLen < readQty) return false;
  for (uint16_t i=0;i<readQty;i++) {
    respRegs[i] = (uint16_t)((rest[2*i] << 8) | rest[2*i+1]);
  }
  return true;
}

// ---------------------------------------------
// Pretty-print helpers
// ---------------------------------------------
const char* errToStr(uint16_t e) {
  switch (e) {
    case AE_NONE:        return "NONE";
    case AE_BUSY:        return "BUSY";
    case AE_MOTOR:       return "MOTOR_FAULT";
    case AE_LEAK:        return "LEAK_FAULT";
    case AE_SCALE:       return "SCALE_FAULT";
    case AE_TIMEOUT:     return "TIMEOUT_FAULT";
    case AE_BAD_ARGS:    return "BAD_ARGS";
    case AE_INVALID_CMD: return "INVALID_CMD";
    case AE_ABORTED:     return "ABORTED";
    default:             return "?";
  }
}

// OFF=7 used by Cleaner; harmless for Dispenser
const char* statusToStr(uint16_t s) {
  switch (s) {
    case 0: return "IDLE";
    case 1: return "ACTIVE";
    case 2: return "MOTOR_FAULT";
    case 3: return "LEAK_FAULT";
    case 4: return "SCALE_FAULT";
    case 5: return "TIMEOUT_FAULT";
    case 6: return "ABORTED_FAULT";
    case 7: return "OFF";
    default: return "?";
  }
}

void printResultBrief(Device dev, const uint16_t* R) {
  uint16_t resultCode    = R[0];
  uint16_t errorCode     = R[1];
  uint16_t systemStatus  = R[2];
  uint16_t aux0_tempx10  = R[4];
  uint16_t aux1_elapx10  = R[5];

  Serial.print(devPrefix(dev));
  Serial.print(F("result="));
  Serial.print(resultCode==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error="));  Serial.print(errToStr(errorCode));
  Serial.print(F("  status=")); Serial.print(statusToStr(systemStatus));

  if (dev == DEV_CLEAN) {
    if (aux0_tempx10 != 0) {
      Serial.print(F("  tempC="));    Serial.print(aux0_tempx10 / 10.0f, 1);
    }
    if (aux1_elapx10 != 0) {
      Serial.print(F("  elapsed_s=")); Serial.print(aux1_elapx10 / 10.0f, 1);
    }
  }
  Serial.println();
}

// ---------------------------------------------
// [sent] datalog echoes (device-prefixed)
// ---------------------------------------------
static void logSent_STATUS(Device d) { Serial.print(F("-------------\n")); Serial.print(devPrefix(d)); Serial.println(F("[sent] STATUS.")); }
static void logSent_CLEAR (Device d) { Serial.print(F("-------------\n")); Serial.print(devPrefix(d)); Serial.println(F("[sent] CLEAR.")); }
static void logSent_ABORT (Device d) { Serial.print(F("-------------\n")); Serial.print(devPrefix(d)); Serial.println(F("[sent] ABORT.")); }
static void logSent_OFF   (Device d) { Serial.print(F("-------------\n")); Serial.print(devPrefix(d)); Serial.println(F("[sent] OFF.")); }

static void logSent_RINSE(Device d, float seconds) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] RINSE seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_TRIGGER(Device d, uint16_t motorId, float seconds, const char* name) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] TRIGGER motor=")); Serial.print(name);
  Serial.print(F(" seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_DISPENSE(Device d, uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s, const char* name) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] DISPENSE motor=")); Serial.print(name);
  Serial.print(F(" target_g=")); Serial.print(target_g, 1);
  Serial.print(F(" slowOffset_g=")); Serial.print(slow_g, 1);
  Serial.print(F(" softCutOffset_g=")); Serial.print(soft_g, 1);
  Serial.print(F(" timeout_s=")); Serial.print(timeout_s); Serial.println(F("."));
}
static void logSent_INIT(Device d, float seconds) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] INIT seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_FROTH(Device d, float targetC, float timeout_s) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] FROTH targetC=")); Serial.print(targetC, 1);
  Serial.print(F("C timeout_s=")); Serial.print(timeout_s, 1); Serial.println(F("."));
}
static void logSent_CLEAN(Device d, float tValve, float tSteam, float tStandby) {
  Serial.print(F("-------------\n")); Serial.print(devPrefix(d));
  Serial.print(F("[sent] CLEAN tValve_s=")); Serial.print(tValve, 1);
  Serial.print(F(" tSteam_s=")); Serial.print(tSteam, 1);
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby, 1); Serial.println(F("."));
}

// ---------------------------------------------
// Motor token <-> numeric ID (Dispenser only)
// ---------------------------------------------
bool motorTokenToId(const String& token, uint16_t& outId) {
  String t = token; t.trim();
  String tl = t; tl.toLowerCase();

  // numeric?
  bool allDigits = true;
  for (uint16_t i=0;i<tl.length();++i) if (!isDigit(tl[i])) { allDigits = false; break; }
  if (allDigits && tl.length()>0) { outId = (uint16_t)tl.toInt(); return true; }

  auto parseIdx = [&](const String& s, uint8_t &idx)->bool {
    long v = s.toInt(); if (v < 1 || v > 255) return false; idx = (uint8_t)v; return true;
  };

  uint8_t idx;
  if (tl.startsWith("sauce")) { if (!parseIdx(tl.substring(5), idx)) return false; if (idx<1 || idx>15) return false; outId = 100 + idx; return true; }
  if (tl.startsWith("milk"))  { if (!parseIdx(tl.substring(4), idx)) return false; if (idx<1 || idx>8 ) return false; outId = idx;       return true; }
  if (tl.startsWith("s"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1 || idx>15) return false; outId = 100 + idx; return true; }
  if (tl.startsWith("m"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1 || idx>8 ) return false; outId = idx;       return true; }

  return false;
}

void idToName(uint16_t motorId, char* out, size_t cap) {
  if (motorId >= 1 && motorId <= 8) {
    snprintf(out, cap, "Milk%u", (unsigned)motorId);
  } else if (motorId >= 101 && motorId <= 115) {
    snprintf(out, cap, "Sauce%u", (unsigned)(motorId - 100));
  } else {
    snprintf(out, cap, "ID%u", (unsigned)motorId);
  }
}

// ---------------------------------------------
// API wrappers  (seq not used; write seq=0)
// ---------------------------------------------
bool api_status(Device dev, uint16_t* outRegs, uint32_t timeoutMs=600);
bool api_status_logged(Device dev, uint16_t* outRegs, uint32_t timeoutMs=2000);

bool api_status(Device dev, uint16_t* outRegs, uint32_t timeoutMs) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS; W[1] = 0;
  return mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool api_status_logged(Device dev, uint16_t* outRegs, uint32_t timeoutMs) {
  logSent_STATUS(dev);
  return api_status(dev, outRegs, timeoutMs);
}

bool api_clear(Device dev, uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[8] = {0};
  W[0] = OP_CLEAR; W[1] = 0;
  logSent_CLEAR(dev);
  return mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_abort(Device dev, uint16_t* outRegs, uint32_t timeoutMs=800) {
  uint16_t W[8] = {0};
  W[0] = OP_ABORT; W[1] = 0;
  logSent_ABORT(dev);
  return mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_off(Device dev, uint16_t* outRegs, uint32_t timeoutMs=2000) {
  if (dev != DEV_CLEAN) return false;
  uint16_t W[8] = {0};
  W[0] = OPC_OFF; W[1] = 0;
  logSent_OFF(dev);
  return mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

// --- Salvage helper: if last command's reply had CRC/timeout, check STATUS for ACTIVE
static bool salvage_acceptance(Device dev, uint16_t* outRegs, const char* opName){
  uint16_t S[RES_QTY] = {0};
  bool ok = api_status(dev, S, 600);
  if (ok && S[2] == 1 /*ACTIVE*/) {
    for (uint16_t i=0;i<RES_QTY;i++) outRegs[i] = S[i];
    Serial.print(devPrefix(dev));
    Serial.print(F("[salvage] reply CRC/timeout, but live STATUS=ACTIVE; proceeding ("));
    Serial.print(opName);
    Serial.println(F(")."));
    return true;
  }
  return false;
}

// ---- Dispenser ops ----
bool api_rinse(Device dev, float seconds, uint16_t* outRegs) {
  if (dev != DEV_DISP) return false;
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OPD_RINSE; W[1] = 0; W[2] = seconds_x10;
  logSent_RINSE(dev, seconds);
  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 3, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "rinse");
  return ok;
}

bool api_trigger(Device dev, uint16_t motorId, float seconds, uint16_t* outRegs) {
  if (dev != DEV_DISP) return false;
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OPD_TRIGGER; W[1] = 0; W[2] = motorId; W[3] = seconds_x10;
  char name[16]; idToName(motorId, name, sizeof(name));
  logSent_TRIGGER(dev, motorId, seconds, name);
  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 4, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "trigger");
  return ok;
}

bool api_dispense(Device dev, uint16_t motorId, float target_g, float slowOffset_g, float softCutOffset_g, uint16_t timeout_s, uint16_t* outRegs) {
  if (dev != DEV_DISP) return false;
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OPD_DISPENSE; W[1] = 0; W[2] = motorId;
  W[3] = to_x10(target_g);
  W[4] = to_x10(slowOffset_g);
  W[5] = to_x10(softCutOffset_g);
  W[6] = timeout_s;

  char name[16]; idToName(motorId, name, sizeof(name));
  logSent_DISPENSE(dev, motorId, target_g, slowOffset_g, softCutOffset_g, timeout_s, name);

  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 7, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "dispense");
  return ok;
}

// ---- Cleaner ops ----
bool api_init(Device dev, float seconds, uint16_t* outRegs) {
  if (dev != DEV_CLEAN) return false;
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OPC_INIT; W[1] = 0; W[2] = seconds_x10;
  logSent_INIT(dev, seconds);
  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 3, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "init");
  return ok;
}

bool api_froth(Device dev, float targetC, float timeout_s, uint16_t* outRegs) {
  if (dev != DEV_CLEAN) return false;
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OPC_FROTH; W[1] = 0;
  W[2] = to_x10(targetC);
  W[3] = to_x10(timeout_s);
  logSent_FROTH(dev, targetC, timeout_s);
  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 4, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "froth");
  return ok;
}

bool api_clean(Device dev, float tValve, float tSteam, float tStandby, uint16_t* outRegs) {
  if (dev != DEV_CLEAN) return false;
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OPC_CLEAN; W[1] = 0;
  W[2] = to_x10(tValve);
  W[3] = to_x10(tSteam);
  W[4] = to_x10(tStandby);
  logSent_CLEAN(dev, tValve, tSteam, tStandby);
  bool ok = mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 5, outRegs, RES_QTY, ACCEPT_MS);
  if (!ok) return salvage_acceptance(dev, outRegs, "clean");
  return ok;
}

// ---------------------------------------------
// Pending operation scheduler (per device)
// ---------------------------------------------
enum : uint8_t { PK_NONE=0, PK_RINSE, PK_TRIGGER, PK_DISPENSE, PK_INIT, PK_FROTH, PK_CLEAN };

struct PendingOp {
  uint8_t  kind = PK_NONE;
  uint32_t deadline = 0;       // absolute ms when we warn timeout (but keep polling)
  uint32_t nextPollDue = 0;    // cadence
  bool     printedTimeout = false;
};

static PendingOp pend[3];              // index 1..2 by Device enum value
static bool abortRequested[3] = {false,false,false};

const char* pendingLabel(uint8_t k) {
  switch (k) {
    case PK_RINSE:    return "rinse";
    case PK_TRIGGER:  return "trigger";
    case PK_DISPENSE: return "dispense";
    case PK_INIT:     return "init";
    case PK_FROTH:    return "froth";
    case PK_CLEAN:    return "clean";
    default:          return "?";
  }
}

void armPending(Device dev, uint8_t k, uint32_t extraMs) {
  PendingOp &p = pend[(uint8_t)dev];
  p.kind = k;
  p.printedTimeout = false;
  p.nextPollDue = millis() + STATUS_POLL_MS;
  p.deadline = millis() + extraMs;
}

void clearPending(Device dev) {
  PendingOp &p = pend[(uint8_t)dev];
  p.kind = PK_NONE;
  p.deadline = 0;
  p.nextPollDue = 0;
  p.printedTimeout = false;
}

// One silent STATUS poll tick; returns true if got response and filled R
bool pollStatusOnce(Device dev, uint16_t* R) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS; W[1] = 0;
  return mbReadWriteMultiple(slaveId(dev), REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, R, RES_QTY, 600, /*verboseOnError*/false);
}

void servicePendingOne(Device dev) {
  PendingOp &p = pend[(uint8_t)dev];
  if (p.kind == PK_NONE) return;

  uint32_t now = millis();

  // Abort request has priority over polling
  if (abortRequested[(uint8_t)dev]) {
    uint16_t R[RES_QTY] = {0};
    bool ok = api_abort(dev, R, 800);
    abortRequested[(uint8_t)dev] = false;
    if (ok) {
      printResultBrief(dev, R);  // immediate feedback
      p.nextPollDue = now + 100; // keep polling for transition out of ACTIVE
    } else {
      Serial.print(devPrefix(dev)); Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
    }
    return;
  }

  // Not time to poll yet
  if ((long)(now - p.nextPollDue) < 0) return;

  // Silent STATUS poll
  uint16_t R[RES_QTY] = {0};
  bool ok = pollStatusOnce(dev, R);
  if (!ok) {
    // Missed poll — just try again next cadence
    p.nextPollDue = millis() + STATUS_POLL_MS;
    return;
  }

  uint16_t st = R[2];

  if (st != 1) { // not ACTIVE -> finalize
    if (st == 0) {
      Serial.print(devPrefix(dev)); Serial.print('['); Serial.print(pendingLabel(p.kind)); Serial.println(F("] done."));
    } else {
      Serial.print(devPrefix(dev)); Serial.print('['); Serial.print(pendingLabel(p.kind)); Serial.print(F("] ended: "));
      Serial.println(statusToStr(st));
    }
    clearPending(dev);
    return;
  }

  // Still ACTIVE; check deadline (print ONCE)
  if (!p.printedTimeout && (long)(now - p.deadline) > 0) {
    Serial.print(devPrefix(dev)); Serial.print('['); Serial.print(pendingLabel(p.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    p.printedTimeout = true; // keep polling silently afterward
  }

  p.nextPollDue = millis() + STATUS_POLL_MS;
}

void servicePending() {
  servicePendingOne(DEV_DISP);
  servicePendingOne(DEV_CLEAN);
}

// ---------------------------------------------
// CLI helpers
// ---------------------------------------------
String inLine;

void printHelp() {
  Serial.println(F("Unified commands (device first):"));
  Serial.println(F("  status <d|c>"));
  Serial.println(F("  clear  <d|c>"));
  Serial.println(F("  abort  <d|c|*>"));
  Serial.println(F("  off    <c>"));
  Serial.println(F("  init   <c> <seconds>"));
  Serial.println(F("  froth  <c> <targetC> <timeout_s>"));
  Serial.println(F("  clean  <c> <tValve_s> <tSteam_s> <tStandby_s>"));
  Serial.println(F("  rinse     <d> <seconds>"));
  Serial.println(F("  trigger   <d> <motorId|name> <seconds>   e.g. 'trigger d Sauce2 1.0' or 'trigger d 102 1.0'"));
  Serial.println(F("  dispense  <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
}

bool parseDeviceToken(const String& tok, Device &out) {
  String t = tok; t.trim(); String tl=t; tl.toLowerCase();
  if (tl=="d" || tl=="disp" || tl=="dispenser" || tl=="1") { out = DEV_DISP;  return true; }
  if (tl=="c" || tl=="clean" || tl=="cleaner" || tl=="2")  { out = DEV_CLEAN; return true; }
  return false;
}
bool isAllToken(const String& tok) {
  String tl = tok; String low=tl; low.toLowerCase();
  return (low=="*" || low=="all" || low=="both");
}

// ---------------------------------------------
// NEW: Boot probe behavior (forever until reply)
// ---------------------------------------------
constexpr uint32_t BOOT_PROBE_TIMEOUT_MS     = 1500; // per attempt STATUS timeout
constexpr uint32_t BOOT_PROBE_RETRY_DELAY_MS = 80;   // pause between tries

bool bootProbeUntilReply(Device dev, uint16_t* R,
                         uint32_t perAttemptTimeoutMs = BOOT_PROBE_TIMEOUT_MS,
                         uint32_t interTryDelayMs     = BOOT_PROBE_RETRY_DELAY_MS)
{
  uint32_t attempt = 0;
  for (;;) {
    attempt++;

    Serial.print(devPrefix(dev));
    Serial.print(F("[boot] Probing STATUS, attempt #"));
    Serial.println(attempt);

    bool ok = api_status_logged(dev, R, perAttemptTimeoutMs);
    if (ok) {
      Serial.print(devPrefix(dev)); Serial.println(F("Modbus OK"));
      printResultBrief(dev, R);
      return true; // success -> leave function
    }

    Serial.print(devPrefix(dev));
    Serial.println(F("[boot] NO RESPONSE / CRC; retrying..."));

    // bus settle / anti-collision before next attempt
    drainLate(8);
    waitBusIdle(T3P5_US, 50);
    delay(interTryDelayMs);
  }
}

// ---------------------------------------------
// Arduino lifecycle
// ---------------------------------------------
void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) {}

  RS485.begin(BUS_BAUD, SERIAL_8N1);
  delay(50);

  Serial.println(F("\nUnified Modbus Master ready (DE drop->3char RX guard, 2.5s acceptance, salvage-on-ACTIVE, anti-collision idle/drain)."));
  printHelp();

  uint16_t R[RES_QTY] = {0};

  // -------- Dispenser: repeat until reply --------
  Serial.print(F("\n[boot] Probing Dispenser STATUS (repeat-until-response)...\n"));
  (void)bootProbeUntilReply(DEV_DISP, R);

  // Settle bus before probing the other slave
  drainLate(8);
  waitBusIdle(T3P5_US, 50);

  // -------- Cleaner: repeat until reply --------
  memset(R, 0, sizeof(R));
  Serial.print(F("\n[boot] Probing Cleaner STATUS (repeat-until-response)...\n"));
  (void)bootProbeUntilReply(DEV_CLEAN, R);

  // Final settle before entering loop
  drainLate(8);
  waitBusIdle(T3P5_US, 50);

  // If we ever add more init, it runs ONLY after both slaves are confirmed alive.
}

void loop() {
  // Service background pending operations
  servicePending();

  // Non-blocking CLI
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      inLine.trim();
      if (!inLine.length()) continue;

      String cmd = inLine; inLine = "";
      String low = cmd; low.toLowerCase();

      uint16_t R[RES_QTY] = {0};
      bool ok = false;

      if (low == "help") { printHelp(); goto after_line; }

      auto usage = [&](const __FlashStringHelper* s){ Serial.println(s); };

      // STATUS <dev>
      if (low.startsWith("status ")) {
        String tok = cmd.substring(7); tok.trim();
        Device dev;
        if (!parseDeviceToken(tok, dev)) { usage(F("Use: status <d|c>")); goto after_line; }
        ok = api_status_logged(dev, R);
        if (ok) printResultBrief(dev, R);
        else    { Serial.print(devPrefix(dev)); Serial.println(F("[status] NO RESPONSE / CRC/timeout")); }
        goto after_line;
      }

      // CLEAR <dev>
      if (low.startsWith("clear ")) {
        String tok = cmd.substring(6); tok.trim();
        Device dev;
        if (!parseDeviceToken(tok, dev)) { usage(F("Use: clear <d|c>")); goto after_line; }
        ok = api_clear(dev, R);
        if (ok) printResultBrief(dev, R);
        else    { Serial.print(devPrefix(dev)); Serial.println(F("[clear] NO RESPONSE / CRC/timeout")); }
        goto after_line;
      }

      // ABORT <dev|*>
      if (low.startsWith("abort ")) {
        String tok = cmd.substring(6); tok.trim();
        if (isAllToken(tok)) {
          for (uint8_t i=1;i<=2;i++){
            Device dev = (Device)i;
            bool needBgAbort = (pend[i].kind != PK_NONE);
            memset(R,0,sizeof(R));
            ok = api_abort(dev, R, 800);
            if (ok) {
              printResultBrief(dev, R);
              abortRequested[i] = false; // immediate success -> don't queue another
            } else {
              if (needBgAbort) abortRequested[i] = true; // queue one background attempt
              Serial.print(devPrefix(dev)); Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
            }
          }
        } else {
          Device dev;
          if (!parseDeviceToken(tok, dev)) { usage(F("Use: abort <d|c|*>")); goto after_line; }
          bool needBgAbort = (pend[(uint8_t)dev].kind != PK_NONE);
          ok = api_abort(dev, R, 800);
          if (ok) {
            printResultBrief(dev, R);
            abortRequested[(uint8_t)dev] = false;
          } else {
            if (needBgAbort) abortRequested[(uint8_t)dev] = true;
            Serial.print(devPrefix(dev)); Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
          }
        }
        goto after_line;
      }

      // OFF <c>
      if (low.startsWith("off ")) {
        String tok = cmd.substring(4); tok.trim();
        Device dev;
        if (!parseDeviceToken(tok, dev) || dev!=DEV_CLEAN) { usage(F("Use: off <c>")); goto after_line; }
        ok = api_off(dev, R);
        if (ok) printResultBrief(dev, R);
        else    { Serial.print(devPrefix(dev)); Serial.println(F("[off] NO RESPONSE / CRC/timeout")); }
        goto after_line;
      }

      // INIT <c> <seconds>
      if (low.startsWith("init ")) {
        String rest = cmd.substring(5); rest.trim();
        int sp = rest.indexOf(' ');
        if (sp < 0) { usage(F("Use: init <c> <seconds>")); goto after_line; }
        String tokDev = rest.substring(0, sp); tokDev.trim();
        String sSecs  = rest.substring(sp+1); sSecs.trim();
        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_CLEAN) { usage(F("Use: init <c> <seconds>")); goto after_line; }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }
        float secs = sSecs.toFloat();
        ok = api_init(dev, secs, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[init] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 5000UL;
            armPending(dev, PK_INIT, (uint32_t)(secs*1000.0f) + guard);
          }
        }
        goto after_line;
      }

      // FROTH <c> <targetC> <timeout_s>
      if (low.startsWith("froth ")) {
        String rest = cmd.substring(6); rest.trim();
        int sp1 = rest.indexOf(' ');
        if (sp1 < 0) { usage(F("Use: froth <c> <targetC> <timeout_s>")); goto after_line; }
        int sp2 = rest.indexOf(' ', sp1+1);
        if (sp2 < 0) { usage(F("Use: froth <c> <targetC> <timeout_s>")); goto after_line; }

        String tokDev = rest.substring(0, sp1); tokDev.trim();
        String sTarget= rest.substring(sp1+1, sp2); sTarget.trim();
        String sTout  = rest.substring(sp2+1); sTout.trim();

        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_CLEAN) { usage(F("Use: froth <c> <targetC> <timeout_s>")); goto after_line; }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }

        float targetC = sTarget.toFloat();
        float tout    = sTout.toFloat();
        ok = api_froth(dev, targetC, tout, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[froth] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 10000UL; // cushion
            armPending(dev, PK_FROTH, (uint32_t)(tout*1000.0f) + guard);
          }
        }
        goto after_line;
      }

      // CLEAN <c> <tValve_s> <tSteam_s> <tStandby_s>
      if (low.startsWith("clean ")) {
        String rest = cmd.substring(6); rest.trim();
        int s1 = rest.indexOf(' ');
        int s2 = (s1>=0)? rest.indexOf(' ', s1+1) : -1;
        int s3 = (s2>=0)? rest.indexOf(' ', s2+1) : -1;
        if (s1<0 || s2<0 || s3<0) { usage(F("Use: clean <c> <tValve_s> <tSteam_s> <tStandby_s>")); goto after_line; }

        String tokDev = rest.substring(0, s1); tokDev.trim();
        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_CLEAN) { usage(F("Use: clean <c> <tValve_s> <tSteam_s> <tStandby_s>")); goto after_line; }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }

        float tValve = rest.substring(s1+1, s2).toFloat();
        float tSteam = rest.substring(s2+1, s3).toFloat();
        float tStby  = rest.substring(s3+1).toFloat();

        ok = api_clean(dev, tValve, tSteam, tStby, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[clean] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 10000UL;
            uint32_t total = (uint32_t)((tValve + tSteam + tStby)*1000.0f) + guard;
            armPending(dev, PK_CLEAN, total);
          }
        }
        goto after_line;
      }

      // RINSE <d> <seconds>
      if (low.startsWith("rinse ")) {
        String rest = cmd.substring(6); rest.trim();
        int sp = rest.indexOf(' ');
        if (sp < 0) { usage(F("Use: rinse <d> <seconds>")); goto after_line; }
        String tokDev = rest.substring(0, sp); tokDev.trim();
        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_DISP) { usage(F("Use: rinse <d> <seconds>")); goto after_line; }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }
        float secs = rest.substring(sp+1).toFloat();

        ok = api_rinse(dev, secs, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[rinse] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 5000UL;
            armPending(dev, PK_RINSE, (uint32_t)(secs*1000.0f) + guard);
          }
        }
        goto after_line;
      }

      // TRIGGER <d> <motorId|name> <seconds>
      if (low.startsWith("trigger ")) {
        String rest = cmd.substring(8); rest.trim();
        int sp1 = rest.indexOf(' ');
        int sp2 = (sp1>=0)? rest.indexOf(' ', sp1+1) : -1;
        if (sp1<0 || sp2<0) { usage(F("Use: trigger <d> <motorId|name> <seconds>")); goto after_line; }

        String tokDev = rest.substring(0, sp1); tokDev.trim();
        String tokId  = rest.substring(sp1+1, sp2); tokId.trim();
        String ssec   = rest.substring(sp2+1); ssec.trim();
        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_DISP) { usage(F("Use: trigger <d> <motorId|name> <seconds>")); goto after_line; }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }

        uint16_t motorId;
        if (!motorTokenToId(tokId, motorId)) { Serial.println(F("[err] unknown motor token")); goto after_line; }
        float secs = ssec.toFloat();

        ok = api_trigger(dev, motorId, secs, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[trigger] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 5000UL;
            armPending(dev, PK_TRIGGER, (uint32_t)(secs*1000.0f) + guard);
          }
        }
        goto after_line;
      }

      // DISPENSE <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>
      if (low.startsWith("dispense ")) {
        String rest = cmd.substring(9); rest.trim();

        // tokenize safely
        int p1 = rest.indexOf(' ');
        if (p1<0) { usage(F("Use: dispense <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>")); goto after_line; }
        String tokDev = rest.substring(0, p1); tokDev.trim();

        Device dev;
        if (!parseDeviceToken(tokDev, dev) || dev!=DEV_DISP) {
          usage(F("Use: dispense <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
          goto after_line;
        }
        if (pend[(uint8_t)dev].kind != PK_NONE) { Serial.println(F("[busy] operation in progress; type 'abort <dev>' or 'status <dev>'.")); goto after_line; }

        String args = rest.substring(p1+1); args.trim();
        // Expect 5 tokens left
        int a1 = args.indexOf(' ');
        int a2 = (a1>=0)? args.indexOf(' ', a1+1) : -1;
        int a3 = (a2>=0)? args.indexOf(' ', a2+1) : -1;
        int a4 = (a3>=0)? args.indexOf(' ', a3+1) : -1;
        if (a1<0 || a2<0 || a3<0 || a4<0) {
          usage(F("Use: dispense <d> <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
          goto after_line;
        }

        String tokId   = args.substring(0, a1); tokId.trim();
        float target_g = args.substring(a1+1, a2).toFloat();
        float slow_g   = args.substring(a2+1, a3).toFloat();
        float soft_g   = args.substring(a3+1, a4).toFloat();
        uint16_t tout_s= (uint16_t)args.substring(a4+1).toInt();

        uint16_t motorId;
        if (!motorTokenToId(tokId, motorId)) { Serial.println(F("[err] unknown motor token")); goto after_line; }

        ok = api_dispense(dev, motorId, target_g, slow_g, soft_g, tout_s, R);
        if (!ok) { Serial.print(devPrefix(dev)); Serial.println(F("[dispense] NO RESPONSE / CRC/timeout")); }
        else {
          printResultBrief(dev, R);
          if (R[0]==RC_OK && R[2]==1) {
            uint32_t guard = 10000UL; // settle/coast/comm cushion
            armPending(dev, PK_DISPENSE, (uint32_t)tout_s*1000UL + guard);
          }
        }
        goto after_line;
      }

      // Unknown command
      Serial.println(F("Unknown command. Type 'help'."));

after_line:
      // After each command line, immediately service to keep UI snappy
      servicePending();

    } else {
      if (inLine.length() < 160) inLine += c;
    }
  }

  // Idle tick: service pendings again if due
  servicePending();
}
