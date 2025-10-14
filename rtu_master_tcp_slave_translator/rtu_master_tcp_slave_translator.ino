/*
  Arduino Nano Every — Modbus RTU Master (0x17) for Mega2560 Dispenser
  RS-485: MAX485 — RO->D0(RX1), DI->D1(TX1), RE+DE->D2 (HIGH=TX, LOW=RX)
  Bus: 19200 8N1, Slave ID = 1

  Motor tokens:
    Milk1..Milk8   (or m1..m8)       -> 1..8
    Sauce1..Sauce15 (or s1..s15)     -> 101..115
    Numeric IDs are accepted too.

  CLI:
    status
    clear
    abort
    rinse <seconds>
    trigger <motorId|name> <seconds>
    dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>

  Behavior:
    • DISPENSE/RINSE/TRIGGER are async on the slave.
    • Master does NOT block. If a start cmd returns OK+ACTIVE, it records a pending op
      and silently polls STATUS every ~150 ms until not ACTIVE.
    • While pending, CLI stays responsive: 'abort' preempts immediately; 'status' prints a snapshot.
    • One final outcome line per op: "[cmd] done." or "[cmd] ended: <FAULT/ABORTED>".
*/

#include <Arduino.h>
#include <string.h>

// ---------------- RS485 link ----------------
#define RS485_EN_PIN 2              // RE/DE tied here (HIGH=TX, LOW=RX)
#define RS485       Serial1         // bus UART
#define BUS_BAUD    19200

// ---------------- Modbus constants ----------------
#define SLAVE_ID  1
#define MB_FUNC_RW_MREGS 0x17

// Register map
#define REG_CMD_BASE  0x0000
#define REG_RES_BASE  0x0100

// Result decode
enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };
enum ApiError   : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};
enum Opcode     : uint16_t { OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_DISPENSE=10, OP_RINSE=11, OP_TRIGGER=12 };

// We always read full result block 0x0100..0x010A (11 regs)
const uint16_t RES_QTY = 0x000B;

// Poll cadence for silent loops
const uint16_t STATUS_POLL_MS = 150;

// ---- Timing: 3.5 chars (not held while DE is high) ----
const uint16_t TCHAR_US = (uint16_t)( (11.0 / BUS_BAUD) * 1e6 + 0.5 ); // ~573 us @19200
const uint16_t T3P5_US  = (uint16_t)( 3.5 * TCHAR_US );                // ~2.0 ms

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

bool readExact(uint8_t* buf, size_t n, uint32_t deadlineMs) {
  size_t got = 0;
  while ((long)(deadlineMs - millis()) >= 0 && got < n) {
    int b = RS485.read();
    if (b >= 0) buf[got++] = (uint8_t)b;
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

// ---------------------------------------------
// Core 0x17 transaction
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
  frame[idx++] = (uint8_t)(crc & 0xFF);
  frame[idx++] = (uint8_t)(crc >> 8);

  purgeRx();
  delayMicroseconds(T3P5_US);

  rs485Tx();
  delayMicroseconds(10);
  RS485.write(frame, idx);
  RS485.flush();
  rs485Rx();
  delayMicroseconds(50);

  // Expect: addr, func, byteCount, data(byteCount), crcLo, crcHi
  const uint16_t expectDataBytes = (uint16_t)(readQty * 2);

  uint8_t hdr[3];
  uint32_t deadline = millis() + overallTimeoutMs;
  if (!readExact(hdr, 3, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on header"));
    return false;
  }

  // Exception?
  if ((hdr[1] & 0x80) && hdr[0] == slaveId) {
    uint8_t exCrc[2];
    (void)readExact(exCrc, 2, deadline);
    if (verboseOnError) {
      Serial.print(F("[modbus] exception code=0x")); Serial.println(hdr[2], HEX);
    }
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
    return false;
  }

  // Read payload + CRC
  const uint16_t rem = expectDataBytes + 2;
  uint8_t rest[300];
  if (!readExact(rest, rem, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on payload"));
    return false;
  }

  // CRC check
  uint8_t full[3 + 300];
  memcpy(full, hdr, 3);
  memcpy(full+3, rest, rem);
  uint16_t gotCrc = (uint16_t)(rest[rem-2] | (rest[rem-1] << 8));
  uint16_t calcCrc = mb_crc16(full, (uint16_t)(3 + expectDataBytes));
  if (gotCrc != calcCrc) {
    if (verboseOnError) {
      Serial.println(F("[modbus] CRC mismatch; header+data dump:"));
      hexDump(full, 3 + expectDataBytes);
      Serial.print(F("gotCrc=0x")); Serial.println(gotCrc, HEX);
      Serial.print(F("calculated=0x")); Serial.println(calcCrc, HEX);
    }
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
// Pretty-print helpers (brief: no weight, no scale)
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

const char* statusToStr(uint16_t s) {
  switch (s) {
    case 0: return "IDLE";
    case 1: return "ACTIVE";
    case 2: return "MOTOR_FAULT";
    case 3: return "LEAK_FAULT";
    case 4: return "SCALE_FAULT";
    case 5: return "TIMEOUT_FAULT";
    case 6: return "ABORTED_FAULT";
    default: return "?";
  }
}

// Brief result line (no weight/scale)
void printResultBrief(const uint16_t* R) {
  uint16_t resultCode    = R[0];
  uint16_t errorCode     = R[1];
  uint16_t systemStatus  = R[2];

  Serial.print(F("result="));
  Serial.print(resultCode==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errToStr(errorCode));
  Serial.print(F("  status=")); Serial.print(statusToStr(systemStatus));
  Serial.println();
}

// ---------------------------------------------
// Motor token <-> numeric ID
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

// motorId -> "MilkX"/"SauceY"
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
// [sent] datalog echoes (use motor NAME, not numeric ID)
// ---------------------------------------------
static void logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void logSent_RINSE(float seconds) {
  Serial.print(F("-------------\n[sent] RINSE seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_TRIGGER(uint16_t motorId, float seconds) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] TRIGGER motor="));
  Serial.print(name);
  Serial.print(F(" seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_DISPENSE(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] DISPENSE motor="));
  Serial.print(name);
  Serial.print(F(" target_g=")); Serial.print(target_g, 1);
  Serial.print(F(" slowOffset_g=")); Serial.print(slow_g, 1);
  Serial.print(F(" softCutOffset_g=")); Serial.print(soft_g, 1);
  Serial.print(F(" timeout_s=")); Serial.print(timeout_s); Serial.println(F("."));
}

// ---------------------------------------------
// API wrappers  (seq not used; write seq=0)
// ---------------------------------------------
bool api_status(uint16_t* outRegs, uint32_t timeoutMs=600) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS;
  W[1] = 0;              // seq=0
  // caller chooses whether to log [sent] STATUS
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_status_logged(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  logSent_STATUS();
  return api_status(outRegs, timeoutMs);
}

bool api_clear(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[8] = {0};
  W[0] = OP_CLEAR;
  W[1] = 0;              // seq=0
  logSent_CLEAR();
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_abort(uint16_t* outRegs, uint32_t timeoutMs=800) {
  uint16_t W[8] = {0};
  W[0] = OP_ABORT;
  W[1] = 0;
  logSent_ABORT();
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_rinse(float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OP_RINSE;
  W[1] = 0;              // seq=0
  W[2] = seconds_x10;
  logSent_RINSE(seconds);
  uint32_t toMs = 2000; // quick acceptance
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 3, outRegs, RES_QTY, toMs);
}

bool api_trigger(uint16_t motorId, float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OP_TRIGGER;
  W[1] = 0;              // seq=0
  W[2] = motorId;
  W[3] = seconds_x10;
  logSent_TRIGGER(motorId, seconds);
  uint32_t toMs = 2000; // quick acceptance
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 4, outRegs, RES_QTY, toMs);
}

bool api_dispense(uint16_t motorId, float target_g, float slowOffset_g, float softCutOffset_g, uint16_t timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OP_DISPENSE;
  W[1] = 0;              // seq=0
  W[2] = motorId;
  W[3] = to_x10(target_g);
  W[4] = to_x10(slowOffset_g);
  W[5] = to_x10(softCutOffset_g);
  W[6] = timeout_s;

  logSent_DISPENSE(motorId, target_g, slowOffset_g, softCutOffset_g, timeout_s);

  uint32_t toMs = 2000; // quick acceptance
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 7, outRegs, RES_QTY, toMs);
}

// ---------------------------------------------
// Pending operation scheduler (non-blocking)
// ---------------------------------------------
enum : uint8_t { PK_NONE=0, PK_RINSE, PK_TRIGGER, PK_DISPENSE };

struct PendingOp {
  uint8_t kind = PK_NONE;
  uint32_t deadline = 0;       // absolute ms when we warn timeout (but keep polling)
  uint32_t nextPollDue = 0;    // cadence
  bool printedTimeout = false; // print timeout once
} pending;

bool abortRequested = false;

const char* pendingLabel(uint8_t k) {
  switch (k) {
    case PK_RINSE:    return "rinse";
    case PK_TRIGGER:  return "trigger";
    case PK_DISPENSE: return "dispense";
    default:          return "?";
  }
}

void armPending(uint8_t k, uint32_t extraMs) {
  pending.kind = k;
  pending.printedTimeout = false;
  pending.nextPollDue = millis() + STATUS_POLL_MS;
  pending.deadline = millis() + extraMs;
}

void clearPending() {
  pending.kind = PK_NONE;
  pending.deadline = 0;
  pending.nextPollDue = 0;
  pending.printedTimeout = false;
}

// One silent STATUS poll tick; returns true if got response and filled R
bool pollStatusOnce(uint16_t* R) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS; W[1] = 0;
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, R, RES_QTY, 600, /*verboseOnError*/false);
}

void servicePending() {
  if (pending.kind == PK_NONE) return;

  uint32_t now = millis();

  // Abort request has priority over polling
  if (abortRequested) {
    uint16_t R[RES_QTY] = {0};
    bool ok = api_abort(R, 800);
    abortRequested = false;
    if (ok) {
      printResultBrief(R); // immediate feedback for the operator
      // Keep polling to observe transition to ABORTED_FAULT (or other not ACTIVE)
      pending.nextPollDue = now + 100;
    } else {
      Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
    }
    return;
  }

  // Not time to poll yet
  if ((long)(now - pending.nextPollDue) < 0) return;

  // Silent STATUS poll
  uint16_t R[RES_QTY] = {0};
  bool ok = pollStatusOnce(R);
  if (!ok) {
    // Missed poll — just try again next cadence
    pending.nextPollDue = millis() + STATUS_POLL_MS;
    return;
  }

  uint16_t st = R[2];

  if (st != 1) { // not ACTIVE -> finalize
    if (st == 0) {
      Serial.print('['); Serial.print(pendingLabel(pending.kind)); Serial.println(F("] done."));
    } else {
      Serial.print('['); Serial.print(pendingLabel(pending.kind)); Serial.print(F("] ended: "));
      Serial.println(statusToStr(st));
    }
    clearPending();
    return;
  }

  // Still ACTIVE; check deadline (print ONCE)
  if (!pending.printedTimeout && (long)(now - pending.deadline) > 0) {
    Serial.print('['); Serial.print(pendingLabel(pending.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    pending.printedTimeout = true; // keep polling silently afterward
  }

  pending.nextPollDue = millis() + STATUS_POLL_MS;
}

// ---------------------------------------------
// Simple USB-serial CLI
// ---------------------------------------------
String inLine;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  status"));
  Serial.println(F("  clear"));
  Serial.println(F("  abort"));
  Serial.println(F("  rinse <seconds>"));
  Serial.println(F("  trigger <motorId|name> <seconds>   e.g. 'trigger 102 1.0' or 'trigger Sauce2 1.0'"));
  Serial.println(F("  dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
}

void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) {}

  RS485.begin(BUS_BAUD);     // 8N1 default
  delay(50);

  Serial.println(F("\nNano Every Modbus Master ready."));
  printHelp();

  // Quick probe
  uint16_t R[RES_QTY] = {0};
  Serial.print(F("\n[boot] Probing STATUS...\n"));
  bool ok = api_status_logged(R, 1500);
  if (ok) { Serial.println(F("Modbus OK")); printResultBrief(R); }
  else    { Serial.println(F("NO RESPONSE / CRC")); }
}

void loop() {
  // Service background pending operation at steady cadence
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

      if (low == "help") {
        printHelp();
      }
      else if (low == "status") {
        ok = api_status_logged(R);
        if (ok) printResultBrief(R);
        else    Serial.println(F("[status] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "clear") {
        ok = api_clear(R);
        if (ok) printResultBrief(R);
        else    Serial.println(F("[clear] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "abort") {
        if (pending.kind != PK_NONE) abortRequested = true;
        ok = api_abort(R, 800);
        if (ok) printResultBrief(R);
        else    Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
      }
      else if (low.startsWith("rinse ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          String s = cmd.substring(6); s.trim();
          float secs = s.toFloat();
          ok = api_rinse(secs, R);
          if (!ok) {
            Serial.println(F("[rinse] NO RESPONSE / CRC/timeout"));
          } else {
            printResultBrief(R);
            if (R[0]==RC_OK && R[2]==1) {
              uint32_t guard = 5000UL;
              armPending(PK_RINSE, (uint32_t)(secs*1000.0f) + guard);
            }
          }
        }
      }
      else if (low.startsWith("trigger ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          String rest = cmd.substring(8); rest.trim();
          int sp = rest.indexOf(' ');
          if (sp < 0) {
            Serial.println(F("Use: trigger <motorId|name> <seconds>"));
          } else {
            String tok  = rest.substring(0, sp); tok.trim();
            String ssec = rest.substring(sp+1); ssec.trim();
            uint16_t motorId;
            if (!motorTokenToId(tok, motorId)) { Serial.println(F("[err] unknown motor token")); continue; }
            float secs = ssec.toFloat();

            ok = api_trigger(motorId, secs, R);
            if (!ok) {
              Serial.println(F("[trigger] NO RESPONSE / CRC/timeout"));
            } else {
              printResultBrief(R);
              if (R[0]==RC_OK && R[2]==1) {
                uint32_t guard = 5000UL;
                armPending(PK_TRIGGER, (uint32_t)(secs*1000.0f) + guard);
              }
            }
          }
        }
      }
      else if (low.startsWith("dispense ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          String rest = cmd.substring(9); rest.trim();
          int sp1 = rest.indexOf(' ');
          int sp2 = rest.indexOf(' ', sp1+1);
          int sp3 = rest.indexOf(' ', sp2+1);
          int sp4 = rest.indexOf(' ', sp3+1);
          if (sp1<0 || sp2<0 || sp3<0 || sp4<0) {
            Serial.println(F("Use: dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
          } else {
            String tok       = rest.substring(0, sp1); tok.trim();
            float target_g   = rest.substring(sp1+1, sp2).toFloat();
            float slow_g     = rest.substring(sp2+1, sp3).toFloat();
            float soft_g     = rest.substring(sp3+1, sp4).toFloat();
            uint16_t tout_s  = (uint16_t)rest.substring(sp4+1).toInt();

            uint16_t motorId;
            if (!motorTokenToId(tok, motorId)) { Serial.println(F("[err] unknown motor token")); continue; }

            ok = api_dispense(motorId, target_g, slow_g, soft_g, tout_s, R);
            if (!ok) {
              Serial.println(F("[dispense] NO RESPONSE / CRC/timeout"));
            } else {
              printResultBrief(R);
              if (R[0]==RC_OK && R[2]==1) {
                uint32_t guard = 10000UL; // extra for settle/coast/comm
                armPending(PK_DISPENSE, (uint32_t)tout_s*1000UL + guard);
              }
            }
          }
        }
      }
      else {
        Serial.println(F("Unknown command. Type 'help'."));
      }

      // After each command line, immediately service pending once to keep it snappy
      servicePending();

    } else {
      if (inLine.length() < 120) inLine += c;
    }
  }

  // Idle tick: service pending again if due
  servicePending();
}
