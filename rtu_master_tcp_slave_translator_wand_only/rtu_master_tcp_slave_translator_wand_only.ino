/*
  Arduino Nano Every — Modbus RTU Master (0x17) for Steam Wand Cleaner
  RS-485: MAX485 — RO->D0(RX1), DI->D1(TX1), RE+DE->D2 (HIGH=TX, LOW=RX)
  Bus: 19200 8N1, Slave ID = 2

  CLI:
    status
    clear
    abort
    off
    init  <seconds>
    froth <targetC> <timeout_s>
    clean <tValve_s> <tSteam_s> <tStandby_s>

  Behavior:
    • INIT/FROTH/CLEAN are async on the slave.
    • Master does NOT block. On acceptance (OK+ACTIVE), it records a pending op
      and silently polls STATUS every ~150 ms until not ACTIVE.
    • While pending, CLI stays responsive: 'abort' preempts immediately; 'status' prints a snapshot.
    • One final outcome line per op: "[cmd] done." or "[cmd] ended: <FAULT/ABORTED/OFF>".

  Robustness:
    • If a start command's reply has CRC/timeout, master immediately probes STATUS.
      If STATUS shows ACTIVE, it treats the command as accepted and proceeds.

  Protocol note:
    • Expects slave to report SYSTEM_STATUS=7 for OFF (distinct from IDLE).
*/

#include <Arduino.h>
#include <string.h>

/* ---------------- RS485 link ---------------- */
#define RS485_EN_PIN 2              // RE/DE tied here (HIGH=TX, LOW=RX)
#define RS485       Serial1         // bus UART
#define BUS_BAUD    19200

/* ---------------- Modbus constants ---------------- */
#define SLAVE_ID  2
#define MB_FUNC_RW_MREGS 0x17

/* Register map */
#define REG_CMD_BASE  0x0000
#define REG_RES_BASE  0x0100

/* Result decode */
enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };
enum ApiError   : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};
enum Opcode     : uint16_t { OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_OFF=4, OP_INIT=10, OP_FROTH=11, OP_CLEAN=12 };

/* We always read full result block 0x0100..0x010A (11 regs) */
const uint16_t RES_QTY = 0x000B;

/* Poll cadence for silent loops */
const uint16_t STATUS_POLL_MS = 150;

/* ---- Timing: RTU 8N1 uses 10 bits/char ---- */
constexpr float   BITS_PER_CHAR = 10.0f;                         // 8N1: 1 start + 8 data + 1 stop
const uint16_t    TCHAR_US      = (uint16_t)((BITS_PER_CHAR / BUS_BAUD) * 1e6 + 0.5f); // ≈521 µs @19200
const uint16_t    T2_US         = (uint16_t)(2.0f * TCHAR_US);   // 2 chars (kept for reference)
const uint16_t    T3P5_US       = (uint16_t)(3.5f * TCHAR_US);   // spec gap

/* ---------------------------------------------
   CRC16 (Modbus) poly 0xA001, init 0xFFFF
   --------------------------------------------- */
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

/* ---------------------------------------------
   RS485 direction control
   --------------------------------------------- */
inline void rs485Rx() { digitalWrite(RS485_EN_PIN, LOW); }
inline void rs485Tx() { digitalWrite(RS485_EN_PIN, HIGH); }

/* ---------------------------------------------
   Utils
   --------------------------------------------- */
void purgeRx() { while (RS485.available()) (void)RS485.read(); }

// Gentle read loop: add tiny delay to avoid busy-spin and give UART time to fill
bool readExact(uint8_t* buf, size_t n, uint32_t deadlineMs) {
  size_t got = 0;
  while ((long)(deadlineMs - millis()) >= 0 && got < n) {
    int b = RS485.read();
    if (b >= 0) {
      buf[got++] = (uint8_t)b;
    } else {
      delayMicroseconds(50);  // small yield
    }
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

/* ---------------------------------------------
   Core 0x17 transaction
   --------------------------------------------- */
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

  purgeRx();
  delayMicroseconds(T3P5_US);

  rs485Tx();
  delayMicroseconds(10);
  RS485.write(frame, idx);
  RS485.flush();

  // Turnaround hardening (IMPLEMENTED #1):
  // Wait 3 chars after TX so the last stop bit clears and line settles,
  // then drop DE and wait another 3 chars before expecting bytes.
  delayMicroseconds(3 * TCHAR_US);
  rs485Rx();
  delayMicroseconds(3 * TCHAR_US);

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
    return false;
  }

  // Unpack data to regs
  if (respRegsLen < readQty) return false;
  for (uint16_t i=0;i<readQty;i++) {
    respRegs[i] = (uint16_t)((rest[2*i] << 8) | rest[2*i+1]);
  }
  return true;
}

/* ---------------------------------------------
   Pretty-print helpers (adds temp & elapsed when present)
   --------------------------------------------- */
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

// OFF is 7 (distinct from IDLE)
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

void printResultBrief(const uint16_t* R) {
  uint16_t resultCode    = R[0];
  uint16_t errorCode     = R[1];
  uint16_t systemStatus  = R[2];
  uint16_t aux0_tempx10  = R[4];
  uint16_t aux1_elapx10  = R[5];

  Serial.print(F("result="));
  Serial.print(resultCode==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errToStr(errorCode));
  Serial.print(F("  status=")); Serial.print(statusToStr(systemStatus));

  if (aux0_tempx10 != 0) {
    Serial.print(F("  tempC="));
    Serial.print(aux0_tempx10 / 10.0f, 1);
  }
  if (aux1_elapx10 != 0) {
    Serial.print(F("  elapsed_s="));
    Serial.print(aux1_elapx10 / 10.0f, 1);
  }
  Serial.println();
}

/* ---------------------------------------------
   [sent] datalog echoes
   --------------------------------------------- */
static void logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void logSent_OFF()    { Serial.println(F("-------------\n[sent] OFF.")); }
static void logSent_INIT(float seconds) {
  Serial.print(F("-------------\n[sent] INIT seconds=")); Serial.print(seconds, 1); Serial.println(F("."));
}
static void logSent_FROTH(float targetC, float timeout_s) {
  Serial.print(F("-------------\n[sent] FROTH targetC=")); Serial.print(targetC, 1);
  Serial.print(F("C timeout_s=")); Serial.print(timeout_s, 1); Serial.println(F("."));
}
static void logSent_CLEAN(float tValve, float tSteam, float tStandby) {
  Serial.print(F("-------------\n[sent] CLEAN tValve_s=")); Serial.print(tValve, 1);
  Serial.print(F(" tSteam_s=")); Serial.print(tSteam, 1);
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby, 1); Serial.println(F("."));
}

/* ---------------------------------------------
   API wrappers  (seq not used; write seq=0)
   --------------------------------------------- */
bool api_status(uint16_t* outRegs, uint32_t timeoutMs=600) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS;
  W[1] = 0;
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
  W[1] = 0;
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

bool api_off(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[8] = {0};
  W[0] = OP_OFF;
  W[1] = 0;
  logSent_OFF();
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

// --- Salvage helper: if last command's reply failed, check STATUS for ACTIVE
static bool salvage_acceptance(uint16_t* outRegs, uint8_t pendingKindLabel){
  uint16_t S[RES_QTY] = {0};
  bool ok = api_status(S, 600);
  if (ok && S[2] == 1 /*ACTIVE*/) {
    for (uint16_t i=0;i<RES_QTY;i++) outRegs[i] = S[i];
    Serial.print(F("[salvage] reply CRC/timeout, but live STATUS=ACTIVE; proceeding ("));
    switch(pendingKindLabel){
      case 1: Serial.print(F("init")); break;
      case 2: Serial.print(F("froth")); break;
      case 3: Serial.print(F("clean")); break;
      default: Serial.print(F("op")); break;
    }
    Serial.println(F(")."));
    return true;
  }
  return false;
}

// (IMPLEMENTED #2): acceptance timeout = 2500 ms
bool api_init(float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OP_INIT;
  W[1] = 0;
  W[2] = seconds_x10;
  logSent_INIT(seconds);
  uint32_t toMs = 2500; // was 2000
  bool ok = mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 3, outRegs, RES_QTY, toMs);
  if (!ok) {
    if (salvage_acceptance(outRegs, /*init*/1)) return true;
  }
  return ok;
}

bool api_froth(float targetC, float timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OP_FROTH;
  W[1] = 0;
  W[2] = to_x10(targetC);
  W[3] = to_x10(timeout_s);
  logSent_FROTH(targetC, timeout_s);
  uint32_t toMs = 2500; // was 2000
  bool ok = mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 4, outRegs, RES_QTY, toMs);
  if (!ok) {
    if (salvage_acceptance(outRegs, /*froth*/2)) return true;
  }
  return ok;
}

bool api_clean(float tValve, float tSteam, float tStandby, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OP_CLEAN;
  W[1] = 0;
  W[2] = to_x10(tValve);
  W[3] = to_x10(tSteam);
  W[4] = to_x10(tStandby);
  logSent_CLEAN(tValve, tSteam, tStandby);
  uint32_t toMs = 2500; // was 2000
  bool ok = mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                                REG_CMD_BASE, W, 5, outRegs, RES_QTY, toMs);
  if (!ok) {
    if (salvage_acceptance(outRegs, /*clean*/3)) return true;
  }
  return ok;
}

/* ---------------------------------------------
   Pending operation scheduler (non-blocking)
   --------------------------------------------- */
enum : uint8_t { PK_NONE=0, PK_INIT, PK_FROTH, PK_CLEAN };

struct PendingOp {
  uint8_t kind = PK_NONE;
  uint32_t deadline = 0;       // absolute ms when we warn timeout (but keep polling)
  uint32_t nextPollDue = 0;    // cadence
  bool printedTimeout = false; // print timeout once
} pending;

bool abortRequestedFlag = false;

const char* pendingLabel(uint8_t k) {
  switch (k) {
    case PK_INIT:   return "init";
    case PK_FROTH:  return "froth";
    case PK_CLEAN:  return "clean";
    default:        return "?";
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
  if (abortRequestedFlag) {
    uint16_t R[RES_QTY] = {0};
    bool ok = api_abort(R, 800);
    abortRequestedFlag = false;
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

/* ---------------------------------------------
   Simple USB-serial CLI
   --------------------------------------------- */
String inLine;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  status"));
  Serial.println(F("  clear"));
  Serial.println(F("  abort"));
  Serial.println(F("  off"));
  Serial.println(F("  init  <seconds>"));
  Serial.println(F("  froth <targetC> <timeout_s>"));
  Serial.println(F("  clean <tValve_s> <tSteam_s> <tStandby_s>"));
}

void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) {}

  RS485.begin(BUS_BAUD);     // 8N1 default
  delay(50);

  Serial.println(F("\nNano Every Modbus Master (Steam Wand) ready (3-char RX guard + 2.5s acceptance)."));
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
        if (pending.kind != PK_NONE) abortRequestedFlag = true;
        ok = api_abort(R, 800);
        if (ok) printResultBrief(R);
        else    Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "off") {
        ok = api_off(R);
        if (ok) printResultBrief(R);
        else    Serial.println(F("[off] NO RESPONSE / CRC/timeout"));
      }
      else if (low.startsWith("init ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          String s = cmd.substring(5); s.trim();
          float secs = s.toFloat();
          ok = api_init(secs, R);
          if (!ok) {
            Serial.println(F("[init] NO RESPONSE / CRC/timeout"));
          } else {
            printResultBrief(R);
            if (R[0]==RC_OK && R[2]==1) {
              uint32_t guard = 5000UL;
              armPending(PK_INIT, (uint32_t)(secs*1000.0f) + guard);
            }
          }
        }
      }
      else if (low.startsWith("froth ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          int sp = low.indexOf(' ', 6);
          if (sp < 0) {
            Serial.println(F("Use: froth <targetC> <timeout_s>"));
          } else {
            float targetC = cmd.substring(6, sp).toFloat();
            float tout    = cmd.substring(sp+1).toFloat();
            ok = api_froth(targetC, tout, R);
            if (!ok) {
              Serial.println(F("[froth] NO RESPONSE / CRC/timeout"));
            } else {
              printResultBrief(R);
              if (R[0]==RC_OK && R[2]==1) {
                uint32_t guard = 10000UL; // extra cushion
                armPending(PK_FROTH, (uint32_t)(tout*1000.0f) + guard);
              }
            }
          }
        }
      }
      else if (low.startsWith("clean ")) {
        if (pending.kind != PK_NONE) {
          Serial.println(F("[busy] operation in progress; type 'abort' or 'status'."));
        } else {
          String rest = cmd.substring(6); rest.trim();
          int s1 = rest.indexOf(' ');
          if (s1<0) { Serial.println(F("Use: clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _after; }
          int s2 = rest.indexOf(' ', s1+1);
          if (s2<0) { Serial.println(F("Use: clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _after; }
          float tValve = rest.substring(0, s1).toFloat();
          float tSteam = rest.substring(s1+1, s2).toFloat();
          float tStby  = rest.substring(s2+1).toFloat();

          ok = api_clean(tValve, tSteam, tStby, R);
          if (!ok) {
            Serial.println(F("[clean] NO RESPONSE / CRC/timeout"));
          } else {
            printResultBrief(R);
            if (R[0]==RC_OK && R[2]==1) {
              uint32_t guard = 10000UL;
              uint32_t total = (uint32_t)((tValve + tSteam + tStby)*1000.0f) + guard;
              armPending(PK_CLEAN, total);
            }
          }
        }
      }
      else {
        Serial.println(F("Unknown command. Type 'help'."));
      }

_after:
      // After each command line, immediately service pending once to keep it snappy
      servicePending();

    } else {
      if (inLine.length() < 120) inLine += c;
    }
  }

  // Idle tick: service pending again if due
  servicePending();
}
