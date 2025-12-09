/*
  ┌────────────────────────────────────────────────────────────────────────────┐
  │ Modbus RTU Master — Dispenser(ID=1) + Cleaner(ID=2) over single RS-485 bus │
  │ Target MCU: Arduino Nano Every (megaAVR 4809)                               │
  └────────────────────────────────────────────────────────────────────────────┘

  Version : 1.1 (anti-idle-spam logging + built-in quick guide)
  Date    : 2025-10-21  (Asia/Riyadh)
  Authors : Erdie Gange · ChatGPT 5

  What this firmware does
  ───────────────────────
  • Drives two Modbus RTU slaves sharing one half-duplex RS-485 line:
      - Dispenser = Slave ID 1 (Mega2560)
      - Cleaner   = Slave ID 2 (Nano Every)
  • Uses only Function 0x17 (Read/Write Multiple Registers) as per both slaves.
  • Per-slave SEQ counters, retry on CRC/timeout, and T3.5 gap compliance.
  • Non-blocking bus scheduler: polls IDLE devices slowly, ACTIVE devices faster.
  • USB CLI for manual control (type `help`). A quick guide is printed on boot.

  Bus / Wiring
  ────────────
  • MAX485 (or equiv.) with DE and /RE tied together to Nano Every pin D2.
  • UART: Serial1 (pins RX1/TX1) → MAX485 RO/DI.
  • Line timing: 19,200 bps, 8N1. T3.5≈2.0 ms. TX turnaround guard ≈150 µs.

  Anti-idle spam logging
  ──────────────────────
  • Status polls log **only** when:
      - system status changes, OR
      - system is non-IDLE, OR
      - RC/ERR is nonzero, OR
      - we’re recovering after a comms fault.
  • Command replies from the CLI are still logged once for visibility.

  CLI Commands (USB Serial at 115200)
  ───────────────────────────────────
  • Dispenser (ID=1)
      d.status
      d.clear
      d.abort
      d.rinse <seconds>
      d.trigger <motorId> <seconds>
      d.dispense <motorId> <target_g> <slowOffset_g> <softCut_g> <timeout_s>

  • Cleaner (ID=2)
      c.status
      c.clear
      c.abort
      c.off
      c.init  <seconds>
      c.froth <targetC> <timeout_s>
      c.clean <tValve> <tSteam> <tStandby>

  Examples
  ────────
      d.dispense 101 120 20 5 45
      d.rinse 2.5
      c.froth 65 40
      c.clean 1.0 3.0 2.0
  ┌──────────────────────────────────────────────────────────────────────────────┐
  │ Arduino Nano Every — Modbus RTU Master (0x17) for Dispenser(ID=1) + Cleaner(ID=2)
  │ RS-485: MAX485 — RO->D0(RX1), DI->D1(TX1), RE+DE->D2 (HIGH=TX, LOW=RX)
  └──────────────────────────────────────────────────────────────────────────────┘

  Bus: 19200 8N1

  Dispenser (ID=1) CLI:
    d.status
    d.clear
    d.abort
    d.rinse <seconds>
    d.trigger <motorId|name> <seconds>
    d.dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>

  Motor tokens:
    Milk1..Milk8   (or m1..m8)       -> 1..8
    Sauce1..Sauce15 (or s1..s15)     -> 101..115
    Numeric IDs are accepted too.

  Cleaner (ID=2) CLI:
    c.status
    c.clear
    c.abort
    c.off
    c.init  <seconds>
    c.froth <targetC> <timeout_s>
    c.clean <tValve_s> <tSteam_s> <tStandby_s>

  Behavior:
    • Asynchronous ops on the slaves; master does not block.
    • On acceptance (OK+ACTIVE), master records a pending op and silently polls STATUS every ~150 ms.
    • While pending: CLI stays responsive; '...status' prints a snapshot; '...abort' preempts immediately.
    • Exactly one final outcome line per op: "[cmd] done." or "[cmd] ended: <FAULT/ABORTED/OFF>".
    • Idle: no datalog spam.

  Robustness:
    • If a start command reply has CRC/timeout, master probes STATUS.
      If STATUS shows ACTIVE, it treats the command as accepted (salvage path).

  Comms:
    • Modbus RTU function 0x17 (Read/Write Multiple Registers) on both nodes.
    • Same transaction packing/timing style as your stand-alone masters.
*/

// --------- Fix for Arduino's auto-prototype generation ---------
struct Device;           // forward declare so prototypes see this type
#define RES_BASE   0x0100
#define RES_COUNT  11
// ---------------------------------------------------------------

#include <Arduino.h>
#include <string.h>
#include <math.h>

/* ========================= Modbus / RS-485 config ==================== */
#define MODBUS_BAUD   19200
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_PORT   Serial1

// DE and /RE tied together to this single pin (HIGH=TX, LOW=RX)
const uint8_t RS485_DE_RE_PIN = 2;
/* ---------------- RS485 link ---------------- */
#define RS485_EN_PIN 2              // RE/DE tied here (HIGH=TX, LOW=RX)
#define RS485       Serial1         // bus UART
#define BUS_BAUD    19200

// Character/gap timing at 19,200 bps (11-bit frames)
const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD;  // ≈573 µs
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);      // ≈2.0 ms
/* ---- Timing: 3.5 chars for inter-frame gap (as in your examples) ---- */
const uint16_t TCHAR_US = (uint16_t)((11.0f / BUS_BAUD) * 1000000.0f + 0.5f); // ~573 µs @19200 (conservative 11 bits)
const uint16_t T3P5_US  = (uint16_t)(3.5f * TCHAR_US);                        // ~2.0 ms

// Turnaround guard for MAX485 drivers before first TX byte
const uint16_t RS485_TX_GUARD_US = 150;
/* ---------------- Modbus constants ---------------- */
#define MB_FUNC_RW_MREGS 0x17

// Master side timeouts/retries
const uint16_t MB_RESP_TIMEOUT_MS = 80;   // per request
const uint8_t  MB_MAX_RETRIES     = 2;    // in addition to first try
/* Common register map for both slaves */
#define REG_CMD_BASE  0x0000
#define REG_RES_BASE  0x0100
const uint16_t RES_QTY = 0x000B; // 0x0100..0x010A

// Frame buffers
#define MB_MAX_FRAME  260
/* Dispenser IDs & opcodes */
#define DISP_SLAVE_ID     1
enum DispOpcode : uint16_t { D_OP_STATUS=1, D_OP_CLEAR=2, D_OP_ABORT=3, D_OP_DISPENSE=10, D_OP_RINSE=11, D_OP_TRIGGER=12 };

/* ========================= Slave IDs & register map =================== */
#define SLAVE_ID_DISPENSER  1
#define SLAVE_ID_CLEANER    2
/* Cleaner IDs & opcodes */
#define CLEAN_SLAVE_ID    2
enum CleanOpcode : uint16_t { C_OP_STATUS=1, C_OP_CLEAR=2, C_OP_ABORT=3, C_OP_OFF=4, C_OP_INIT=10, C_OP_FROTH=11, C_OP_CLEAN=12 };

// Common command block (write)
#define REG_OPCODE   0x0000
#define REG_SEQ      0x0001

// Opcodes (shared semantics)
enum Opcode : uint16_t {
  OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_OFF=4,
  OP_DISPENSE=10, OP_RINSE=11, OP_TRIGGER=12,      // Dispenser
  OP_INIT=10, OP_FROTH=11, OP_CLEAN=12             // Cleaner (overlaps IDs intentionally)
/* Result decode */
enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };
enum ApiError   : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};

enum SystemStatus : uint16_t {
  SYS_IDLE=0, SYS_ACTIVE=1, SYS_MOTOR_FAULT=2, SYS_LEAK_FAULT=3, SYS_SCALE_FAULT=4,
  SYS_TIMEOUT_FAULT=5, SYS_ABORTED_FAULT=6, SYS_OFF=7
  SYS_IDLE=0, SYS_ACTIVE=1, SYS_MOTOR_FAULT=2, SYS_LEAK_FAULT=3, SYS_SCALE_FAULT=4, SYS_TIMEOUT_FAULT=5, SYS_ABORTED_FAULT=6, SYS_OFF=7
};

static const __FlashStringHelper* sysStr(uint16_t s){
  switch(s){
    case SYS_IDLE: return F("IDLE");
    case SYS_ACTIVE: return F("ACTIVE");
    case SYS_MOTOR_FAULT: return F("MOTOR_FAULT");
    case SYS_LEAK_FAULT:  return F("LEAK_FAULT");
    case SYS_SCALE_FAULT: return F("SCALE_FAULT");
    case SYS_TIMEOUT_FAULT: return F("TIMEOUT_FAULT");
    case SYS_ABORTED_FAULT: return F("ABORTED_FAULT");
    case SYS_OFF: return F("OFF");
    default: return F("UNKNOWN");
  }
}
/* Poll cadence for silent loops */
const uint16_t STATUS_POLL_MS = 150;

/* ========================= RS-485 helpers ============================= */
inline void rs485RxMode(){ digitalWrite(RS485_DE_RE_PIN, LOW); }
inline void rs485TxMode(){ digitalWrite(RS485_DE_RE_PIN, HIGH); delayMicroseconds(RS485_TX_GUARD_US); }

/* ========================= CRC-16/Modbus ============================== */
uint16_t mb_crc16(const uint8_t* data, uint16_t len){
/* ---------------------------------------------
   CRC16 (Modbus) poly 0xA001, init 0xFFFF
   --------------------------------------------- */
uint16_t mb_crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i=0;i<len;i++){
    crc ^= data[i];
    for (uint8_t b=0;b<8;b++){
      if (crc & 1) { crc = (crc>>1) ^ 0xA001; }
      else crc >>= 1;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

/* ========================= Device state & scheduler =================== */
struct Device {
  uint8_t id;
  const char* name;
  uint16_t seq;
  uint16_t lastRC, lastERR, lastSYS, lastSEQ, lastAUX, lastELAPSEDx10;
  bool commsFault;
  unsigned long nextStatusDueMs;
};
Device DISP { SLAVE_ID_DISPENSER, "DISPENSER", 0, 0,0,0,0,0,0, false, 0 };
Device CLEAN{ SLAVE_ID_CLEANER,   "CLEANER",   0, 0,0,0,0,0,0, false, 0 };

// Poll cadence
const unsigned long POLL_ACTIVE_MS = 150;
const unsigned long POLL_IDLE_MS   = 800;

// Bus idle tracker for T3.5 enforcement
static unsigned long lastBusActivityUs = 0;
/* ---------------------------------------------
   RS485 direction control
   --------------------------------------------- */
inline void rs485Rx() { digitalWrite(RS485_EN_PIN, LOW); }
inline void rs485Tx() { digitalWrite(RS485_EN_PIN, HIGH); }

/* ---------------------------------------------
   Utils
   --------------------------------------------- */
void purgeRx() { while (RS485.available()) (void)RS485.read(); }

bool readExact(uint8_t* buf, size_t n, uint32_t deadlineMs) {
  size_t got = 0;
  while ((long)(deadlineMs - millis()) >= 0 && got < n) {
    int b = RS485.read();
    if (b >= 0) buf[got++] = (uint8_t)b;
    else delayMicroseconds(50);
  }
  return (got == n);
}

void waitT3p5GapBeforeTx(){
  unsigned long now = micros();
  unsigned long since = now - lastBusActivityUs;
  if (since < MB_T3P5_US){
    delayMicroseconds((unsigned int)(MB_T3P5_US - since));
void hexDump(const uint8_t* b, size_t n) {
  for (size_t i=0;i<n;i++) {
    if (i && (i%16)==0) Serial.println();
    if (b[i] < 16) Serial.print('0');
    Serial.print(b[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

/* ========================= 0x17 transaction helper =================== */
bool mb17_transact(uint8_t slaveId,
                   uint16_t readStart, uint16_t readQty,
                   uint16_t writeStart, const uint16_t* writeWords, uint16_t writeQty,
                   uint8_t* outData, uint16_t& outLen)
/* ---------------------------------------------
   Core 0x17 transaction (same packing/timing style you used)
   --------------------------------------------- */
bool mbReadWriteMultiple(uint8_t slaveId,
                         uint16_t readStart, uint16_t readQty,
                         uint16_t writeStart, const uint16_t* writeRegs, uint16_t writeQty,
                         uint16_t* respRegs, uint16_t respRegsLen,
                         uint32_t overallTimeoutMs,
                         bool verboseOnError = true)
{
  // -------- Build request frame --------
  uint8_t req[MB_MAX_FRAME];
  uint16_t idx = 0;
  req[idx++] = slaveId;
  req[idx++] = 0x17;
  const uint8_t func = MB_FUNC_RW_MREGS;
  const uint8_t writeByteCount = (uint8_t)(writeQty * 2);
  const uint16_t pduLen  = 1+1 +2+2 +2+2 +1 + writeByteCount; // no CRC
  const uint16_t frameLen = pduLen + 2;
  uint8_t frame[256];
  if (frameLen > sizeof(frame)) return false;

  auto put16 = [&](uint16_t v){
    req[idx++] = (uint8_t)(v >> 8);
    req[idx++] = (uint8_t)(v & 0xFF);
  };

  put16(readStart);   // Read starting address
  put16(readQty);     // Read quantity
  put16(writeStart);  // Write starting address
  put16(writeQty);    // Write quantity
  req[idx++] = (uint8_t)(writeQty * 2); // Byte count

  for (uint16_t i=0;i<writeQty;i++){
    put16(writeWords[i]);
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

  // CRC
  uint16_t crc = mb_crc16(req, idx);
  req[idx++] = (uint8_t)(crc & 0xFF);        // CRC lo
  req[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC hi

  // -------- Send with T3.5 and turnaround guard --------
  waitT3p5GapBeforeTx();
  rs485TxMode();
  MODBUS_PORT.write(req, idx);
  MODBUS_PORT.flush();
  rs485RxMode();

  // -------- Receive with gap-based end-of-frame --------
  uint8_t rx[MB_MAX_FRAME];
  uint16_t rxlen = 0;
  unsigned long startMs = millis();
  unsigned long lastByteUs = micros();

  // Wait for first byte within timeout
  while ((millis() - startMs) < MB_RESP_TIMEOUT_MS){
    if (MODBUS_PORT.available() > 0) break;
  }
  if (MODBUS_PORT.available() == 0){
    lastBusActivityUs = micros();
    return false; // timeout (no first byte)
  // Exception?
  if ((hdr[1] & 0x80) && hdr[0] == slaveId) {
    uint8_t exCrc[2];
    (void)readExact(exCrc, 2, deadline);
    if (verboseOnError) {
      Serial.print(F("[modbus] exception code=0x")); Serial.println(hdr[2], HEX);
    }
    return false;
  }

  // Read bytes until silence ≥ T3.5 or timeout
  while ((millis() - startMs) < MB_RESP_TIMEOUT_MS){
    while (MODBUS_PORT.available()){
      int b = MODBUS_PORT.read();
      if (b < 0) break;
      if (rxlen < MB_MAX_FRAME) rx[rxlen++] = (uint8_t)b;
      lastByteUs = micros();
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
    // End-of-frame detection (silent gap)
    if ((micros() - lastByteUs) >= MB_T3P5_US) break;
    return false;
  }
  lastBusActivityUs = micros();

  if (rxlen < 5) return false; // too short

  // Validate CRC
  uint16_t rxcrc = (uint16_t)(rx[rxlen-2] | ((uint16_t)rx[rxlen-1] << 8));
  uint16_t calc  = mb_crc16(rx, (uint16_t)(rxlen-2));
  if (rxcrc != calc) return false;

  // Exception?
  if (rx[0] != slaveId) return false;
  if (rx[1] == (0x80 | 0x17)){
    // rx[2] holds exception code; treat as failure
  // Read payload + CRC
  const uint16_t rem = expectDataBytes + 2;
  uint8_t rest[300];
  if (!readExact(rest, rem, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on payload"));
    return false;
  }
  if (rx[1] != 0x17) return false;

  // Parse data: [addr][0x17][byteCount][data...][CRC]
  uint8_t byteCount = rx[2];
  if (byteCount % 2) return false;
  if (byteCount != RES_COUNT*2) return false;

  // Copy out data bytes
  if (outData && byteCount <= MB_MAX_FRAME){
    memcpy(outData, &rx[3], byteCount);
    outLen = byteCount;
  } else {
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

/* Convert data bytes (big-endian word stream) to uint16 regs */
void bytesToRegsBE(const uint8_t* data, uint16_t countBytes, uint16_t* outRegs, uint16_t expectRegs){
  uint16_t n = countBytes / 2;
  if (n > expectRegs) n = expectRegs;
  for (uint16_t i=0;i<n;i++){
    outRegs[i] = (uint16_t)((data[2*i] << 8) | data[2*i+1]);
/* ---------------------------------------------
   Pretty-print helpers
   --------------------------------------------- */
const char* errStr(uint16_t e) {
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
const char* sysStr(uint16_t s) {
  switch (s) {
    case SYS_IDLE:           return "IDLE";
    case SYS_ACTIVE:         return "ACTIVE";
    case SYS_MOTOR_FAULT:    return "MOTOR_FAULT";
    case SYS_LEAK_FAULT:     return "LEAK_FAULT";
    case SYS_SCALE_FAULT:    return "SCALE_FAULT";
    case SYS_TIMEOUT_FAULT:  return "TIMEOUT_FAULT";
    case SYS_ABORTED_FAULT:  return "ABORTED_FAULT";
    case SYS_OFF:            return "OFF";
    default:                 return "?";
  }
}

/* ========================= High-level wrappers ======================== */
bool transact_result_block(uint8_t slaveId,
                           const uint16_t* wr, uint16_t wqty,
                           uint16_t outRegs[RES_COUNT])
{
  uint8_t data[RES_COUNT*2];
  uint16_t outLen=0;
/* ---------------------------------------------
   Motor token <-> numeric ID for Dispenser
   --------------------------------------------- */
bool motorTokenToId(const String& token, uint16_t& outId) {
  String t = token; t.trim();
  String tl = t; tl.toLowerCase();

  bool ok = mb17_transact(slaveId, RES_BASE, RES_COUNT, REG_OPCODE, wr, wqty, data, outLen);
  if (!ok) return false;
  // numeric?
  bool allDigits = true;
  for (uint16_t i=0;i<tl.length();++i) if (!isDigit(tl[i])) { allDigits=false; break; }
  if (allDigits && tl.length()>0) { outId = (uint16_t)tl.toInt(); return true; }

  bytesToRegsBE(data, outLen, outRegs, RES_COUNT);
  return true;
}
  auto parseIdx = [&](const String& s, uint8_t &idx)->bool {
    long v = s.toInt(); if (v < 1 || v > 255) return false; idx = (uint8_t)v; return true;
  };

void printHelp(){
  Serial.println(F("=== MASTER QUICK GUIDE ==="));
  Serial.println(F("Wiring: Serial1 <-> MAX485, DE&/RE -> D2 (HIGH=TX, LOW=RX). 19200 8N1."));
  Serial.println(F("Slaves: Dispenser(ID=1), Cleaner(ID=2). Modbus Function 0x17 only."));
  Serial.println(F("Polling: ACTIVE every 150ms, IDLE every 800ms. Idle logs suppressed."));
  Serial.println(F("Commands:"));
  Serial.println(F("  d.status | d.clear | d.abort"));
  Serial.println(F("  d.rinse <sec>"));
  Serial.println(F("  d.trigger <motorId> <sec>"));
  Serial.println(F("  d.dispense <motorId> <target_g> <slow_g> <soft_g> <timeout_s>"));
  Serial.println(F("  c.status | c.clear | c.abort | c.off"));
  Serial.println(F("  c.init <sec>"));
  Serial.println(F("  c.froth <targetC> <timeout_s>"));
  Serial.println(F("  c.clean <tValve> <tSteam> <tStandby>"));
  Serial.println(F("Examples: d.dispense 101 120 20 5 45 | c.froth 65 40 | c.clean 1 3 2"));
  uint8_t idx=0;
  if (tl.startsWith("sauce")) { if (!parseIdx(tl.substring(5), idx)) return false; if (idx<1||idx>15) return false; outId = 100 + idx; return true; }
  if (tl.startsWith("milk"))  { if (!parseIdx(tl.substring(4), idx)) return false; if (idx<1||idx>8 ) return false; outId = idx;       return true; }
  if (tl.startsWith("s"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1||idx>15) return false; outId = 100 + idx; return true; }
  if (tl.startsWith("m"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1||idx>8 ) return false; outId = idx;       return true; }

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

/* -------- Conditional logging: no spam when idle -------- */
void logResultMaybe(Device& D, const uint16_t r[RES_COUNT], bool force){
  bool statusChanged = (r[2] != D.lastSYS);
  bool nonIdle       = (r[2] != SYS_IDLE);
  bool rcFail        = (r[0] != 0) || (r[1] != 0);
  bool recovering    = D.commsFault; // last cycle had comms fault
/* =========================================================================================
   DISPENSER (ID=1)
   ========================================================================================= */

  if (!(force || statusChanged || nonIdle || rcFail || recovering)) {
    return; // suppress idle spam
  }
struct PendingOp {
  uint8_t kind = 0;
  uint32_t deadline = 0;
  uint32_t nextPollDue = 0;
  bool printedTimeout = false;
  bool active = false;
};

  Serial.print('['); Serial.print(D.name);
  Serial.print(F("] RC=")); Serial.print(r[0]);
  Serial.print(F(" ERR=")); Serial.print(r[1]);
  Serial.print(F(" SYS=")); Serial.print(sysStr(r[2]));
  Serial.print(F(" SEQ=")); Serial.print(r[3]);
  Serial.print(F(" AUX0=")); Serial.print(r[4]);
  Serial.print(F(" ELAPSEDx10=")); Serial.println(r[5]);
enum : uint8_t { D_PK_NONE=0, D_PK_RINSE, D_PK_TRIGGER, D_PK_DISPENSE };

struct DeviceDisp {
  const char* name = "DISPENSER";
  const uint8_t id = DISP_SLAVE_ID;
  PendingOp pend;
  bool abortRequested = false;
} DISP;

/* [sent] echoes */
static void d_logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void d_logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void d_logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void d_logSent_RINSE(float seconds) {
  Serial.print(F("-------------\n[sent] RINSE seconds=")); Serial.print(seconds,1); Serial.println(F("."));
}
static void d_logSent_TRIGGER(uint16_t motorId, float seconds) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] TRIGGER motor=")); Serial.print(name);
  Serial.print(F(" seconds=")); Serial.print(seconds,1); Serial.println(F("."));
}
static void d_logSent_DISPENSE(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] DISPENSE motor=")); Serial.print(name);
  Serial.print(F(" target_g=")); Serial.print(target_g,1);
  Serial.print(F(" slowOffset_g=")); Serial.print(slow_g,1);
  Serial.print(F(" softCutOffset_g=")); Serial.print(soft_g,1);
  Serial.print(F(" timeout_s=")); Serial.print(timeout_s); Serial.println(F("."));
}

void updateDeviceFromResult(Device& D, const uint16_t r[RES_COUNT]){
  D.lastRC = r[0]; D.lastERR = r[1]; D.lastSYS = r[2]; D.lastSEQ = r[3];
  D.lastAUX = r[4]; D.lastELAPSEDx10 = r[5];
  D.commsFault = false;
/* Brief result line (same style across) */
void d_printResultBrief(const uint16_t* R) {
  Serial.print(F("result=")); Serial.print(R[0]==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errStr(R[1]));
  Serial.print(F("  status=")); Serial.print(sysStr(R[2]));
  Serial.println();
}

  unsigned long now = millis();
  if (D.lastSYS == SYS_ACTIVE){
    D.nextStatusDueMs = now + POLL_ACTIVE_MS;
  } else {
    D.nextStatusDueMs = now + POLL_IDLE_MS;
  }
/* API wrappers (seq=0) */
bool d_api_status(uint16_t* outRegs, uint32_t timeoutMs=600) {
  uint16_t W[2] = { D_OP_STATUS, 0 };
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool d_api_status_logged(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  d_logSent_STATUS();
  return d_api_status(outRegs, timeoutMs);
}
bool d_api_clear(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[2] = { D_OP_CLEAR, 0 };
  d_logSent_CLEAR();
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool d_api_abort(uint16_t* outRegs, uint32_t timeoutMs=800) {
  uint16_t W[2] = { D_OP_ABORT, 0 };
  d_logSent_ABORT();
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool d_api_rinse(float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[3] = { D_OP_RINSE, 0, seconds_x10 };
  d_logSent_RINSE(seconds);
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 3, outRegs, RES_QTY, 2000);
}
bool d_api_trigger(uint16_t motorId, float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[4] = { D_OP_TRIGGER, 0, motorId, seconds_x10 };
  d_logSent_TRIGGER(motorId, seconds);
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 4, outRegs, RES_QTY, 2000);
}
bool d_api_dispense(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v<0) v=0; if (v>65535) v=65535; return (uint16_t)v; };
  uint16_t W[7] = { D_OP_DISPENSE, 0, motorId, to_x10(target_g), to_x10(slow_g), to_x10(soft_g), timeout_s };
  d_logSent_DISPENSE(motorId, target_g, slow_g, soft_g, timeout_s);
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 7, outRegs, RES_QTY, 2000);
}

/* ------------------ Dispenser ops (ID=1) ------------------ */
bool d_status(){
  uint16_t wr[2] = { OP_STATUS, DISP.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 2, r)){
      logResultMaybe(DISP, r, /*force=*/false);
      updateDeviceFromResult(DISP, r);
      return true;
    }
/* Salvage acceptance (if start reply failed) */
static bool d_salvage_accept(uint16_t* outRegs, uint8_t kindLabel){
  uint16_t S[RES_QTY]={0};
  bool ok = d_api_status(S,600);
  if (ok && S[2]==SYS_ACTIVE){
    memcpy(outRegs, S, RES_QTY*sizeof(uint16_t));
    const char* tag = (kindLabel==D_PK_RINSE)?"rinse":(kindLabel==D_PK_TRIGGER)?"trigger":"dispense";
    Serial.print(F("[salvage] reply CRC/timeout, but live STATUS=ACTIVE; proceeding ("));
    Serial.print(tag); Serial.println(F(")."));
    return true;
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (status)")); }
  return false;
}
bool d_clear(){
  uint16_t wr[2] = { OP_CLEAR, ++DISP.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 2, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;
    }
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (clear)")); }
  return false;

/* Pending scheduler */
const char* d_pendingLabel(uint8_t k){ return (k==D_PK_RINSE)?"rinse":(k==D_PK_TRIGGER)?"trigger":(k==D_PK_DISPENSE)?"dispense":"?"; }
void d_armPending(uint8_t k, uint32_t extraMs){ DISP.pend.kind=k; DISP.pend.printedTimeout=false; DISP.pend.nextPollDue=millis()+STATUS_POLL_MS; DISP.pend.deadline=millis()+extraMs; DISP.pend.active=true; }
void d_clearPending(){ DISP.pend.kind=D_PK_NONE; DISP.pend.deadline=0; DISP.pend.nextPollDue=0; DISP.pend.printedTimeout=false; DISP.pend.active=false; }

bool d_pollStatusOnce(uint16_t* R) {
  uint16_t W[2] = { D_OP_STATUS, 0 };
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, R, RES_QTY, 600, /*verbose*/false);
}
bool d_abort(){
  uint16_t wr[2] = { OP_ABORT, ++DISP.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 2, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;

/* Explicit status (prints once) */
bool d_status(bool forcePrint=false){
  uint16_t r[RES_QTY];
  if (d_api_status(r, 1000)){
    if (forcePrint){
      Serial.print(F("[DISPENSER] STATUS sys=")); Serial.print(sysStr(r[2]));
      if (r[1]) { Serial.print(F(" | err=")); Serial.print(errStr(r[1])); }
      if (r[4]) { Serial.print(F(" | weight_g=")); Serial.print(r[4]/10.0f,1); }
      Serial.println();
    } else {
      // background: finalize pending if transitioned
      if (DISP.pend.active && r[2] != SYS_ACTIVE){
        if (r[2]==SYS_IDLE) {
          Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.println(F("] done."));
        } else {
          Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.print(F("] ended: "));
          Serial.println(sysStr(r[2]));
        }
        d_clearPending();
      }
    }
    return true;
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (abort)")); }
  if (forcePrint) Serial.println(F("[DISPENSER] STATUS -> NO REPLY"));
  return false;
}
bool d_rinse(float seconds){
  uint16_t sx10 = (uint16_t)lroundf(seconds * 10.0f);
  uint16_t wr[3] = { OP_RINSE, ++DISP.seq, sx10 };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 3, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;

void d_servicePending(){
  if (!DISP.pend.active) return;
  uint32_t now = millis();

  if (DISP.abortRequested){
    uint16_t R[RES_QTY]={0};
    bool ok = d_api_abort(R,800);
    DISP.abortRequested = false;
    if (ok) {
      d_printResultBrief(R);
      DISP.pend.nextPollDue = now + 100;
    } else {
      Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
    }
    return;
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (rinse)")); }
  return false;
}
bool d_trigger(uint16_t motorId, float seconds){
  uint16_t sx10 = (uint16_t)lroundf(seconds * 10.0f);
  uint16_t wr[4] = { OP_TRIGGER, ++DISP.seq, motorId, sx10 };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 4, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;

  if ((long)(now - DISP.pend.nextPollDue) < 0) return;

  uint16_t R[RES_QTY]={0};
  bool ok = d_pollStatusOnce(R);
  if (!ok){ DISP.pend.nextPollDue = millis()+STATUS_POLL_MS; return; }

  if (R[2] != SYS_ACTIVE){
    if (R[2]==SYS_IDLE) {
      Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.println(F("] done."));
    } else {
      Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.print(F("] ended: "));
      Serial.println(sysStr(R[2]));
    }
    d_clearPending();
    return;
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (trigger)")); }
  return false;
}
bool d_dispense(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s){
  uint16_t wr[7] = {
    OP_DISPENSE, ++DISP.seq,
    motorId,
    (uint16_t)lroundf(target_g * 10.0f),
    (uint16_t)lroundf(slow_g   * 10.0f),
    (uint16_t)lroundf(soft_g   * 10.0f),
    timeout_s
  };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 7, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;
    }

  if (!DISP.pend.printedTimeout && (long)(now - DISP.pend.deadline) > 0){
    Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    DISP.pend.printedTimeout = true;
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (dispense)")); }
  return false;
  DISP.pend.nextPollDue = millis() + STATUS_POLL_MS;
}

/* ------------------ Cleaner ops (ID=2) ------------------ */
bool c_status(){
  uint16_t wr[2] = { OP_STATUS, CLEAN.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 2, r)){
      logResultMaybe(CLEAN, r, /*force=*/false);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (status)")); }
  return false;
/* =========================================================================================
   CLEANER (ID=2)
   ========================================================================================= */

struct DeviceClean {
  const char* name = "CLEANER";
  const uint8_t id = CLEAN_SLAVE_ID;
  PendingOp pend;
  bool abortRequested = false;
} CLEAN;

/* [sent] echoes */
static void c_logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void c_logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void c_logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void c_logSent_OFF()    { Serial.println(F("-------------\n[sent] OFF.")); }
static void c_logSent_INIT(float seconds) {
  Serial.print(F("-------------\n[sent] INIT seconds=")); Serial.print(seconds,1); Serial.println(F("."));
}
bool c_clear(){
  uint16_t wr[2] = { OP_CLEAR, ++CLEAN.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 2, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (clear)")); }
  return false;
static void c_logSent_FROTH(float targetC, float timeout_s) {
  Serial.print(F("-------------\n[sent] FROTH targetC=")); Serial.print(targetC,1);
  Serial.print(F("C timeout_s=")); Serial.print(timeout_s,1); Serial.println(F("."));
}
bool c_abort(){
  uint16_t wr[2] = { OP_ABORT, ++CLEAN.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 2, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (abort)")); }
  return false;
static void c_logSent_CLEAN(float tValve, float tSteam, float tStandby) {
  Serial.print(F("-------------\n[sent] CLEAN tValve_s=")); Serial.print(tValve,1);
  Serial.print(F(" tSteam_s=")); Serial.print(tSteam,1);
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby,1); Serial.println(F("."));
}
bool c_off(){
  uint16_t wr[2] = { OP_OFF, ++CLEAN.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 2, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (off)")); }
  return false;

/* Brief result line (adds tempC/elapsed if present) */
void c_printResultBrief(const uint16_t* R) {
  Serial.print(F("result=")); Serial.print(R[0]==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errStr(R[1]));
  Serial.print(F("  status=")); Serial.print(sysStr(R[2]));
  if (R[4]) { Serial.print(F("  tempC=")); Serial.print(R[4]/10.0f,1); }
  Serial.println();
}
bool c_init(float seconds){
  uint16_t sx10 = (uint16_t)lroundf(seconds * 10.0f);
  uint16_t wr[3] = { OP_INIT, ++CLEAN.seq, sx10 };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 3, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (init)")); }
  return false;

/* API wrappers (seq=0) */
bool c_api_status(uint16_t* outRegs, uint32_t timeoutMs=600) {
  uint16_t W[2] = { C_OP_STATUS, 0 };
  return mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool c_froth(float targetC, float timeout_s){
  uint16_t wr[4] = {
    OP_FROTH, ++CLEAN.seq,
    (uint16_t)lroundf(targetC * 10.0f),
    (uint16_t)lroundf(timeout_s * 10.0f)
  };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 4, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (froth)")); }
  return false;
bool c_api_status_logged(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  c_logSent_STATUS();
  return c_api_status(outRegs, timeoutMs);
}
bool c_clean(float tValve, float tSteam, float tStandby){
  uint16_t wr[5] = {
    OP_CLEAN, ++CLEAN.seq,
    (uint16_t)lroundf(tValve   * 10.0f),
    (uint16_t)lroundf(tSteam   * 10.0f),
    (uint16_t)lroundf(tStandby * 10.0f)
  };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(CLEAN.id, wr, 5, r)){
      logResultMaybe(CLEAN, r, /*force=*/true);
      updateDeviceFromResult(CLEAN, r);
      return true;
bool c_api_clear(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[2] = { C_OP_CLEAR, 0 };
  c_logSent_CLEAR();
  return mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool c_api_abort(uint16_t* outRegs, uint32_t timeoutMs=800) {
  uint16_t W[2] = { C_OP_ABORT, 0 };
  c_logSent_ABORT();
  return mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool c_api_off(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[2] = { C_OP_OFF, 0 };
  c_logSent_OFF();
  return mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool c_api_init(float seconds, uint16_t* outRegs) {
  uint16_t sx10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[3] = { C_OP_INIT, 0, sx10 };
  c_logSent_INIT(seconds);
  bool ok = mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 3, outRegs, RES_QTY, 2500);
  if (!ok) {
    uint16_t S[RES_QTY]={0}; if (c_api_status(S,600) && S[2]==SYS_ACTIVE) { memcpy(outRegs,S,sizeof(S)); Serial.println(F("[salvage] reply CRC/timeout, STATUS=ACTIVE; proceeding (init).")); return true; }
  }
  return ok;
}
bool c_api_froth(float targetC, float timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v=(long)(f*10.0f+0.5f); if (v<0)v=0; if (v>65535)v=65535; return (uint16_t)v; };
  uint16_t W[4] = { C_OP_FROTH, 0, to_x10(targetC), to_x10(timeout_s) };
  c_logSent_FROTH(targetC, timeout_s);
  bool ok = mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 4, outRegs, RES_QTY, 2500);
  if (!ok) {
    uint16_t S[RES_QTY]={0}; if (c_api_status(S,600) && S[2]==SYS_ACTIVE) { memcpy(outRegs,S,sizeof(S)); Serial.println(F("[salvage] reply CRC/timeout, STATUS=ACTIVE; proceeding (froth).")); return true; }
  }
  return ok;
}
bool c_api_clean(float tValve, float tSteam, float tStandby, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v=(long)(f*10.0f+0.5f); if (v<0)v=0; if (v>65535)v=65535; return (uint16_t)v; };
  uint16_t W[5] = { C_OP_CLEAN, 0, to_x10(tValve), to_x10(tSteam), to_x10(tStandby) };
  c_logSent_CLEAN(tValve, tSteam, tStandby);
  bool ok = mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 5, outRegs, RES_QTY, 2500);
  if (!ok) {
    uint16_t S[RES_QTY]={0}; if (c_api_status(S,600) && S[2]==SYS_ACTIVE) { memcpy(outRegs,S,sizeof(S)); Serial.println(F("[salvage] reply CRC/timeout, STATUS=ACTIVE; proceeding (clean).")); return true; }
  }
  return ok;
}

/* Explicit status (prints once) */
bool c_status(bool forcePrint=false){
  uint16_t r[RES_QTY];
  if (c_api_status(r, 1000)){
    if (forcePrint){
      Serial.print(F("[CLEANER] STATUS sys=")); Serial.print(sysStr(r[2]));
      if (r[1]) { Serial.print(F(" | err=")); Serial.print(errStr(r[1])); }
      if (r[4]) { Serial.print(F(" | tempC=")); Serial.print(r[4]/10.0f,1); }
      if (r[5]) { Serial.print(F(" | elapsed_s=")); Serial.print(r[5]/10.0f,1); }
      Serial.println();
    } else {
      if (CLEAN.pend.active && r[2] != SYS_ACTIVE){
        if (r[2]==SYS_IDLE || r[2]==SYS_OFF) {
          Serial.print('['); Serial.print((CLEAN.pend.kind==1)?"init":(CLEAN.pend.kind==2)?"froth":(CLEAN.pend.kind==3)?"clean":"?");
          Serial.println(F("] done."));
        } else {
          Serial.print('['); Serial.print((CLEAN.pend.kind==1)?"init":(CLEAN.pend.kind==2)?"froth":(CLEAN.pend.kind==3)?"clean":"?");
          Serial.print(F("] ended: ")); Serial.println(sysStr(r[2]));
        }
        CLEAN.pend.active=false; CLEAN.pend.kind=0; CLEAN.pend.deadline=0; CLEAN.pend.nextPollDue=0; CLEAN.pend.printedTimeout=false;
      }
    }
    return true;
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (clean)")); }
  if (forcePrint) Serial.println(F("[CLEANER] STATUS -> NO REPLY"));
  return false;
}

/* ========================= CLI (USB Serial) =========================== */
String readLine(){
  static String buf;
  while (Serial.available()){
    char ch = (char)Serial.read();
    if (ch=='\r') continue;
    if (ch=='\n'){ String out=buf; buf=""; out.trim(); return out; }
    if (buf.length() < 120) buf += ch;
  }
  return String();
}
/* Pending scheduler */
const char* c_pendingLabel(uint8_t k){ return (k==1)?"init":(k==2)?"froth":(k==3)?"clean":"?"; }
void c_armPending(uint8_t k, uint32_t extraMs){ CLEAN.pend.kind=k; CLEAN.pend.printedTimeout=false; CLEAN.pend.nextPollDue=millis()+STATUS_POLL_MS; CLEAN.pend.deadline=millis()+extraMs; CLEAN.pend.active=true; }

void handleCLI(){
  String cmd = readLine();
  if (!cmd.length()) return;
  String s = cmd; s.trim();
void c_servicePending(){
  if (!CLEAN.pend.active) return;
  uint32_t now = millis();

  // normalize lower for routing; keep originals for numbers
  String lower = s; lower.toLowerCase();
  if (CLEAN.abortRequested){
    uint16_t R[RES_QTY]={0};
    bool ok = c_api_abort(R,800);
    CLEAN.abortRequested=false;
    if (ok){ c_printResultBrief(R); CLEAN.pend.nextPollDue = now + 100; }
    else   { Serial.println(F("[abort] NO RESPONSE / CRC/timeout")); }
    return;
  }

  if (lower == "help"){ printHelp(); return; }
  if ((long)(now - CLEAN.pend.nextPollDue) < 0) return;

  // Dispenser
  if (lower == "d.status"){ d_status(); return; }
  if (lower == "d.clear"){ d_clear(); return; }
  if (lower == "d.abort"){ d_abort(); return; }
  if (lower.startsWith("d.rinse ")){
    float sec = lower.substring(8).toFloat(); d_rinse(sec); return;
  }
  if (lower.startsWith("d.trigger ")){
    String rest = lower.substring(10); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp<0){ Serial.println(F("Use: d.trigger <motorId> <sec>")); return; }
    uint16_t motorId = (uint16_t)rest.substring(0, sp).toInt();
    float sec = rest.substring(sp+1).toFloat();
    d_trigger(motorId, sec); return;
  }
  if (lower.startsWith("d.dispense ")){
    String rest = lower.substring(11); rest.trim();
    int s1 = rest.indexOf(' '); if (s1<0){ Serial.println(F("Use: d.dispense <motorId> <target_g> <slow_g> <soft_g> <timeout_s>")); return; }
    int s2 = rest.indexOf(' ', s1+1); if (s2<0){ Serial.println(F("Use: d.dispense ...")); return; }
    int s3 = rest.indexOf(' ', s2+1); if (s3<0){ Serial.println(F("Use: d.dispense ...")); return; }
    int s4 = rest.indexOf(' ', s3+1); if (s4<0){ Serial.println(F("Use: d.dispense ...")); return; }
    uint16_t motorId   = (uint16_t)rest.substring(0, s1).toInt();
    float target_g     = rest.substring(s1+1, s2).toFloat();
    float slow_g       = rest.substring(s2+1, s3).toFloat();
    float soft_g       = rest.substring(s3+1, s4).toFloat();
    uint16_t timeout_s = (uint16_t)rest.substring(s4+1).toInt();
    d_dispense(motorId, target_g, slow_g, soft_g, timeout_s); return;
  }
  uint16_t R[RES_QTY]={0};
  bool ok = c_api_status(R,600);
  if (!ok){ CLEAN.pend.nextPollDue = millis()+STATUS_POLL_MS; return; }

  // Cleaner
  if (lower == "c.status"){ c_status(); return; }
  if (lower == "c.clear"){ c_clear(); return; }
  if (lower == "c.abort"){ c_abort(); return; }
  if (lower == "c.off"){ c_off(); return; }
  if (lower.startsWith("c.init ")){
    float sec = lower.substring(7).toFloat(); c_init(sec); return;
  }
  if (lower.startsWith("c.froth ")){
    int sp = lower.indexOf(' ', 8); if (sp<0){ Serial.println(F("Use: c.froth <targetC> <timeout_s>")); return; }
    float targetC  = lower.substring(8, sp).toFloat();
    float tout     = lower.substring(sp+1).toFloat();
    c_froth(targetC, tout); return;
  }
  if (lower.startsWith("c.clean ")){
    String rest = lower.substring(8); rest.trim();
    int s1 = rest.indexOf(' '); if (s1<0){ Serial.println(F("Use: c.clean <tValve> <tSteam> <tStandby>")); return; }
    int s2 = rest.indexOf(' ', s1+1); if (s2<0){ Serial.println(F("Use: c.clean <tValve> <tSteam> <tStandby>")); return; }
    float tValve   = rest.substring(0, s1).toFloat();
    float tSteam   = rest.substring(s1+1, s2).toFloat();
    float tStandby = rest.substring(s2+1).toFloat();
    c_clean(tValve, tSteam, tStandby); return;
  if (R[2] != SYS_ACTIVE){
    if (R[2]==SYS_IDLE || R[2]==SYS_OFF) {
      Serial.print('['); Serial.print(c_pendingLabel(CLEAN.pend.kind)); Serial.println(F("] done."));
    } else {
      Serial.print('['); Serial.print(c_pendingLabel(CLEAN.pend.kind)); Serial.print(F("] ended: "));
      Serial.println(sysStr(R[2]));
    }
    CLEAN.pend.active=false; CLEAN.pend.kind=0; CLEAN.pend.deadline=0; CLEAN.pend.nextPollDue=0; CLEAN.pend.printedTimeout=false;
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
  if (!CLEAN.pend.printedTimeout && (long)(now - CLEAN.pend.deadline) > 0){
    Serial.print('['); Serial.print(c_pendingLabel(CLEAN.pend.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    CLEAN.pend.printedTimeout = true;
  }
  CLEAN.pend.nextPollDue = millis() + STATUS_POLL_MS;
}

/* ========================= Periodic poller ============================ */
void pollScheduler(){
  unsigned long now = millis();

  // Dispenser polling
  if (now >= DISP.nextStatusDueMs){
    d_status();
  }
  // Cleaner polling
  if (now >= CLEAN.nextStatusDueMs){
    c_status();
  }
/* =========================================================================================
   CLI + SCHEDULER
   ========================================================================================= */

String inLine;

void printHelp() {
  Serial.println(F("Commands (prefix d.=dispenser, c.=cleaner):"));
  Serial.println(F("  d.status"));
  Serial.println(F("  d.clear"));
  Serial.println(F("  d.abort"));
  Serial.println(F("  d.rinse <seconds>"));
  Serial.println(F("  d.trigger <motorId|name> <seconds>   e.g. 'd.trigger 102 1.0' or 'd.trigger Sauce2 1.0'"));
  Serial.println(F("  d.dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
  Serial.println(F("  c.status"));
  Serial.println(F("  c.clear"));
  Serial.println(F("  c.abort"));
  Serial.println(F("  c.off"));
  Serial.println(F("  c.init  <seconds>"));
  Serial.println(F("  c.froth <targetC> <timeout_s>"));
  Serial.println(F("  c.clean <tValve_s> <tSteam_s> <tStandby_s>"));
}

/* ========================= Setup / Loop =============================== */
void setup(){
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  rs485RxMode();                // default to receive
void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);
  while (!Serial) {}

  RS485.begin(BUS_BAUD);     // 8N1 default
  delay(50);

  Serial.println(F("\nNano Every Modbus Master ready (Dispenser+Cleaner)."));
  printHelp();

  // Prime initial poll times
  DISP.nextStatusDueMs = CLEAN.nextStatusDueMs = millis() + 200;
  // Quick probes
  uint16_t R[RES_QTY] = {0};
  Serial.print(F("\n[boot] Probing Dispenser STATUS...\n"));
  if (d_api_status_logged(R, 1500)) { Serial.println(F("Modbus OK")); d_printResultBrief(R); }
  else                               Serial.println(F("NO RESPONSE / CRC"));

  Serial.println(F("=== Modbus RTU Master (0x17-only) for Dispenser(ID=1) + Cleaner(ID=2) ==="));
  printHelp();  // show the guide on boot
  Serial.print(F("\n[boot] Probing Cleaner STATUS...\n"));
  if (c_api_status_logged(R, 1500)) { Serial.println(F("Modbus OK")); c_printResultBrief(R); }
  else                               Serial.println(F("NO RESPONSE / CRC"));
}

void loop(){
  handleCLI();        // manual control
  pollScheduler();    // automatic status polling (active faster, idle slower)
void loop() {
  // Background pending schedulers (silent unless completion/timeout)
  d_servicePending();
  c_servicePending();

  // Non-blocking CLI
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      inLine.trim();
      if (!inLine.length()) continue;
      String cmd = inLine; inLine = "";
      String low = cmd; low.toLowerCase();

      // DISPENSER commands
      if (low == "d.status") {
        uint16_t R[RES_QTY]={0};
        d_logSent_STATUS();
        if (d_api_status(R)) d_printResultBrief(R);
        else Serial.println(F("[status] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "d.clear") {
        uint16_t R[RES_QTY]={0};
        if (d_api_clear(R)) d_printResultBrief(R);
        else Serial.println(F("[clear] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "d.abort") {
        uint16_t R[RES_QTY]={0};
        if (DISP.pend.active) DISP.abortRequested=true;
        if (d_api_abort(R,800)) d_printResultBrief(R);
        else Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
      }
      else if (low.startsWith("d.rinse ")) {
        if (DISP.pend.active) { Serial.println(F("[busy] operation in progress; type 'd.abort' or 'd.status'.")); }
        else {
          String s = cmd.substring(8); s.trim();
          float secs = s.toFloat();
          uint16_t R[RES_QTY]={0};
          bool ok = d_api_rinse(secs, R);
          if (!ok) {
            // salvage: if active, proceed
            if (d_salvage_accept(R, D_PK_RINSE)) ok = true;
          }
          if (!ok) Serial.println(F("[rinse] NO RESPONSE / CRC/timeout"));
          else {
            d_printResultBrief(R);
            if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
              uint32_t guard = 5000UL;
              d_armPending(D_PK_RINSE, (uint32_t)(secs*1000.0f)+guard);
            }
          }
        }
      }
      else if (low.startsWith("d.trigger ")) {
        if (DISP.pend.active) { Serial.println(F("[busy] operation in progress; type 'd.abort' or 'd.status'.")); }
        else {
          String rest = cmd.substring(10); rest.trim();
          int sp = rest.indexOf(' ');
          if (sp < 0) { Serial.println(F("Use: d.trigger <motorId|name> <seconds>")); }
          else {
            String tok = rest.substring(0,sp); tok.trim();
            String ssec = rest.substring(sp+1); ssec.trim();
            uint16_t motorId;
            if (!motorTokenToId(tok, motorId)) { Serial.println(F("[err] unknown motor token")); goto _after_cli; }
            float secs = ssec.toFloat();
            uint16_t R[RES_QTY]={0};
            bool ok = d_api_trigger(motorId, secs, R);
            if (!ok) {
              if (d_salvage_accept(R, D_PK_TRIGGER)) ok = true;
            }
            if (!ok) Serial.println(F("[trigger] NO RESPONSE / CRC/timeout"));
            else {
              d_printResultBrief(R);
              if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
                uint32_t guard = 5000UL;
                d_armPending(D_PK_TRIGGER, (uint32_t)(secs*1000.0f)+guard);
              }
            }
          }
        }
      }
      else if (low.startsWith("d.dispense ")) {
        if (DISP.pend.active) { Serial.println(F("[busy] operation in progress; type 'd.abort' or 'd.status'.")); }
        else {
          String rest = cmd.substring(11); rest.trim();
          int sp1 = rest.indexOf(' ');
          int sp2 = rest.indexOf(' ', sp1+1);
          int sp3 = rest.indexOf(' ', sp2+1);
          int sp4 = rest.indexOf(' ', sp3+1);
          if (sp1<0 || sp2<0 || sp3<0 || sp4<0) {
            Serial.println(F("Use: d.dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
          } else {
            String tok      = rest.substring(0, sp1); tok.trim();
            float target_g  = rest.substring(sp1+1, sp2).toFloat();
            float slow_g    = rest.substring(sp2+1, sp3).toFloat();
            float soft_g    = rest.substring(sp3+1, sp4).toFloat();
            uint16_t tout_s = (uint16_t)rest.substring(sp4+1).toInt();

            uint16_t motorId;
            if (!motorTokenToId(tok, motorId)) { Serial.println(F("[err] unknown motor token")); goto _after_cli; }

            uint16_t R[RES_QTY]={0};
            bool ok = d_api_dispense(motorId, target_g, slow_g, soft_g, tout_s, R);
            if (!ok) {
              if (d_salvage_accept(R, D_PK_DISPENSE)) ok = true;
            }
            if (!ok) Serial.println(F("[dispense] NO RESPONSE / CRC/timeout"));
            else {
              d_printResultBrief(R);
              if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
                uint32_t guard = 10000UL;
                d_armPending(D_PK_DISPENSE, (uint32_t)tout_s*1000UL + guard);
              }
            }
          }
        }
      }

      // CLEANER commands
      else if (low == "c.status") {
        uint16_t R[RES_QTY]={0};
        c_logSent_STATUS();
        if (c_api_status(R)) c_printResultBrief(R);
        else Serial.println(F("[status] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "c.clear") {
        uint16_t R[RES_QTY]={0};
        if (c_api_clear(R)) c_printResultBrief(R);
        else Serial.println(F("[clear] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "c.abort") {
        uint16_t R[RES_QTY]={0};
        if (CLEAN.pend.active) CLEAN.abortRequested=true;
        if (c_api_abort(R,800)) c_printResultBrief(R);
        else Serial.println(F("[abort] NO RESPONSE / CRC/timeout"));
      }
      else if (low == "c.off") {
        uint16_t R[RES_QTY]={0};
        if (c_api_off(R)) c_printResultBrief(R);
        else Serial.println(F("[off] NO RESPONSE / CRC/timeout"));
      }
      else if (low.startsWith("c.init ")) {
        if (CLEAN.pend.active) { Serial.println(F("[busy] operation in progress; type 'c.abort' or 'c.status'.")); }
        else {
          String s = cmd.substring(7); s.trim();
          float secs = s.toFloat();
          uint16_t R[RES_QTY]={0};
          bool ok = c_api_init(secs, R);
          if (!ok) Serial.println(F("[init] NO RESPONSE / CRC/timeout"));
          else {
            c_printResultBrief(R);
            if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
              uint32_t guard = 5000UL;
              c_armPending(/*init*/1, (uint32_t)(secs*1000.0f)+guard);
            }
          }
        }
      }
      else if (low.startsWith("c.froth ")) {
        if (CLEAN.pend.active) { Serial.println(F("[busy] operation in progress; type 'c.abort' or 'c.status'.")); }
        else {
          int sp = low.indexOf(' ', 8);
          if (sp < 0) { Serial.println(F("Use: c.froth <targetC> <timeout_s>")); }
          else {
            float targetC = cmd.substring(8, sp).toFloat();
            float tout    = cmd.substring(sp+1).toFloat();
            uint16_t R[RES_QTY]={0};
            bool ok = c_api_froth(targetC, tout, R);
            if (!ok) Serial.println(F("[froth] NO RESPONSE / CRC/timeout"));
            else {
              c_printResultBrief(R);
              if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
                uint32_t guard = 10000UL;
                c_armPending(/*froth*/2, (uint32_t)(tout*1000.0f)+guard);
              }
            }
          }
        }
      }
      else if (low.startsWith("c.clean ")) {
        if (CLEAN.pend.active) { Serial.println(F("[busy] operation in progress; type 'c.abort' or 'c.status'.")); }
        else {
          String rest = cmd.substring(8); rest.trim();
          int s1 = rest.indexOf(' '); if (s1<0) { Serial.println(F("Use: c.clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _after_cli; }
          int s2 = rest.indexOf(' ', s1+1); if (s2<0) { Serial.println(F("Use: c.clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _after_cli; }
          float tValve = rest.substring(0, s1).toFloat();
          float tSteam = rest.substring(s1+1, s2).toFloat();
          float tStby  = rest.substring(s2+1).toFloat();
          uint16_t R[RES_QTY]={0};
          bool ok = c_api_clean(tValve, tSteam, tStby, R);
          if (!ok) Serial.println(F("[clean] NO RESPONSE / CRC/timeout"));
          else {
            c_printResultBrief(R);
            if (R[0]==RC_OK && R[2]==SYS_ACTIVE) {
              uint32_t guard = 10000UL;
              uint32_t total = (uint32_t)((tValve + tSteam + tStby)*1000.0f) + guard;
              c_armPending(/*clean*/3, total);
            }
          }
        }
      }
      else if (low == "help") {
        printHelp();
      }
      else {
        Serial.println(F("Unknown command. Type 'help'."));
      }

_after_cli:
      // Immediately tick schedulers once for snappy feedback
      d_servicePending();
      c_servicePending();

    } else {
      if (inLine.length() < 160) inLine += c;
    }
  }

  // Idle ticks
  d_servicePending();
  c_servicePending();
}