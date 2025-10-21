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

// Character/gap timing at 19,200 bps (11-bit frames)
const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD;  // ≈573 µs
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);      // ≈2.0 ms

// Turnaround guard for MAX485 drivers before first TX byte
const uint16_t RS485_TX_GUARD_US = 150;

// Master side timeouts/retries
const uint16_t MB_RESP_TIMEOUT_MS = 80;   // per request
const uint8_t  MB_MAX_RETRIES     = 2;    // in addition to first try

// Frame buffers
#define MB_MAX_FRAME  260

/* ========================= Slave IDs & register map =================== */
#define SLAVE_ID_DISPENSER  1
#define SLAVE_ID_CLEANER    2

// Common command block (write)
#define REG_OPCODE   0x0000
#define REG_SEQ      0x0001

// Opcodes (shared semantics)
enum Opcode : uint16_t {
  OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_OFF=4,
  OP_DISPENSE=10, OP_RINSE=11, OP_TRIGGER=12,      // Dispenser
  OP_INIT=10, OP_FROTH=11, OP_CLEAN=12             // Cleaner (overlaps IDs intentionally)
};

enum SystemStatus : uint16_t {
  SYS_IDLE=0, SYS_ACTIVE=1, SYS_MOTOR_FAULT=2, SYS_LEAK_FAULT=3, SYS_SCALE_FAULT=4,
  SYS_TIMEOUT_FAULT=5, SYS_ABORTED_FAULT=6, SYS_OFF=7
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

/* ========================= RS-485 helpers ============================= */
inline void rs485RxMode(){ digitalWrite(RS485_DE_RE_PIN, LOW); }
inline void rs485TxMode(){ digitalWrite(RS485_DE_RE_PIN, HIGH); delayMicroseconds(RS485_TX_GUARD_US); }

/* ========================= CRC-16/Modbus ============================== */
uint16_t mb_crc16(const uint8_t* data, uint16_t len){
  uint16_t crc = 0xFFFF;
  for (uint16_t i=0;i<len;i++){
    crc ^= data[i];
    for (uint8_t b=0;b<8;b++){
      if (crc & 1) { crc = (crc>>1) ^ 0xA001; }
      else crc >>= 1;
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

void waitT3p5GapBeforeTx(){
  unsigned long now = micros();
  unsigned long since = now - lastBusActivityUs;
  if (since < MB_T3P5_US){
    delayMicroseconds((unsigned int)(MB_T3P5_US - since));
  }
}

/* ========================= 0x17 transaction helper =================== */
bool mb17_transact(uint8_t slaveId,
                   uint16_t readStart, uint16_t readQty,
                   uint16_t writeStart, const uint16_t* writeWords, uint16_t writeQty,
                   uint8_t* outData, uint16_t& outLen)
{
  // -------- Build request frame --------
  uint8_t req[MB_MAX_FRAME];
  uint16_t idx = 0;
  req[idx++] = slaveId;
  req[idx++] = 0x17;

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
  }

  // Read bytes until silence ≥ T3.5 or timeout
  while ((millis() - startMs) < MB_RESP_TIMEOUT_MS){
    while (MODBUS_PORT.available()){
      int b = MODBUS_PORT.read();
      if (b < 0) break;
      if (rxlen < MB_MAX_FRAME) rx[rxlen++] = (uint8_t)b;
      lastByteUs = micros();
    }
    // End-of-frame detection (silent gap)
    if ((micros() - lastByteUs) >= MB_T3P5_US) break;
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
    return false;
  }
  return true;
}

/* Convert data bytes (big-endian word stream) to uint16 regs */
void bytesToRegsBE(const uint8_t* data, uint16_t countBytes, uint16_t* outRegs, uint16_t expectRegs){
  uint16_t n = countBytes / 2;
  if (n > expectRegs) n = expectRegs;
  for (uint16_t i=0;i<n;i++){
    outRegs[i] = (uint16_t)((data[2*i] << 8) | data[2*i+1]);
  }
}

/* ========================= High-level wrappers ======================== */
bool transact_result_block(uint8_t slaveId,
                           const uint16_t* wr, uint16_t wqty,
                           uint16_t outRegs[RES_COUNT])
{
  uint8_t data[RES_COUNT*2];
  uint16_t outLen=0;

  bool ok = mb17_transact(slaveId, RES_BASE, RES_COUNT, REG_OPCODE, wr, wqty, data, outLen);
  if (!ok) return false;

  bytesToRegsBE(data, outLen, outRegs, RES_COUNT);
  return true;
}

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
}

/* -------- Conditional logging: no spam when idle -------- */
void logResultMaybe(Device& D, const uint16_t r[RES_COUNT], bool force){
  bool statusChanged = (r[2] != D.lastSYS);
  bool nonIdle       = (r[2] != SYS_IDLE);
  bool rcFail        = (r[0] != 0) || (r[1] != 0);
  bool recovering    = D.commsFault; // last cycle had comms fault

  if (!(force || statusChanged || nonIdle || rcFail || recovering)) {
    return; // suppress idle spam
  }

  Serial.print('['); Serial.print(D.name);
  Serial.print(F("] RC=")); Serial.print(r[0]);
  Serial.print(F(" ERR=")); Serial.print(r[1]);
  Serial.print(F(" SYS=")); Serial.print(sysStr(r[2]));
  Serial.print(F(" SEQ=")); Serial.print(r[3]);
  Serial.print(F(" AUX0=")); Serial.print(r[4]);
  Serial.print(F(" ELAPSEDx10=")); Serial.println(r[5]);
}

void updateDeviceFromResult(Device& D, const uint16_t r[RES_COUNT]){
  D.lastRC = r[0]; D.lastERR = r[1]; D.lastSYS = r[2]; D.lastSEQ = r[3];
  D.lastAUX = r[4]; D.lastELAPSEDx10 = r[5];
  D.commsFault = false;

  unsigned long now = millis();
  if (D.lastSYS == SYS_ACTIVE){
    D.nextStatusDueMs = now + POLL_ACTIVE_MS;
  } else {
    D.nextStatusDueMs = now + POLL_IDLE_MS;
  }
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
}
bool d_abort(){
  uint16_t wr[2] = { OP_ABORT, ++DISP.seq };
  uint16_t r[RES_COUNT];
  for (uint8_t attempt=0; attempt<=MB_MAX_RETRIES; ++attempt){
    if (transact_result_block(DISP.id, wr, 2, r)){
      logResultMaybe(DISP, r, /*force=*/true);
      updateDeviceFromResult(DISP, r);
      return true;
    }
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (abort)")); }
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
    }
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
    }
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
  }
  if (!DISP.commsFault){ DISP.commsFault = true; Serial.println(F("[DISPENSER] COMM FAULT (dispense)")); }
  return false;
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
    }
  }
  if (!CLEAN.commsFault){ CLEAN.commsFault = true; Serial.println(F("[CLEANER] COMM FAULT (clean)")); }
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

void handleCLI(){
  String cmd = readLine();
  if (!cmd.length()) return;
  String s = cmd; s.trim();

  // normalize lower for routing; keep originals for numbers
  String lower = s; lower.toLowerCase();

  if (lower == "help"){ printHelp(); return; }

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
  }

  Serial.println(F("Unknown command. Type 'help'."));
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
}

/* ========================= Setup / Loop =============================== */
void setup(){
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  rs485RxMode();                // default to receive
  Serial.begin(115200);
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);

  // Prime initial poll times
  DISP.nextStatusDueMs = CLEAN.nextStatusDueMs = millis() + 200;

  Serial.println(F("=== Modbus RTU Master (0x17-only) for Dispenser(ID=1) + Cleaner(ID=2) ==="));
  printHelp();  // show the guide on boot
}

void loop(){
  handleCLI();        // manual control
  pollScheduler();    // automatic status polling (active faster, idle slower)
}
