/*
  Arduino Nano Every — Modbus RTU Master (0x17) for Mega2560 Dispenser
  RS-485: MAX485 — RO->D0(RX1), DI->D1(TX1), RE+DE->D2 (HIGH=TX, LOW=RX)
  Bus: 19200 8N1, Slave ID = 1

  Motor tokens:
    Milk1..Milk8  (or m1..m8)       -> 1..8
    Sauce1..Sauce15 (or s1..s15)    -> 101..115
    Numeric IDs are accepted too.

  CLI:
    status
    clear
    rinse <seconds>
    trigger <motorId|name> <seconds>
    dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>
*/

#include <Arduino.h>

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
enum ApiError   : uint16_t { AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7 };
enum Opcode     : uint16_t { OP_STATUS=1, OP_CLEAR=2, OP_DISPENSE=10, OP_RINSE=11, OP_TRIGGER=12 };

// We always read full result block 0x0100..0x010A (11 regs)
const uint16_t RES_QTY = 0x000B;

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
  while (millis() <= deadlineMs && got < n) {
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

  // Turnaround: purge, idle, TX, immediate DE release, then t3.5 idle
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
// Helpers: printing result
// ---------------------------------------------
const char* errToStr(uint16_t e) {
  switch (e) {
    case AE_NONE: return "NONE";
    case AE_BUSY: return "BUSY";
    case AE_MOTOR: return "MOTOR_FAULT";
    case AE_LEAK: return "LEAK_FAULT";
    case AE_SCALE: return "SCALE_FAULT";
    case AE_TIMEOUT: return "TIMEOUT_FAULT";
    case AE_BAD_ARGS: return "BAD_ARGS";
    case AE_INVALID_CMD: return "INVALID_CMD";
    default: return "?";
  }
}

void printResultRegs(const uint16_t* R) {
  uint16_t resultCode = R[0];
  uint16_t errorCode  = R[1];

  Serial.print(F("result="));
  Serial.print(resultCode==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errToStr(errorCode));
}

// ---------------------------------------------
// Motor token -> numeric ID mapper
// Accepts: "sauce2", "s2", "milk1", "m1", or numbers "102", "3"
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

// ---------------------------------------------
// API wrappers
// ---------------------------------------------
uint16_t seqCounter = 1;

bool api_status(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[8] = {0};
  W[0] = OP_STATUS;
  W[1] = seqCounter++;
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_clear(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  uint16_t W[8] = {0};
  W[0] = OP_CLEAR;
  W[1] = seqCounter++;
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}

bool api_rinse(float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OP_RINSE;
  W[1] = seqCounter++;
  W[2] = seconds_x10;
  uint32_t toMs = (uint32_t)(seconds*1000.0f) + 5000UL;
  if (toMs < 3000) toMs = 3000;
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 3, outRegs, RES_QTY, toMs);
}

bool api_trigger(uint16_t motorId, float seconds, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t W[8] = {0};
  W[0] = OP_TRIGGER;
  W[1] = seqCounter++;
  W[2] = motorId;
  W[3] = seconds_x10;
  uint32_t toMs = (uint32_t)(seconds*1000.0f) + 5000UL;
  if (toMs < 3000) toMs = 3000;
  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 4, outRegs, RES_QTY, toMs);
}

bool api_dispense(uint16_t motorId, float target_g, float slowOffset_g, float softCutOffset_g, uint16_t timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = lroundf(f*10.0f); if (v < 0) v = 0; if (v > 65535) v = 65535; return (uint16_t)v; };
  uint16_t W[8] = {0};
  W[0] = OP_DISPENSE;
  W[1] = seqCounter++;
  W[2] = motorId;
  W[3] = to_x10(target_g);
  W[4] = to_x10(slowOffset_g);
  W[5] = to_x10(softCutOffset_g);
  W[6] = timeout_s;

  uint32_t toMs = (uint32_t)timeout_s * 1000UL + 10000UL;
  if (toMs < 15000UL) toMs = 15000UL;
  if (toMs > 9UL*60UL*1000UL) toMs = 9UL*60UL*1000UL;

  return mbReadWriteMultiple(SLAVE_ID, REG_RES_BASE, RES_QTY,
                             REG_CMD_BASE, W, 7, outRegs, RES_QTY, toMs);
}

// ---------------------------------------------
// Simple USB-serial CLI
// ---------------------------------------------
String inLine;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  status"));
  Serial.println(F("  clear"));
  Serial.println(F("  rinse <seconds>"));
  Serial.println(F("  trigger <motorId|name> <seconds>   e.g. 'trigger 102 1.0' or 'trigger sauce2 1.0'"));
  Serial.println(F("  dispense <motorId|name> <target_g> <slowOffset_g> <softCutOffset_g> <timeout_s>"));
}

void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) {}

  RS485.begin(BUS_BAUD);     // 8N1 default
  delay(50);

  Serial.println(F("\nNano Every Modbus Master (0x17, names+IDs) ready."));
  printHelp();

  // Quick probe
  uint16_t R[RES_QTY] = {0};
  Serial.print(F("\n[boot] Probing STATUS... "));
  bool ok = api_status(R, 1500);
  if (ok) { Serial.println(F("OK")); printResultRegs(R); }
  else    { Serial.println(F("NO RESPONSE / CRC")); }
}

void loop() {
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
        ok = api_status(R);
        Serial.println(ok ? F("\n[status] response:") : F("[status] NO RESPONSE / CRC/timeout"));
        if (ok) printResultRegs(R);
      }
      else if (low == "clear") {
        ok = api_clear(R);
        Serial.println(ok ? F("\n[clear] response:") : F("[clear] NO RESPONSE / CRC/timeout"));
        if (ok) printResultRegs(R);
      }
      else if (low.startsWith("rinse ")) {
        String s = cmd.substring(6); s.trim();
        float secs = s.toFloat();
        ok = api_rinse(secs, R);
        Serial.println(ok ? F("\n[rinse] response:") : F("[rinse] NO RESPONSE / CRC/timeout"));
        if (ok) printResultRegs(R);
      }
      else if (low.startsWith("trigger ")) {
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
          Serial.println(ok ? F("\n[trigger] response:") : F("[trigger] NO RESPONSE / CRC/timeout"));
          if (ok) printResultRegs(R);
        }
      }
      else if (low.startsWith("dispense ")) {
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
          Serial.println(ok ? F("\n[dispense] response:") : F("[dispense] NO RESPONSE / CRC/timeout"));
          if (ok) printResultRegs(R);
        }
      }
      else {
        Serial.println(F("Unknown command. Type 'help'."));
      }
    } else {
      if (inLine.length() < 120) inLine += c;
    }
  }
}
