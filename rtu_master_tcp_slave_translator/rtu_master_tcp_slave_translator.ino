

#include <Arduino.h>
#include <string.h>


#define RS485_EN_PIN 2              
#define RS485       Serial1         
#define BUS_BAUD    19200


const uint16_t TCHAR_US = (uint16_t)((11.0f / BUS_BAUD) * 1000000.0f + 0.5f); 
const uint16_t T3P5_US  = (uint16_t)(3.5f * TCHAR_US);                        


#define MB_FUNC_RW_MREGS 0x17


#define REG_CMD_BASE  0x0000
#define REG_RES_BASE  0x0100
const uint16_t RES_QTY = 0x000B; 


#define DISP_SLAVE_ID     1
enum DispOpcode : uint16_t { D_OP_STATUS=1, D_OP_CLEAR=2, D_OP_ABORT=3, D_OP_DISPENSE=10, D_OP_RINSE=11, D_OP_TRIGGER=12 };


#define CLEAN_SLAVE_ID    2
enum CleanOpcode : uint16_t { C_OP_STATUS=1, C_OP_CLEAR=2, C_OP_ABORT=3, C_OP_OFF=4, C_OP_INIT=10, C_OP_FROTH=11, C_OP_CLEAN=12 };


enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };
enum ApiError   : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};
enum SystemStatus : uint16_t {
  SYS_IDLE=0, SYS_ACTIVE=1, SYS_MOTOR_FAULT=2, SYS_LEAK_FAULT=3, SYS_SCALE_FAULT=4, SYS_TIMEOUT_FAULT=5, SYS_ABORTED_FAULT=6, SYS_OFF=7
};


const uint16_t STATUS_POLL_MS = 150;


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


inline void rs485Rx() { digitalWrite(RS485_EN_PIN, LOW); }
inline void rs485Tx() { digitalWrite(RS485_EN_PIN, HIGH); }


void purgeRx() { while (RS485.available()) (void)RS485.read(); }

bool waitForByte(uint32_t deadlineMs) {
  while ((long)(deadlineMs - millis()) >= 0) {
    if (RS485.available()) return true;
    delayMicroseconds(100);
  }
  return false;
}

bool readExact(uint8_t* buf, size_t n, uint32_t deadlineMs) {
  size_t got = 0;
  while ((long)(deadlineMs - millis()) >= 0 && got < n) {
    int b = RS485.read();
    if (b >= 0) buf[got++] = (uint8_t)b;
    else delayMicroseconds(50);
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


bool mbReadWriteMultiple(uint8_t slaveId,
                         uint16_t readStart, uint16_t readQty,
                         uint16_t writeStart, const uint16_t* writeRegs, uint16_t writeQty,
                         uint16_t* respRegs, uint16_t respRegsLen,
                         uint32_t overallTimeoutMs,
                         bool verboseOnError = true)
{
  const uint8_t func = MB_FUNC_RW_MREGS;
  const uint8_t writeByteCount = (uint8_t)(writeQty * 2);
  const uint16_t pduLen  = 1+1 +2+2 +2+2 +1 + writeByteCount; 
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
  delayMicroseconds(20);
  RS485.write(frame, idx);
  RS485.flush();
  rs485Rx();
  delayMicroseconds(200);

  
  const uint16_t expectDataBytes = (uint16_t)(readQty * 2);

  uint8_t hdr[3];
  uint32_t deadline = millis() + overallTimeoutMs;
  if (!readExact(hdr, 3, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on header"));
    return false;
  }

  
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

  
  const uint16_t rem = expectDataBytes + 2;
  uint8_t rest[300];
  if (!readExact(rest, rem, deadline)) {
    if (verboseOnError) Serial.println(F("[modbus] timeout on payload"));
    return false;
  }

  
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

  
  if (respRegsLen < readQty) return false;
  for (uint16_t i=0;i<readQty;i++) {
    respRegs[i] = (uint16_t)((rest[2*i] << 8) | rest[2*i+1]);
  }
  return true;
}


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


bool motorTokenToId(const String& token, uint16_t& outId) {
  String t = token; t.trim();
  String tl = t; tl.toLowerCase();

  
  bool allDigits = true;
  for (uint16_t i=0;i<tl.length();++i) if (!isDigit(tl[i])) { allDigits=false; break; }
  if (allDigits && tl.length()>0) { outId = (uint16_t)tl.toInt(); return true; }

  auto parseIdx = [&](const String& s, uint8_t &idx)->bool {
    long v = s.toInt(); if (v < 1 || v > 255) return false; idx = (uint8_t)v; return true;
  };

  uint8_t idx=0;
  if (tl.startsWith("sauce")) { if (!parseIdx(tl.substring(5), idx)) return false; if (idx<1||idx>15) return false; outId = (uint16_t)(20 + idx); return true; }
  if (tl.startsWith("milk"))  { if (!parseIdx(tl.substring(4), idx)) return false; if (idx<1||idx>8 ) return false; outId = (uint16_t)(10 + idx); return true; }
  if (tl.startsWith("s"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1||idx>15) return false; outId = (uint16_t)(20 + idx); return true; }
  if (tl.startsWith("m"))     { if (!parseIdx(tl.substring(1), idx)) return false; if (idx<1||idx>8 ) return false; outId = (uint16_t)(10 + idx); return true; }

  return false;
}
void idToName(uint16_t motorId, char* out, size_t cap) {
  if (motorId >= 11 && motorId <= 18) {
    snprintf(out, cap, "Milk%u", (unsigned)(motorId - 10));
  } else if (motorId >= 21 && motorId <= 35) {
    snprintf(out, cap, "Sauce%u", (unsigned)(motorId - 20));
  } else {
    snprintf(out, cap, "ID%u", (unsigned)motorId);
  }
}



struct PendingOp {
  uint8_t kind = 0;
  uint32_t deadline = 0;
  uint32_t nextPollDue = 0;
  bool printedTimeout = false;
  bool active = false;
};

enum : uint8_t { D_PK_NONE=0, D_PK_RINSE, D_PK_TRIGGER, D_PK_DISPENSE };

struct DeviceDisp {
  const char* name = "DISPENSER";
  const uint8_t id = DISP_SLAVE_ID;
  PendingOp pend;
  bool abortRequested = false;
} DISP;


static void d_logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void d_logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void d_logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void d_logSent_RINSE(float seconds) {
  Serial.print(F("-------------\n[sent] RINSE seconds=")); Serial.print(seconds,1); Serial.println(F("."));
}
static void d_logSent_TRIGGER(uint16_t motorId, float seconds, bool highSpeed) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] TRIGGER motor=")); Serial.print(name);
  Serial.print(F(" seconds=")); Serial.print(seconds,1);
  Serial.print(F(" speed=")); Serial.print(highSpeed ? F("HIGH") : F("LOW"));
  Serial.println(F("."));
}
static void d_logSent_DISPENSE(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s) {
  char name[16]; idToName(motorId, name, sizeof(name));
  Serial.print(F("-------------\n[sent] DISPENSE motor=")); Serial.print(name);
  Serial.print(F(" target_g=")); Serial.print(target_g,1);
  Serial.print(F(" slowPct=")); Serial.print(slow_g,1);
  Serial.print(F(" viscousPct=")); Serial.print(soft_g,1);
  Serial.print(F(" timeout_s=")); Serial.print(timeout_s); Serial.println(F("."));
}


void d_printResultBrief(const uint16_t* R) {
  Serial.print(F("result=")); Serial.print(R[0]==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errStr(R[1]));
  Serial.print(F("  status=")); Serial.print(sysStr(R[2]));
  Serial.println();
}


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
bool d_api_trigger(uint16_t motorId, float seconds, bool highSpeed, uint16_t* outRegs) {
  uint16_t seconds_x10 = (uint16_t)(seconds*10.0f + 0.5f);
  uint16_t flags = highSpeed ? 0x0001 : 0x0000;
  uint16_t W[5] = { D_OP_TRIGGER, 0, motorId, seconds_x10, flags };
  d_logSent_TRIGGER(motorId, seconds, highSpeed);
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 5, outRegs, RES_QTY, 2000);
}
bool d_api_dispense(uint16_t motorId, float target_g, float slow_g, float soft_g, uint16_t timeout_s, uint16_t* outRegs) {
  auto to_x10 = [](float f)->uint16_t { long v = (long)(f*10.0f + 0.5f); if (v<0) v=0; if (v>65535) v=65535; return (uint16_t)v; };
  uint16_t W[7] = { D_OP_DISPENSE, 0, motorId, to_x10(target_g), to_x10(slow_g), to_x10(soft_g), timeout_s };
  d_logSent_DISPENSE(motorId, target_g, slow_g, soft_g, timeout_s);
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 7, outRegs, RES_QTY, 2000);
}


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
  return false;
}


const char* d_pendingLabel(uint8_t k){ return (k==D_PK_RINSE)?"rinse":(k==D_PK_TRIGGER)?"trigger":(k==D_PK_DISPENSE)?"dispense":"?"; }
void d_armPending(uint8_t k, uint32_t extraMs){ DISP.pend.kind=k; DISP.pend.printedTimeout=false; DISP.pend.nextPollDue=millis()+STATUS_POLL_MS; DISP.pend.deadline=millis()+extraMs; DISP.pend.active=true; }
void d_clearPending(){ DISP.pend.kind=D_PK_NONE; DISP.pend.deadline=0; DISP.pend.nextPollDue=0; DISP.pend.printedTimeout=false; DISP.pend.active=false; }

bool d_pollStatusOnce(uint16_t* R) {
  uint16_t W[2] = { D_OP_STATUS, 0 };
  return mbReadWriteMultiple(DISP.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, R, RES_QTY, 600, false);
}


bool d_status(bool forcePrint=false){
  uint16_t r[RES_QTY];
  if (d_api_status(r, 1000)){
    if (forcePrint){
      Serial.print(F("[DISPENSER] STATUS sys=")); Serial.print(sysStr(r[2]));
      if (r[1]) { Serial.print(F(" | err=")); Serial.print(errStr(r[1])); }
      if (r[4]) { Serial.print(F(" | weight_g=")); Serial.print(r[4]/10.0f,1); }
      Serial.println();
    } else {
      
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
  if (forcePrint) Serial.println(F("[DISPENSER] STATUS -> NO REPLY"));
  return false;
}

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

  if (!DISP.pend.printedTimeout && (long)(now - DISP.pend.deadline) > 0){
    Serial.print('['); Serial.print(d_pendingLabel(DISP.pend.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    DISP.pend.printedTimeout = true;
  }
  DISP.pend.nextPollDue = millis() + STATUS_POLL_MS;
}



struct DeviceClean {
  const char* name = "CLEANER";
  const uint8_t id = CLEAN_SLAVE_ID;
  PendingOp pend;
  bool abortRequested = false;
} CLEAN;


static void c_logSent_STATUS() { Serial.println(F("-------------\n[sent] STATUS.")); }
static void c_logSent_CLEAR()  { Serial.println(F("-------------\n[sent] CLEAR.")); }
static void c_logSent_ABORT()  { Serial.println(F("-------------\n[sent] ABORT.")); }
static void c_logSent_OFF()    { Serial.println(F("-------------\n[sent] OFF.")); }
static void c_logSent_INIT(float seconds) {
  Serial.print(F("-------------\n[sent] INIT seconds=")); Serial.print(seconds,1); Serial.println(F("."));
}
static void c_logSent_FROTH(float targetC, float timeout_s) {
  Serial.print(F("-------------\n[sent] FROTH targetC=")); Serial.print(targetC,1);
  Serial.print(F("C timeout_s=")); Serial.print(timeout_s,1); Serial.println(F("."));
}
static void c_logSent_CLEAN(float tValve, float tSteam, float tStandby) {
  Serial.print(F("-------------\n[sent] CLEAN tValve_s=")); Serial.print(tValve,1);
  Serial.print(F(" tSteam_s=")); Serial.print(tSteam,1);
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby,1); Serial.println(F("."));
}


void c_printResultBrief(const uint16_t* R) {
  Serial.print(F("result=")); Serial.print(R[0]==RC_OK ? F("OK") : F("FAIL"));
  Serial.print(F("  error=")); Serial.print(errStr(R[1]));
  Serial.print(F("  status=")); Serial.print(sysStr(R[2]));
  if (R[4]) { Serial.print(F("  tempC=")); Serial.print(R[4]/10.0f,1); }
  Serial.println();
}


bool c_api_status(uint16_t* outRegs, uint32_t timeoutMs=600) {
  uint16_t W[2] = { C_OP_STATUS, 0 };
  return mbReadWriteMultiple(CLEAN.id, REG_RES_BASE, RES_QTY, REG_CMD_BASE, W, 2, outRegs, RES_QTY, timeoutMs);
}
bool c_api_status_logged(uint16_t* outRegs, uint32_t timeoutMs=2000) {
  c_logSent_STATUS();
  return c_api_status(outRegs, timeoutMs);
}
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
  if (forcePrint) Serial.println(F("[CLEANER] STATUS -> NO REPLY"));
  return false;
}


const char* c_pendingLabel(uint8_t k){ return (k==1)?"init":(k==2)?"froth":(k==3)?"clean":"?"; }
void c_armPending(uint8_t k, uint32_t extraMs){ CLEAN.pend.kind=k; CLEAN.pend.printedTimeout=false; CLEAN.pend.nextPollDue=millis()+STATUS_POLL_MS; CLEAN.pend.deadline=millis()+extraMs; CLEAN.pend.active=true; }

void c_servicePending(){
  if (!CLEAN.pend.active) return;
  uint32_t now = millis();

  if (CLEAN.abortRequested){
    uint16_t R[RES_QTY]={0};
    bool ok = c_api_abort(R,800);
    CLEAN.abortRequested=false;
    if (ok){ c_printResultBrief(R); CLEAN.pend.nextPollDue = now + 100; }
    else   { Serial.println(F("[abort] NO RESPONSE / CRC/timeout")); }
    return;
  }

  if ((long)(now - CLEAN.pend.nextPollDue) < 0) return;

  uint16_t R[RES_QTY]={0};
  bool ok = c_api_status(R,600);
  if (!ok){ CLEAN.pend.nextPollDue = millis()+STATUS_POLL_MS; return; }

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

  if (!CLEAN.pend.printedTimeout && (long)(now - CLEAN.pend.deadline) > 0){
    Serial.print('['); Serial.print(c_pendingLabel(CLEAN.pend.kind)); Serial.println(F("] wait timeout; still ACTIVE"));
    CLEAN.pend.printedTimeout = true;
  }
  CLEAN.pend.nextPollDue = millis() + STATUS_POLL_MS;
}



String inLine;

void printHelp() {
  Serial.println(F("Commands (prefix d.=dispenser, c.=cleaner):"));
  Serial.println(F("  d.status"));
  Serial.println(F("  d.clear"));
  Serial.println(F("  d.abort"));
  Serial.println(F("  d.rinse <seconds>"));
  Serial.println(F("  d.trigger <motorId|name> <seconds> [s|speed]   e.g. 'd.trigger 22 1.0 s' or 'd.trigger Sauce2 1.0 speed'"));
  Serial.println(F("  d.dispense <motorId|name> <target_g> <slowPct0-100> <viscousPct0-100> <timeout_s>"));
  Serial.println(F("  c.status"));
  Serial.println(F("  c.clear"));
  Serial.println(F("  c.abort"));
  Serial.println(F("  c.off"));
  Serial.println(F("  c.init  <seconds>"));
  Serial.println(F("  c.froth <targetC> <timeout_s>"));
  Serial.println(F("  c.clean <tValve_s> <tSteam_s> <tStandby_s>"));
}

void setup() {
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) {}

  RS485.begin(BUS_BAUD);     
  delay(50);

  Serial.println(F("\nNano Every Modbus Master ready (Dispenser+Cleaner)."));
  printHelp();

  
  uint16_t R[RES_QTY] = {0};
  Serial.print(F("\n[boot] Probing Dispenser STATUS...\n"));
  if (d_api_status_logged(R, 1500)) { Serial.println(F("Modbus OK")); d_printResultBrief(R); }
  else                               Serial.println(F("NO RESPONSE / CRC"));

  Serial.print(F("\n[boot] Probing Cleaner STATUS...\n"));
  if (c_api_status_logged(R, 1500)) { Serial.println(F("Modbus OK")); c_printResultBrief(R); }
  else                               Serial.println(F("NO RESPONSE / CRC"));
}

void loop() {
  
  d_servicePending();
  c_servicePending();

  
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      inLine.trim();
      if (!inLine.length()) continue;
      String cmd = inLine; inLine = "";
      String low = cmd; low.toLowerCase();

      
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
          int sp1 = rest.indexOf(' ');
          if (sp1 < 0) { Serial.println(F("Use: d.trigger <motorId|name> <seconds> [s|speed]")); }
          else {
            String tok  = rest.substring(0, sp1); tok.trim();
            String tail = rest.substring(sp1+1); tail.trim();

            int sp2 = tail.indexOf(' ');
            String ssec, smod;
            if (sp2 < 0) { ssec = tail; smod = ""; }
            else { ssec = tail.substring(0, sp2); smod = tail.substring(sp2+1); smod.trim(); }

            bool highSpeed = false;
            if (smod.length()) {
              String ml = smod; ml.toLowerCase();
              if (ml == "s" || ml == "speed") highSpeed = true;
              else { Serial.println(F("Use: d.trigger <motorId|name> <seconds> [s|speed]")); goto _after_cli; }
            }

            uint16_t motorId;
            if (!motorTokenToId(tok, motorId)) { Serial.println(F("[err] unknown motor token")); goto _after_cli; }
            float secs = ssec.toFloat();

            uint16_t R[RES_QTY]={0};
            bool ok = d_api_trigger(motorId, secs, highSpeed, R);
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
            Serial.println(F("Use: d.dispense <motorId|name> <target_g> <slowPct0-100> <viscousPct0-100> <timeout_s>"));
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
              c_armPending(1, (uint32_t)(secs*1000.0f)+guard);
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
                c_armPending(2, (uint32_t)(tout*1000.0f)+guard);
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
              c_armPending(3, total);
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
      
      d_servicePending();
      c_servicePending();

    } else {
      if (inLine.length() < 160) inLine += c;
    }
  }

  
  d_servicePending();
  c_servicePending();
}
