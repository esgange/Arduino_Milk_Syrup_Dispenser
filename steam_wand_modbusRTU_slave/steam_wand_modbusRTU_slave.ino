/*
  ┌───────────────────────────────────────────────────────────────────────────┐
  │ Steam Wand Module — Modbus RTU Slave                                     │
  │ Target MCU: Arduino Nano Every (megaAVR 4809)                            │
  └───────────────────────────────────────────────────────────────────────────┘

  Version : 1.6  (ABORT-only manual control + CLI help + structured datalogs)
  Date    : 2025-10-14  (Asia/Riyadh)
  Authors : Erdie Gange · ChatGPT 5

  What this firmware does
  ───────────────────────
  • Steam-wand controller with a single-shot Modbus RTU slave API (Function 0x17).
  • Exposes: STATUS(1), ABORT(2), INIT(10), FROTH(11), CLEAN(12).
  • **ABORT** is the sole way to enter manual control (L298 inputs Hi‑Z).
  • Boots in manual control (Hi‑Z) and **auto-enables** drivers on first use.
  • Adds **CLI help** and **structured datalogs** (see tags: [api], [run], [cfg], [event], [warn]).

  Register map (Holding, 16‑bit big‑endian)
  ──────────────────────────────────────────
    Command Block @ 0x0000
      0x0000  opcode: 1=STATUS, 2=ABORT, 10=INIT, 11=FROTH, 12=CLEAN
      0x0001  seq (echo)
      0x0002..0x0007  args (x100 scaling for seconds/°C)

    Result Block @ 0x0100
      0x0100  resultCode: 0=OK, 1=FAIL
      0x0101  errorCode : 0=NONE,1=BUSY,2=MOTOR,3=LEAK,4=SCALE,5=TIMEOUT,6=BAD_ARGS,7=INVALID_CMD
      0x0102  systemStatus: 0=IDLE,1=ACTIVE,4=SCALE_FAULT,5=TIMEOUT_FAULT
      0x0103  seq (echo)
      0x0104  aux0 = TEMP_Cx100          (signed; -32768 if invalid)
      0x0105  aux1 = elapsed_s_x10       (if applicable)
      0x0106..0x010A  reserved/0

  CLI (USB) commands (type 'help')
  ────────────────────────────────
    help
    status
    abort
    init  <seconds>
    froth <targetC> <timeout_s>
    clean <tValve_s> <tSteam_s> <tStandby_s>

  Notes
  ─────
  • External pull‑downs must be present on ENA/ENB and IN1..IN4.
  • L298 inputs are set to INPUT (Hi‑Z) in manual-control.
  • Any motor action auto‑enables ENA/ENB and sets INx as OUTPUT before driving.
*/

#include <Arduino.h>
#include <string.h>
#include <math.h>

/* ========================= Compile-time flags ========================= */
#define HAVE_L298_PULLDOWNS  1   // external pulldowns present on EN/IN lines

/* ========================= Modbus / RS-485 =========================== */
#define SLAVE_ID        2
#define MODBUS_BAUD     19200
#define MODBUS_CONFIG   SERIAL_8N1

const uint8_t RS485_RE_PIN = A1;  // RE: LOW = RX enable
const uint8_t RS485_DE_PIN = A2;  // DE: HIGH = TX enable
#define MODBUS_PORT Serial1

const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD; // ~573us @19200
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);     // ~2.0ms

/* ========================= Hardware map ============================== */
const uint8_t SOLENOID_PIN = 2;
const uint8_t M1_IN1 = 3;
const uint8_t M1_IN2 = 4;
const uint8_t M2_IN1 = 5;
const uint8_t M2_IN2 = 6;
const uint8_t ENA_PIN = 7;   // L298 ENA (M1)
const uint8_t ENB_PIN = 8;   // L298 ENB (M2)

/* MAX6675 (bit-banged) */
const uint8_t TC_SO  = 12;
const uint8_t TC_CS  = 10;
const uint8_t TC_SCK = 13;

/* ========================= App state ================================= */
enum Mode : uint8_t { MODE_OFF, MODE_STANDBY, MODE_INIT, MODE_FROTH, MODE_CLEAN };
volatile bool g_isBusy = false;
volatile bool g_abort  = false;
Mode g_currentMode     = MODE_OFF;

static unsigned long g_lastTcReadMs = 0;
static float  g_tempC   = NAN;
static bool   g_tempOk  = false;
static uint16_t g_errorMask = 0;   // bit0=TCfault, bit1=aborted, bit2=timeout
static uint32_t g_lastElapsedMs = 0;
static bool g_motorsEnabled = false;  // for auto re-enable

/* ========================= Result / API =============================== */
#define REG_SPACE_SIZE   0x0120
static uint16_t regSpace[REG_SPACE_SIZE]; // holding register image

enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };

enum ApiError : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7
};

enum Opcode : uint16_t { OP_STATUS=1, OP_ABORT=2, OP_INIT=10, OP_FROTH=11, OP_CLEAN=12 };

enum SystemStatus : uint16_t {
  SYS_IDLE=0,
  SYS_ACTIVE=1,
  /*2=MOTOR_FAULT (unused)*/
  /*3=LEAK_FAULT  (unused)*/
  SYS_SCALE_FAULT=4,
  SYS_TIMEOUT_FAULT=5
};
volatile SystemStatus systemStatus = SYS_IDLE;

volatile bool apiBusy = false;

/* ========================= Helpers: names & logging =================== */
static const __FlashStringHelper* modeName(Mode m){
  switch(m){
    case MODE_OFF: return F("OFF");
    case MODE_STANDBY: return F("STANDBY");
    case MODE_INIT: return F("INIT");
    case MODE_FROTH: return F("FROTH");
    case MODE_CLEAN: return F("CLEAN");
    default: return F("?");
  }
}
static const __FlashStringHelper* sysName(SystemStatus s){
  switch(s){
    case SYS_IDLE: return F("IDLE");
    case SYS_ACTIVE: return F("ACTIVE");
    case SYS_SCALE_FAULT: return F("SCALE_FAULT");
    case SYS_TIMEOUT_FAULT: return F("TIMEOUT_FAULT");
    default: return F("?");
  }
}
static const __FlashStringHelper* opName(uint16_t op){
  switch(op){
    case OP_STATUS: return F("STATUS");
    case OP_ABORT:  return F("ABORT");
    case OP_INIT:   return F("INIT");
    case OP_FROTH:  return F("FROTH");
    case OP_CLEAN:  return F("CLEAN");
    default:        return F("UNKNOWN");
  }
}

static void logSnapshot(const __FlashStringHelper* tag){
  Serial.print(F("[event] ")); Serial.print(tag);
  Serial.print(F(" mode=")); Serial.print(modeName(g_currentMode));
  Serial.print(F(" sys=")); Serial.print(sysName(systemStatus));
  Serial.print(F(" busy=")); Serial.print(g_isBusy?F("YES"):F("NO"));
  Serial.print(F(" motors=")); Serial.print(g_motorsEnabled?F("EN"):F("HZ"));
  Serial.print(F(" t=")); Serial.print(millis()/1000.0f,3); Serial.print(F(" s"));
  if (g_tempOk){ Serial.print(F(" temp=")); Serial.print(g_tempC,2); Serial.print(F(" C")); }
  else          { Serial.print(F(" temp=fault/open")); }
  Serial.println();
}

static void logApiStart(uint16_t opcode){
  Serial.print(F("[api] START ")); Serial.print(opName(opcode));
  Serial.print(F(" t=")); Serial.print(millis()/1000.0f,3); Serial.println(F(" s"));
}
static void logApiArgs_INIT(float secs){
  Serial.print(F("[cfg] INIT secs=")); Serial.println(secs,2);
}
static void logApiArgs_FROTH(float targetC, float timeout_s){
  Serial.print(F("[cfg] FROTH targetC=")); Serial.print(targetC,2);
  Serial.print(F(" C timeout=")); Serial.print(timeout_s,2); Serial.println(F(" s"));
}
static void logApiArgs_CLEAN(float tValve, float tSteam, float tStandby){
  Serial.print(F("[cfg] CLEAN tValve=")); Serial.print(tValve,2);
  Serial.print(F(" s tSteam=")); Serial.print(tSteam,2);
  Serial.print(F(" s tStandby=")); Serial.print(tStandby,2); Serial.println(F(" s"));
}
static void logApiBusyReject(uint16_t opcode){
  Serial.print(F("[api] REJECT BUSY for ")); Serial.println(opName(opcode));
}
static void logApiDone(uint16_t opcode, uint16_t rc, uint16_t err, unsigned long elapsedMs){
  Serial.print(F("[api] DONE  ")); Serial.print(opName(opcode));
  Serial.print(F(" rc=")); Serial.print(rc==RC_OK?F("OK"):F("FAIL"));
  Serial.print(F(" err=")); Serial.print(err);
  Serial.print(F(" elap=")); Serial.print(elapsedMs/1000.0f,3); Serial.println(F(" s"));
}

/* ========================= RS-485 / CRC =============================== */
inline void rs485RxMode(){ digitalWrite(RS485_DE_PIN, LOW); digitalWrite(RS485_RE_PIN, LOW); }
inline void rs485TxMode(){ digitalWrite(RS485_RE_PIN, HIGH); digitalWrite(RS485_DE_PIN, HIGH); }

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
  rs485TxMode();
  MODBUS_PORT.write(data, len);
  MODBUS_PORT.flush();
  rs485RxMode();
}
void mb_send_ok(const uint8_t addr, const uint8_t* pdu, uint16_t pdulen){
  uint8_t frame[260];
  frame[0]=addr;
  memcpy(frame+1, pdu, pdulen);
  uint16_t crc = mb_crc16(frame, pdulen+1);
  frame[pdulen+1] = crc & 0xFF;
  frame[pdulen+2] = (crc>>8) & 0xFF;
  mb_send(frame, pdulen+3);
}
void mb_send_exc(const uint8_t addr, uint8_t func, uint8_t code){
  uint8_t resp[3];
  resp[0] = func | 0x80;
  resp[1] = code;
  mb_send_ok(addr, resp, 2);
}

/* ========================= Register helpers =========================== */
static inline uint16_t regRead(uint16_t addr){ return (addr<REG_SPACE_SIZE)? regSpace[addr] : 0; }
static inline void     regWrite(uint16_t addr, uint16_t val){ if(addr<REG_SPACE_SIZE) regSpace[addr]=val; }

/* ========================= L298 controls ============================== */
static inline void enableMotors(){
  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); digitalWrite(ENB_PIN, HIGH);
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  g_motorsEnabled = true;
}
static inline void disableMotors(){
#if HAVE_L298_PULLDOWNS
  pinMode(ENA_PIN, INPUT); pinMode(ENB_PIN, INPUT);
  pinMode(M1_IN1, INPUT);  pinMode(M1_IN2, INPUT);
  pinMode(M2_IN1, INPUT);  pinMode(M2_IN2, INPUT);
#else
  pinMode(ENA_PIN, OUTPUT); digitalWrite(ENA_PIN, LOW);
  pinMode(ENB_PIN, OUTPUT); digitalWrite(ENB_PIN, LOW);
  pinMode(M1_IN1, OUTPUT);  digitalWrite(M1_IN1, LOW);
  pinMode(M1_IN2, OUTPUT);  digitalWrite(M1_IN2, LOW);
  pinMode(M2_IN1, OUTPUT);  digitalWrite(M2_IN1, LOW);
  pinMode(M2_IN2, OUTPUT);  digitalWrite(M2_IN2, LOW);
#endif
  g_motorsEnabled = false;
}
static inline void ensureMotorsEnabled(){ if (!g_motorsEnabled) enableMotors(); }

inline void mStop(uint8_t in1, uint8_t in2){ digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
inline void mFwd (uint8_t in1, uint8_t in2){ digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
inline void mRev (uint8_t in1, uint8_t in2){ digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

inline void M1_Stop(){ ensureMotorsEnabled(); mStop(M1_IN1, M1_IN2); }
inline void M1_Fwd (){ ensureMotorsEnabled(); mFwd (M1_IN1, M1_IN2); }
inline void M1_Rev (){ ensureMotorsEnabled(); mRev (M1_IN1, M1_IN2); }
inline void M2_Stop(){ ensureMotorsEnabled(); mStop(M2_IN1, M2_IN2); }
inline void M2_Fwd (){ ensureMotorsEnabled(); mFwd (M2_IN1, M2_IN2); }
inline void M2_Rev (){ ensureMotorsEnabled(); mRev (M2_IN1, M2_IN2); }

inline void solenoidOn()  { digitalWrite(SOLENOID_PIN, HIGH); }
inline void solenoidOff() { digitalWrite(SOLENOID_PIN, LOW);  }

unsigned long secToMs(float s){ if (s < 0) s = 0; return (unsigned long)(s*1000.0f + 0.5f); }

/* ========================= MAX6675 (bit-banged) ======================= */
uint16_t tc_readRaw(){
  uint16_t v=0;
  digitalWrite(TC_CS, LOW); delayMicroseconds(2);
  digitalWrite(TC_SCK, LOW); delayMicroseconds(1);
  for(int i=0;i<16;++i){
    digitalWrite(TC_SCK, HIGH); delayMicroseconds(1);
    v <<= 1; if (digitalRead(TC_SO)) v |= 1;
    digitalWrite(TC_SCK, LOW); delayMicroseconds(1);
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
  if (tc_openFault(raw)){ g_tempOk=false; g_tempC=NAN; return false; }
  uint16_t data = (raw>>3)&0x0FFF; c = data * 0.25f;
  g_tempOk=true; g_tempC=c; return true;
}
bool tc_tryReadC_nonblocking(float &c){
  unsigned long now = millis();
  if (now - g_lastTcReadMs >= 220UL){
    g_lastTcReadMs = now; uint16_t raw = tc_readRaw();
    if (!tc_openFault(raw)){ g_tempC = ((raw>>3)&0x0FFF) * 0.25f; g_tempOk = true; }
    else { g_tempOk=false; g_tempC=NAN; }
  }
  c = g_tempC; return g_tempOk;
}

/* ========================= State helpers ============================== */
void toOff(){
  M1_Stop(); M2_Stop(); solenoidOff();
  disableMotors();
  g_currentMode=MODE_OFF;
}
void toStandby(){
  M1_Rev(); M2_Fwd(); solenoidOff();
  g_currentMode=MODE_STANDBY;
}

/* ========================= Result builder ============================= */
static uint16_t currentApiError(){
  if (systemStatus == SYS_TIMEOUT_FAULT) return AE_TIMEOUT;
  if (systemStatus == SYS_SCALE_FAULT)   return AE_SCALE;
  return AE_NONE;
}
static void buildResult(uint16_t rc, uint16_t seq, uint16_t err){
  int16_t tempx100 = g_tempOk ? (int16_t)lroundf(g_tempC*100.0f) : (int16_t)-32768;
  uint16_t elap_x10 = (uint16_t)(((g_lastElapsedMs)+50)/100); // 0.1 s units
  regWrite(0x0100, rc);
  regWrite(0x0101, err);
  regWrite(0x0102, (uint16_t)systemStatus);
  regWrite(0x0103, seq);
  regWrite(0x0104, (uint16_t)tempx100);
  regWrite(0x0105, elap_x10);
  regWrite(0x0106, 0); regWrite(0x0107, 0);
  regWrite(0x0108, 0); regWrite(0x0109, 0); regWrite(0x010A, 0);
}

/* ========================= Mode engines =============================== */
bool pollDuringRun(uint16_t pollMs = 5){
  unsigned long start = millis();
  while (millis() - start < pollMs){ delay(1); }
  return g_abort;
}

void modeInit(float seconds){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_INIT; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0; unsigned long t0 = millis();
  Serial.println(F("[run] INIT begin"));
  solenoidOff(); M2_Rev();
  unsigned long dur = secToMs(seconds);
  while (!g_abort && (millis() - t0 < dur)) pollDuringRun(10);
  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); Serial.println(F("[run] INIT aborted")); toOff(); systemStatus = SYS_IDLE; }
  else { Serial.println(F("[run] INIT done")); toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

void modeFroth(float targetC, float timeoutSec){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_FROTH; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0; unsigned long t0 = millis();
  Serial.println(F("[run] FROTH begin"));
  solenoidOff(); M2_Rev();
  unsigned long tout = secToMs(timeoutSec);
  while (!g_abort){
    float c; bool ok = tc_readC_blocking(c);
    if (!ok){ g_errorMask |= (1<<0); systemStatus = SYS_SCALE_FAULT; Serial.println(F("[err] TC open/fault")); break; }
    if (c >= targetC){ Serial.println(F("[run] FROTH reached target")); break; }
    if (millis() - t0 > tout){ g_errorMask |= (1<<2); systemStatus = SYS_TIMEOUT_FAULT; Serial.println(F("[err] FROTH timeout")); break; }
    pollDuringRun(5);
  }
  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); Serial.println(F("[run] FROTH aborted")); toOff(); systemStatus = SYS_IDLE; }
  else if (systemStatus == SYS_ACTIVE) { Serial.println(F("[run] FROTH done→STANDBY")); toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

void modeClean(float tValve, float tSteam, float tStandby){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_CLEAN; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0; unsigned long t0 = millis();
  Serial.println(F("[run] CLEAN begin"));

  M1_Fwd();
  { unsigned long t1=millis(), dur=secToMs(tValve); while(!g_abort && (millis()-t1<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); Serial.println(F("[run] CLEAN aborted@phase0")); toOff(); g_isBusy=false; g_lastElapsedMs = millis()-t0; systemStatus = SYS_IDLE; return; }
  solenoidOn();

  { unsigned long t2=millis(), dur=secToMs(tSteam); while(!g_abort && (millis()-t2<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); Serial.println(F("[run] CLEAN aborted@phase1")); toOff(); g_isBusy=false; g_lastElapsedMs = millis()-t0; systemStatus = SYS_IDLE; return; }
  solenoidOff(); M2_Rev();

  { unsigned long t3=millis(), dur=secToMs(tStandby); while(!g_abort && (millis()-t3<dur)) pollDuringRun(5); }

  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); Serial.println(F("[run] CLEAN aborted@phase2")); toOff(); systemStatus = SYS_IDLE; }
  else { Serial.println(F("[run] CLEAN done→STANDBY")); toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

/* ========================= ABORT behavior ============================= */
void doAbort(){
  Serial.println(F("[api] ABORT requested"));
  if (g_isBusy){ g_abort = true; while (g_isBusy) { delay(1); } }
  toOff(); systemStatus = SYS_IDLE; logSnapshot(F("ABORTED_TO_MANUAL"));
}

/* ========================= Executor (0x17 API) ======================= */
static void executeOpcode(uint16_t opcode, uint16_t seq){
  unsigned long opStart = millis();

  if (g_isBusy && !(opcode==OP_STATUS || opcode==OP_ABORT)){
    logApiBusyReject(opcode);
    buildResult(RC_FAIL, seq, AE_BUSY);
    return;
  }

  logApiStart(opcode);

  switch (opcode){
    case OP_STATUS: {
      float c; (void)tc_tryReadC_nonblocking(c);
      buildResult(RC_OK, seq, AE_NONE);
      logSnapshot(F("STATUS"));
      logApiDone(opcode, RC_OK, AE_NONE, millis()-opStart);
    } break;

    case OP_ABORT: {
      doAbort();
      buildResult(RC_OK, seq, AE_NONE);
      logApiDone(opcode, RC_OK, AE_NONE, millis()-opStart);
    } break;

    case OP_INIT: {
      float secs = ((int32_t)regRead(0x0002))/100.0f;
      if (secs <= 0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); logApiDone(opcode, RC_FAIL, AE_BAD_ARGS, millis()-opStart); break; }
      logApiArgs_INIT(secs);
      modeInit(secs);
      uint16_t err = currentApiError();
      uint16_t rc  = (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) ? RC_OK : RC_FAIL;
      buildResult(rc, seq, rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD));
      logApiDone(opcode, rc, (rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD)), millis()-opStart);
    } break;

    case OP_FROTH: {
      float targetC   = ((int32_t)regRead(0x0002))/100.0f;
      float timeout_s = ((int32_t)regRead(0x0003))/100.0f;
      if (timeout_s <= 0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); logApiDone(opcode, RC_FAIL, AE_BAD_ARGS, millis()-opStart); break; }
      logApiArgs_FROTH(targetC, timeout_s);
      modeFroth(targetC, timeout_s);
      uint16_t err = currentApiError();
      uint16_t rc  = (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) ? RC_OK : RC_FAIL;
      buildResult(rc, seq, rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD));
      logApiDone(opcode, rc, (rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD)), millis()-opStart);
    } break;

    case OP_CLEAN: {
      float tValve   = ((int32_t)regRead(0x0002))/100.0f;
      float tSteam   = ((int32_t)regRead(0x0003))/100.0f;
      float tStandby = ((int32_t)regRead(0x0004))/100.0f;
      if (tValve<0 || tSteam<0 || tStandby<0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); logApiDone(opcode, RC_FAIL, AE_BAD_ARGS, millis()-opStart); break; }
      logApiArgs_CLEAN(tValve, tSteam, tStandby);
      modeClean(tValve, tSteam, tStandby);
      uint16_t err = currentApiError();
      uint16_t rc  = (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) ? RC_OK : RC_FAIL;
      buildResult(rc, seq, rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD));
      logApiDone(opcode, rc, (rc==RC_OK?AE_NONE:(err?err:AE_INVALID_CMD)), millis()-opStart);
    } break;

    default:
      buildResult(RC_FAIL, seq, AE_INVALID_CMD);
      logApiDone(opcode, RC_FAIL, AE_INVALID_CMD, millis()-opStart);
      break;
  }
}

/* ========================= Modbus 0x17 frame proc ===================== */
void modbus_process_0x17(uint8_t* frame, uint16_t len){
  uint8_t addr = frame[0];
  uint8_t func = frame[1];
  if (addr != SLAVE_ID){ return; }
  if (func != 0x17){ mb_send_exc(addr, func, 0x01); return; }

  uint16_t readStart  = (uint16_t)((frame[2]<<8) | frame[3]);
  uint16_t readQty    = (uint16_t)((frame[4]<<8) | frame[5]);
  uint16_t writeStart = (uint16_t)((frame[6]<<8) | frame[7]);
  uint16_t writeQty   = (uint16_t)((frame[8]<<8) | frame[9]);
  uint8_t  writeBytes = frame[10];

  if (writeBytes != (uint8_t)(writeQty*2)){ mb_send_exc(addr, func, 0x03); return; }
  if (len < (uint16_t)(11 + writeBytes + 2)){ return; }

  const uint8_t* writeData = frame + 11;
  for (uint16_t i=0;i<writeQty;i++){
    uint16_t val = (uint16_t)((writeData[2*i]<<8) | writeData[2*i+1]);
    if ((uint32_t)(writeStart+i) < REG_SPACE_SIZE) regWrite(writeStart + i, val);
  }

  uint16_t opcode = regRead(0x0000);
  uint16_t seq    = regRead(0x0001);

  if (g_isBusy && !(opcode==OP_STATUS || opcode==OP_ABORT)){
    logApiBusyReject(opcode);
    buildResult(RC_FAIL, seq, AE_BUSY);
  } else {
    executeOpcode(opcode, seq);
  }

  const uint16_t respBytes = (uint16_t)(readQty*2);
  uint16_t pdulen = 3 + respBytes; uint8_t* pdu = (uint8_t*)malloc(pdulen); if (!pdu) return;
  pdu[0] = func; pdu[1] = (uint8_t)respBytes;
  for (uint16_t i=0;i<readQty;i++){
    uint16_t v = regRead(readStart + i);
    pdu[2 + 2*i    ] = (uint8_t)(v >> 8);
    pdu[2 + 2*i + 1] = (uint8_t)(v & 0xFF);
  }
  mb_send_ok(addr, pdu, pdulen); free(pdu);
}

/* ========================= Gap-based poller =========================== */
void modbusPoll(){
  static uint8_t  rxBuf[260];
  static uint16_t rxLen = 0; static unsigned long lastByteUs = 0;
  while (MODBUS_PORT.available()){
    int b = MODBUS_PORT.read(); if (b < 0) break; if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = (uint8_t)b; lastByteUs = micros();
  }
  if (rxLen > 0){
    unsigned long gap = micros() - lastByteUs;
    if (gap >= MB_T3P5_US){
      if (rxLen >= 5){ uint16_t crcCalc = mb_crc16(rxBuf, (uint16_t)(rxLen-2)); uint16_t crcRx = (uint16_t)(rxBuf[rxLen-2] | (rxBuf[rxLen-1]<<8)); if (crcCalc == crcRx){ modbus_process_0x17(rxBuf, rxLen); } }
      rxLen = 0;
    }
  }
}

/* ========================= CLI helpers (USB) ========================== */
void printStatusBlocking_USB(){
  Serial.print(F("Mode=")); Serial.print(modeName(g_currentMode));
  Serial.print(F(" | Busy=")); Serial.print(g_isBusy?F("YES"):F("NO"));
  float c; bool ok = tc_readC_blocking(c);
  Serial.print(F(" | Temp=")); if(ok){ Serial.print(c,2); Serial.print(F(" C")); } else Serial.print(F("fault/open"));
  Serial.print(F(" | Motors=")); Serial.print(g_motorsEnabled?F("EN"):F("HZ"));
  Serial.print(F(" | Sys=")); Serial.println(sysName(systemStatus));
}

void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  abort"));
  Serial.println(F("  init <seconds>"));
  Serial.println(F("  froth <targetC> <timeout_s>"));
  Serial.println(F("  clean <tValve_s> <tSteam_s> <tStandby_s>"));
}

void execCliOpcode(uint16_t opcode){
  // Directly reuse executor path for consistent logs
  executeOpcode(opcode, /*seq*/0);
}

String readLineNonBlocking_USB(){
  static String buf; while (Serial.available()){ char ch=(char)Serial.read(); if (ch=='\r') continue; if (ch=='\n'){ String out=buf; buf=""; out.trim(); return out; } buf += ch; } return String();
}

/* ========================= Setup / Loop =============================== */
void setup(){
  pinMode(SOLENOID_PIN, OUTPUT); solenoidOff();
  disableMotors(); // boot in manual control (Hi-Z)

  pinMode(TC_CS, OUTPUT); pinMode(TC_SCK, OUTPUT); pinMode(TC_SO, INPUT);
  digitalWrite(TC_CS, HIGH); digitalWrite(TC_SCK, LOW);

  pinMode(RS485_RE_PIN, OUTPUT); pinMode(RS485_DE_PIN, OUTPUT); rs485RxMode();

  Serial.begin(115200);
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);
  memset(regSpace, 0, sizeof(regSpace));

  Serial.println(F("=== Steam Wand Modbus RTU (v1.6 | single-shot 0x17) ==="));
  Serial.println(F("Opcodes: STATUS(1) ABORT(2) INIT(10) FROTH(11) CLEAN(12)"));
  Serial.println(F("Boot: manual-control (L298 inputs Hi-Z; external pull-downs required)"));
  Serial.println(F("Type 'help' for CLI."));
}

void loop(){
  modbusPoll();
  float c; (void)tc_tryReadC_nonblocking(c); // keep cache fresh

  String cmd = readLineNonBlocking_USB();
  if (cmd.length()){
    String lower = cmd; lower.toLowerCase(); lower.trim();
    if (lower == "help"){ printHelp(); }
    else if (lower == "status"){ execCliOpcode(OP_STATUS); printStatusBlocking_USB(); }
    else if (lower == "abort"){ execCliOpcode(OP_ABORT); }
    else if (lower.startsWith("init ")){
      float secs = lower.substring(5).toFloat();
      regWrite(0x0000, OP_INIT); regWrite(0x0001, 0); regWrite(0x0002, (uint16_t)lroundf(secs*100.0f));
      executeOpcode(OP_INIT, 0);
    }
    else if (lower.startsWith("froth ")){
      int sp = lower.indexOf(' ', 6); if (sp<0){ Serial.println(F("Use: froth <targetC> <timeout_s>")); }
      else {
        float targetC = lower.substring(6, sp).toFloat();
        float tout = lower.substring(sp+1).toFloat();
        regWrite(0x0000, OP_FROTH); regWrite(0x0001, 0);
        regWrite(0x0002, (uint16_t)lroundf(targetC*100.0f));
        regWrite(0x0003, (uint16_t)lroundf(tout*100.0f));
        executeOpcode(OP_FROTH, 0);
      }
    }
    else if (lower.startsWith("clean ")){
      // clean <tValve> <tSteam> <tStandby>
      String rest = lower.substring(6); rest.trim();
      int s1 = rest.indexOf(' '); if (s1<0){ Serial.println(F("Use: clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _end; }
      int s2 = rest.indexOf(' ', s1+1); if (s2<0){ Serial.println(F("Use: clean <tValve_s> <tSteam_s> <tStandby_s>")); goto _end; }
      float tValve = rest.substring(0, s1).toFloat();
      float tSteam = rest.substring(s1+1, s2).toFloat();
      float tStby  = rest.substring(s2+1).toFloat();
      regWrite(0x0000, OP_CLEAN); regWrite(0x0001, 0);
      regWrite(0x0002, (uint16_t)lroundf(tValve*100.0f));
      regWrite(0x0003, (uint16_t)lroundf(tSteam*100.0f));
      regWrite(0x0004, (uint16_t)lroundf(tStby*100.0f));
      executeOpcode(OP_CLEAN, 0);
    }
    else {
      Serial.println(F("Unknown command. Type 'help'."));
    }
  }
_end:;
}
