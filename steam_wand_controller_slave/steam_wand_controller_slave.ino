/*
  ┌───────────────────────────────────────────────────────────────────────────┐
  │ Steam Wand Module — Modbus RTU Slave                                      │
  │ Target MCU: Arduino Nano Every (megaAVR 4809)                             │
  └───────────────────────────────────────────────────────────────────────────┘

  Version : 1.1  (Modbus single-shot API aligned with Mega 2560)
  Date    : 2025-10-10  (Asia/Riyadh)
  Authors : Erdie Gange · ChatGPT 5

  Modbus RTU Slave API (single-shot, same as Mega)
  ─────────────────────────────────────────────────
  Link & UART:
    - RS485 RE -> A1 (active-LOW), RS485 DE -> A2 (active-HIGH)
    - Port     -> Serial1 @ 19200 8N1
    - Slave ID -> 2

  Function code:
    - Only 0x17 Read/Write Multiple Registers is accepted.
      Others reply Exception 0x01 (Illegal Function).

  Transaction model (synchronous, single-shot):
    - Master sends 0x17 with a **Command Block write** and **Result Block read**
      in the **same** transaction.
    - Slave executes to completion (or until it fails/aborts), then fills
      the Result Block in that same response. (Use a generous master timeout.)

  Register map (Holding registers, 16-bit big-endian per Modbus)
  ───────────────────────────────────────────────────────────────
    Command Block @ 0x0000
      0x0000  opcode: 1=STATUS, 2=CLEAR, 10=INIT, 11=FROTH, 12=CLEAN
      0x0001  seq (echoed back)
      0x0002  arg0
      0x0003  arg1
      0x0004  arg2
      0x0005  arg3
      0x0006  arg4
      0x0007  arg5

    Result Block @ 0x0100
      0x0100  resultCode: 0=OK, 1=FAIL
      0x0101  errorCode : 0=NONE,1=BUSY,2=MOTOR,3=LEAK,4=SCALE,5=TIMEOUT,6=BAD_ARGS,7=INVALID_CMD
      0x0102  systemStatus: 0=IDLE,1=ACTIVE,4=SCALE_FAULT,5=TIMEOUT_FAULT
      0x0103  seq (echo)
      0x0104  aux0 = TEMP_Cx100          (cached, signed; -32768 if invalid)
      0x0105  aux1 = elapsed_s_x10       (if applicable)
      0x0106  aux2 = 0                   (reserved)
      0x0107  aux3 = 0                   (reserved)
      0x0108  0 (unused)
      0x0109  0 (unused)
      0x010A  0 (unused)

  Opcode arguments (x100 scaling for seconds/°C)
  ───────────────────────────────────────────────
    INIT (10):   arg0 = INIT_SECSx100
    FROTH (11):  arg0 = FROTH_TARGETCx100, arg1 = FROTH_TIMEOUT_SECSx100
    CLEAN (12):  arg0 = CLEAN_T_VALVE_SECSx100
                 arg1 = CLEAN_T_STEAM_SECSx100
                 arg2 = CLEAN_T_STANDBY_SECSx100
    CLEAR (2):   If idle -> clear faults; If busy -> **abort current run** (OFF).

  Notes
  ─────
  • Core behavior (motors, solenoid, MAX6675 pacing, sequences) unchanged.
  • TC fault maps to errorCode=AE_SCALE (4). Timeout maps to AE_TIMEOUT (5).
  • Broadcast address 0 is ignored in this single-shot surface.
*/

#include <Arduino.h>
#include <string.h>
#include <math.h>

/* ───────────────────────── USER CONFIG / MODBUS ──────────────────────────── */
#define SLAVE_ID        2                 // Steam Wand = ID 2
#define MODBUS_BAUD     19200             // Match Mega bus
#define MODBUS_CONFIG   SERIAL_8N1        // Match Mega bus

/* RS-485 (MAX485) direction control */
const uint8_t RS485_RE_PIN = A1;  // RE: LOW = RX enable
const uint8_t RS485_DE_PIN = A2;  // DE: HIGH = TX enable

/* Use Serial1 for RS-485 Modbus */
#define MODBUS_PORT Serial1

/* RTU timing (≈11 bits/char @8N1 for gap calc) */
const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD;
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);

/* ───────────────────────── APPLICATION PINS / IO ─────────────────────────── */
const uint8_t SOLENOID_PIN = 2;
const uint8_t M1_IN1 = 3;
const uint8_t M1_IN2 = 4;
const uint8_t M2_IN1 = 5;
const uint8_t M2_IN2 = 6;
/* MAX6675 (bit-banged) */
const uint8_t TC_SO  = 12;
const uint8_t TC_CS  = 10;
const uint8_t TC_SCK = 13;

/* ───────────────────────────── APP STATE ─────────────────────────────────── */
enum Mode : uint8_t { MODE_OFF, MODE_STANDBY, MODE_INIT, MODE_FROTH, MODE_CLEAN };
volatile bool g_isBusy = false;
volatile bool g_abort  = false;
Mode g_currentMode     = MODE_OFF;

static unsigned long g_lastTcReadMs = 0;
static float  g_tempC   = NAN;
static bool   g_tempOk  = false;
static uint16_t g_errorMask = 0;   // bit0=TCfault, bit1=aborted, bit2=timeout

/* For result aux fields */
static uint32_t g_lastElapsedMs = 0;

/* ───────────────────────── L298 / ACTUATORS HELPERS ──────────────────────── */
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

/* ───────────────────────────── MAX6675 (BIT-BANG) ────────────────────────── */
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

/* ───────────────────────────── STATE HELPERS ─────────────────────────────── */
void toOff(){ M1_Stop(); M2_Stop(); solenoidOff(); g_currentMode=MODE_OFF; }
void toStandby(){ M1_Rev(); M2_Fwd(); solenoidOff(); g_currentMode=MODE_STANDBY; }

/* ──────────────────────────── 0x17 API SURFACE ───────────────────────────── */
#define REG_SPACE_SIZE   0x0120
static uint16_t regSpace[REG_SPACE_SIZE]; // holding register image

/* API result codes */
enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };

/* API error codes (aligned with Mega) */
enum ApiError : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7
};

/* Opcodes aligned to Mega style */
enum Opcode : uint16_t { OP_STATUS=1, OP_CLEAR=2, OP_INIT=10, OP_FROTH=11, OP_CLEAN=12 };

/* System status (reuse Mega numeric layout for compatibility) */
enum SystemStatus : uint16_t {
  SYS_IDLE=0,
  SYS_ACTIVE=1,
  /*2=MOTOR_FAULT (unused)*/
  /*3=LEAK_FAULT  (unused)*/
  SYS_SCALE_FAULT=4,   // used for TC fault
  SYS_TIMEOUT_FAULT=5
};
volatile SystemStatus systemStatus = SYS_IDLE;

/* Busy flag for single-shot executor */
volatile bool apiBusy = false;

/* ───── RS-485 / CRC / TX helpers (RTU) ───── */
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

/* Build & send normal response */
void mb_send_ok(const uint8_t addr, const uint8_t* pdu, uint16_t pdulen){
  uint8_t frame[260];
  frame[0]=addr;
  memcpy(frame+1, pdu, pdulen);
  uint16_t crc = mb_crc16(frame, pdulen+1);
  frame[pdulen+1] = crc & 0xFF;
  frame[pdulen+2] = (crc>>8) & 0xFF;
  mb_send(frame, pdulen+3);
}

/* Build & send exception */
void mb_send_exc(const uint8_t addr, uint8_t func, uint8_t code){
  uint8_t resp[3];
  resp[0] = func | 0x80;
  resp[1] = code;
  mb_send_ok(addr, resp, 2);
}

/* Simple reg helpers */
static inline uint16_t regRead(uint16_t addr){ return (addr<REG_SPACE_SIZE)? regSpace[addr] : 0; }
static inline void     regWrite(uint16_t addr, uint16_t val){ if(addr<REG_SPACE_SIZE) regSpace[addr]=val; }

/* Map faults to API error */
static uint16_t currentApiError(){
  if (systemStatus == SYS_TIMEOUT_FAULT) return AE_TIMEOUT;
  if (systemStatus == SYS_SCALE_FAULT)   return AE_SCALE; // TC fault treated as SCALE
  return AE_NONE;
}

/* Fill Result Block @0x0100 */
static void buildResult(uint16_t rc, uint16_t seq, uint16_t err){
  /* aux0=tempCx100 (signed), aux1=elapsed_s_x10, aux2/aux3=0 */
  int16_t tempx100 = g_tempOk ? (int16_t)lroundf(g_tempC*100.0f) : (int16_t)-32768;
  uint16_t elap_x10 = (uint16_t)(((g_lastElapsedMs)+50)/100); // 0.1 s units

  regWrite(0x0100, rc);
  regWrite(0x0101, err);
  regWrite(0x0102, (uint16_t)systemStatus);
  regWrite(0x0103, seq);
  regWrite(0x0104, (uint16_t)tempx100);
  regWrite(0x0105, elap_x10);
  regWrite(0x0106, 0);
  regWrite(0x0107, 0);
  regWrite(0x0108, 0);
  regWrite(0x0109, 0);
  regWrite(0x010A, 0);
}

/* ─────────────────────────── CORE MODE ENGINES ───────────────────────────── */
/* Keep bus responsive while running (no new commands accepted while busy) */
bool pollDuringRun(uint16_t pollMs = 5){
  unsigned long start = millis();
  while (millis() - start < pollMs){
    // (No re-entrancy; apiBusy stays true)
    delay(1);
  }
  return g_abort;
}

void modeInit(float seconds){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_INIT; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0;
  unsigned long t0 = millis();
  solenoidOff();
  M2_Rev();
  unsigned long dur = secToMs(seconds);
  while (!g_abort && (millis() - t0 < dur)) pollDuringRun(10);
  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); toOff(); systemStatus = SYS_IDLE; }
  else { toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

void modeFroth(float targetC, float timeoutSec){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_FROTH; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0;
  unsigned long t0 = millis(), tout = secToMs(timeoutSec);
  solenoidOff(); M2_Rev();
  while (!g_abort){
    float c;
    bool ok = tc_readC_blocking(c);
    if (!ok){ g_errorMask |= (1<<0); systemStatus = SYS_SCALE_FAULT; break; }
    if (c >= targetC) break;
    if (millis() - t0 > tout){ g_errorMask |= (1<<2); systemStatus = SYS_TIMEOUT_FAULT; break; }
    pollDuringRun(5);
  }
  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); toOff(); systemStatus = SYS_IDLE; }
  else if (systemStatus == SYS_ACTIVE) { toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

/* CLEAN (updated, unchanged logic)
   0) M1 forward
   1) after tValve  -> Solenoid ON
   2) after tSteam  -> M2 reverse AND Solenoid OFF
   3) after tStandby-> STANDBY
*/
void modeClean(float tValve, float tSteam, float tStandby){
  g_isBusy = true; g_abort = false; g_currentMode = MODE_CLEAN; systemStatus = SYS_ACTIVE;
  g_errorMask = 0; g_lastElapsedMs = 0;

  unsigned long t0 = millis();

  M1_Fwd(); // 0

  { unsigned long t1=millis(), dur=secToMs(tValve);
    while(!g_abort && (millis()-t1<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); toOff(); g_isBusy=false; g_lastElapsedMs = millis()-t0; systemStatus = SYS_IDLE; return; }
  solenoidOn();

  { unsigned long t2=millis(), dur=secToMs(tSteam);
    while(!g_abort && (millis()-t2<dur)) pollDuringRun(5); }
  if (g_abort){ g_errorMask |= (1<<1); toOff(); g_isBusy=false; g_lastElapsedMs = millis()-t0; systemStatus = SYS_IDLE; return; }
  solenoidOff();   // NEW requirement
  M2_Rev();

  { unsigned long t3=millis(), dur=secToMs(tStandby);
    while(!g_abort && (millis()-t3<dur)) pollDuringRun(5); }

  g_lastElapsedMs = millis() - t0;
  if (g_abort){ g_errorMask |= (1<<1); toOff(); systemStatus = SYS_IDLE; }
  else { toStandby(); systemStatus = SYS_IDLE; }
  g_isBusy = false;
}

/* CLEAR behavior (idle: clear faults; busy: abort) */
void doClear(){
  if (g_isBusy){
    g_abort = true;            // request abort; mode engines will exit
    while (g_isBusy) { delay(1); }
  }
  // reset flags/state
  g_errorMask = 0;
  toOff();
  systemStatus = SYS_IDLE;
}

/* ───────────────────────────── 0x17 EXECUTOR ─────────────────────────────── */
static void executeOpcode(uint16_t opcode, uint16_t seq){
  // If active and opcode not CLEAR/STATUS -> BUSY
  if (g_isBusy && !(opcode==OP_CLEAR || opcode==OP_STATUS)){
    buildResult(RC_FAIL, seq, AE_BUSY);
    return;
  }

  switch (opcode){
    case OP_STATUS: {
      // Update temperature cache quickly for aux0
      float c; (void)tc_tryReadC_nonblocking(c);
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_CLEAR: {
      doClear();
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_INIT: {
      float secs = ((int32_t)regRead(0x0002))/100.0f;
      if (secs <= 0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); break; }
      apiBusy = true;
      modeInit(secs);
      apiBusy = false;
      uint16_t err = currentApiError();
      if (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) buildResult(RC_OK, seq, AE_NONE);
      else buildResult(RC_FAIL, seq, err?err:AE_INVALID_CMD);
    } break;

    case OP_FROTH: {
      float targetC   = ((int32_t)regRead(0x0002))/100.0f;
      float timeout_s = ((int32_t)regRead(0x0003))/100.0f;
      if (timeout_s <= 0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); break; }
      apiBusy = true;
      modeFroth(targetC, timeout_s);
      apiBusy = false;
      uint16_t err = currentApiError();
      if (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) buildResult(RC_OK, seq, AE_NONE);
      else buildResult(RC_FAIL, seq, err?err:AE_INVALID_CMD);
    } break;

    case OP_CLEAN: {
      float tValve   = ((int32_t)regRead(0x0002))/100.0f;
      float tSteam   = ((int32_t)regRead(0x0003))/100.0f;
      float tStandby = ((int32_t)regRead(0x0004))/100.0f;
      if (tValve<0 || tSteam<0 || tStandby<0){ buildResult(RC_FAIL, seq, AE_BAD_ARGS); break; }
      apiBusy = true;
      modeClean(tValve, tSteam, tStandby);
      apiBusy = false;
      uint16_t err = currentApiError();
      if (err==AE_NONE && !(g_errorMask & ((1<<0)|(1<<2)))) buildResult(RC_OK, seq, AE_NONE);
      else buildResult(RC_FAIL, seq, err?err:AE_INVALID_CMD);
    } break;

    default:
      buildResult(RC_FAIL, seq, AE_INVALID_CMD);
      break;
  }
}

/* ──────────────────────── 0x17 FRAME PROCESSOR ───────────────────────────── */
void modbus_process_0x17(uint8_t* frame, uint16_t len){
  // Minimal length already checked by caller
  uint8_t addr = frame[0];
  uint8_t func = frame[1];
  if (addr != SLAVE_ID){ return; }

  if (func != 0x17){
    // Only 0x17 supported in this surface
    mb_send_exc(addr, func, 0x01);
    return;
  }

  // Header fields
  uint16_t readStart  = (uint16_t)((frame[2]<<8) | frame[3]);
  uint16_t readQty    = (uint16_t)((frame[4]<<8) | frame[5]);
  uint16_t writeStart = (uint16_t)((frame[6]<<8) | frame[7]);
  uint16_t writeQty   = (uint16_t)((frame[8]<<8) | frame[9]);
  uint8_t  writeBytes = frame[10];

  if (writeBytes != (uint8_t)(writeQty*2)){
    mb_send_exc(addr, func, 0x03); // Illegal Data Value
    return;
  }

  // Expected total length = 1+1 + 2+2 + 2+2 + 1 + writeBytes + 2(CRC)
  uint16_t totalLenNoCRC = (uint16_t)(11 + writeBytes);
  if (len < totalLenNoCRC + 2){ return; } // safety

  // CRC already validated by outer poller

  // Write incoming registers
  const uint8_t* writeData = frame + 11;
  for (uint16_t i=0;i<writeQty;i++){
    uint16_t val = (uint16_t)((writeData[2*i]<<8) | writeData[2*i+1]);
    if ((uint32_t)(writeStart+i) < REG_SPACE_SIZE) regWrite(writeStart + i, val);
  }

  // If currently running and opcode not CLEAR/STATUS -> build BUSY result now
  uint16_t opcode = regRead(0x0000);
  uint16_t seq    = regRead(0x0001);

  if (g_isBusy && !(opcode==OP_CLEAR || opcode==OP_STATUS)){
    buildResult(RC_FAIL, seq, AE_BUSY);
  } else {
    executeOpcode(opcode, seq);
  }

  // Build 0x17 response: [addr][func][byteCount][readData...][crc]
  const uint16_t respBytes = (uint16_t)(readQty*2);
  uint16_t pdulen = 3 + respBytes;
  uint8_t* pdu = (uint8_t*)malloc(pdulen);
  if (!pdu) return;
  pdu[0] = func;
  pdu[1] = (uint8_t)respBytes;
  // Read-out
  for (uint16_t i=0;i<readQty;i++){
    uint16_t v = regRead(readStart + i);
    pdu[2 + 2*i    ] = (uint8_t)(v >> 8);
    pdu[2 + 2*i + 1] = (uint8_t)(v & 0xFF);
  }
  mb_send_ok(addr, pdu, pdulen);
  free(pdu);
}

/* ────────────────────── GAP-BASED MODBUS POLLER (RTU) ────────────────────── */
void modbusPoll(){
  static uint8_t  rxBuf[260];
  static uint16_t rxLen = 0;
  static unsigned long lastByteUs = 0;

  while (MODBUS_PORT.available()){
    int b = MODBUS_PORT.read();
    if (b < 0) break;
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = (uint8_t)b;
    lastByteUs = micros();
  }

  if (rxLen > 0){
    unsigned long gap = micros() - lastByteUs;
    if (gap >= MB_T3P5_US){
      // Validate CRC
      if (rxLen >= 5){
        uint16_t crcCalc = mb_crc16(rxBuf, (uint16_t)(rxLen-2));
        uint16_t crcRx   = (uint16_t)(rxBuf[rxLen-2] | (rxBuf[rxLen-1]<<8));
        if (crcCalc == crcRx){
          modbus_process_0x17(rxBuf, rxLen);
        }
      }
      rxLen = 0;
    }
  }
}

/* ───────────────────────────── OPTIONAL USB DEBUG ────────────────────────── */
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

/* ─────────────────────────────── SETUP / LOOP ────────────────────────────── */
void setup(){
  // Actuators
  pinMode(SOLENOID_PIN, OUTPUT); solenoidOff();
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  toOff();

  // MAX6675
  pinMode(TC_CS, OUTPUT); pinMode(TC_SCK, OUTPUT); pinMode(TC_SO, INPUT);
  digitalWrite(TC_CS, HIGH); digitalWrite(TC_SCK, LOW);

  // RS-485
  pinMode(RS485_RE_PIN, OUTPUT);
  pinMode(RS485_DE_PIN, OUTPUT);
  rs485RxMode();

  // Ports
  Serial.begin(115200);                         // USB debug (optional)
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);

  // Zero register image
  memset(regSpace, 0, sizeof(regSpace));

  Serial.println(F("\n=== Steam Wand Modbus RTU (single-shot 0x17, Mega-compatible) ==="));
  Serial.println(F("Modbus: Serial1 @ 19200 8N1, Slave ID=2"));
  Serial.println(F("Opcodes: STATUS(1) CLEAR(2) INIT(10) FROTH(11) CLEAN(12)"));
}

void loop(){
  // Always service the bus
  modbusPoll();

  // Keep temp cache fresh for STATUS reads
  float c; (void)tc_tryReadC_nonblocking(c);

  // USB debug (optional)
  String cmd = readLineNonBlocking_USB();
  if (cmd.length()){
    if (cmd=="off"){ g_abort=true; toOff(); Serial.println(F("[USB] OFF")); }
    else if (cmd=="status"){ printStatusBlocking_USB(); }
  }
}
