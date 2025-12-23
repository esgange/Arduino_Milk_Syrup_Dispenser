

#include <Arduino.h>
#include <string.h>
#include <math.h>


#define HAVE_L298_PULLDOWNS  1


#define MODBUS_SLAVE_ID  2
#define MODBUS_BAUD      19200
#define MODBUS_CONFIG    SERIAL_8N1

const uint8_t RS485_RE_PIN = A1;
const uint8_t RS485_DE_PIN = A2;
#define MODBUS_PORT Serial1


const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD;
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);


#define MB_MAX_READ_QTY 64


const uint8_t SOLENOID_PIN = 2;
const uint8_t M1_IN1 = 3;
const uint8_t M1_IN2 = 4;
const uint8_t M2_IN1 = 5;
const uint8_t M2_IN2 = 6;
const uint8_t ENA_PIN = 7;
const uint8_t ENB_PIN = 8;


const uint8_t TC_SO  = 12;
const uint8_t TC_CS  = 10;
const uint8_t TC_SCK = 13;


const unsigned long MAX_TIMEOUT_MS   = 10UL * 60UL * 1000UL;


enum Mode : uint8_t { MODE_OFF, MODE_STANDBY, MODE_INIT, MODE_FROTH, MODE_CLEAN };
Mode currentMode = MODE_OFF;

volatile bool apiBusy      = false;
volatile bool opBusy       = false;
volatile bool abortRequested = false;


enum ActiveOp : uint8_t { OP_NONE, OP_INIT_NB, OP_FROTH_NB, OP_CLEAN_NB };
volatile ActiveOp activeOp = OP_NONE;

static unsigned long opStartMs = 0;
static unsigned long opPhaseEndMs = 0; 
static uint8_t cleanPhase = 0;         


static uint16_t targetC_x10 = 0;
static uint16_t timeout_s_x10 = 0;


static uint16_t initSeconds_x10 = 0;
static uint16_t clean_tValve_x10 = 0, clean_tSteam_x10 = 0, clean_tStandby_x10 = 0;


static unsigned long lastTcReadMs = 0;
static float  tempC   = NAN;
static bool   tempOk  = false;


#define REG_SPACE_SIZE   0x0120
static uint16_t regSpace[REG_SPACE_SIZE];

enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };

enum ApiError : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};

enum Opcode : uint16_t {
  OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_OFF=4, OP_INIT=10, OP_FROTH=11, OP_CLEAN=12
};

enum SystemStatus : uint16_t {
  SYS_IDLE=0,
  SYS_ACTIVE=1,
  SYS_MOTOR_FAULT=2,   
  SYS_LEAK_FAULT=3,    
  SYS_SCALE_FAULT=4,   
  SYS_TIMEOUT_FAULT=5,
  SYS_ABORTED_FAULT=6,
  SYS_OFF=7            
};
volatile SystemStatus systemStatus = SYS_IDLE;


static const __FlashStringHelper* systemStatusStr(){
  switch (systemStatus){
    case SYS_IDLE:           return F("IDLE");
    case SYS_ACTIVE:         return F("ACTIVE");
    case SYS_MOTOR_FAULT:    return F("MOTOR_FAULT");
    case SYS_LEAK_FAULT:     return F("LEAK_FAULT");
    case SYS_SCALE_FAULT:    return F("SCALE_FAULT");
    case SYS_TIMEOUT_FAULT:  return F("TIMEOUT_FAULT");
    case SYS_ABORTED_FAULT:  return F("ABORTED_FAULT");
    case SYS_OFF:            return F("OFF");
    default:                 return F("UNKNOWN");
  }
}

static inline void printCommonNoTime(){
  Serial.print(F(" mode="));
  switch (currentMode){
    case MODE_OFF:     Serial.print(F("OFF")); break;
    case MODE_STANDBY: Serial.print(F("STANDBY")); break;
    case MODE_INIT:    Serial.print(F("INIT")); break;
    case MODE_FROTH:   Serial.print(F("FROTH")); break;
    case MODE_CLEAN:   Serial.print(F("CLEAN")); break;
  }
  Serial.print(F(" sys=")); Serial.print(systemStatusStr());
  Serial.print(F(" busy=")); Serial.print(opBusy ? F("YES") : F("NO"));
  Serial.println();
}
static inline void printCommonEndWithTime(){
  Serial.print(F(" t="));
  float t = (opStartMs ? (millis() - opStartMs) / 1000.0f : 0.0f);
  Serial.print(t, 3);
  Serial.println(F(" s"));
}


void logBegin_INIT(uint16_t secs_x10){
  Serial.print(F("[event] INIT_BEGIN"));
  Serial.print(F(" secs=")); Serial.print(secs_x10/10.0f, 1);
  printCommonNoTime();
}
void logBegin_FROTH(uint16_t targetCx10, uint16_t timeoutSx10){
  Serial.print(F("[event] FROTH_BEGIN"));
  Serial.print(F(" targetC=")); Serial.print(targetCx10/10.0f, 1);
  Serial.print(F("C"));
  Serial.print(F(" timeout_s=")); Serial.print(timeoutSx10/10.0f, 1);
  printCommonNoTime();
}
void logBegin_CLEAN(uint16_t tValve_x10, uint16_t tSteam_x10, uint16_t tStandby_x10){
  Serial.print(F("[event] CLEAN_P0_BEGIN"));
  Serial.print(F(" tValve_s="));   Serial.print(tValve_x10/10.0f, 1);
  Serial.print(F(" tSteam_s="));   Serial.print(tSteam_x10/10.0f, 1);
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby_x10/10.0f, 1);
  printCommonNoTime();
}
void logBegin_CLEAN_P1(uint16_t tSteam_x10){
  Serial.print(F("[event] CLEAN_P1_STEAM"));
  Serial.print(F(" tSteam_s=")); Serial.print(tSteam_x10/10.0f, 1);
  printCommonNoTime();
}
void logBegin_CLEAN_P2(uint16_t tStandby_x10){
  Serial.print(F("[event] CLEAN_P2_STANDBY"));
  Serial.print(F(" tStandby_s=")); Serial.print(tStandby_x10/10.0f, 1);
  printCommonNoTime();
}


void logEnd(const __FlashStringHelper* label){
  Serial.print(F("[event] ")); Serial.print(label);
  printCommonEndWithTime();
}
void logEnd_FrothTargetReached(float tempC_now){
  Serial.print(F("[event] FROTH_TARGET_REACHED"));
  Serial.print(F(" tempC=")); Serial.print(tempC_now, 1);
  printCommonEndWithTime();
}


inline void rs485RxMode(){ digitalWrite(RS485_DE_PIN, LOW); digitalWrite(RS485_RE_PIN, LOW); }
inline void rs485TxMode(){ digitalWrite(RS485_RE_PIN, HIGH); digitalWrite(RS485_DE_PIN, HIGH); delayMicroseconds(150); } 

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

void mb_send_ok(const uint8_t addr, const uint8_t* pdu, uint16_t pdulen){
  uint8_t frame[260];
  frame[0]=addr;
  memcpy(frame+1, pdu, pdulen);
  uint16_t crc = mb_crc16(frame, pdulen+1);
  frame[pdulen+1] = crc & 0xFF;           
  frame[pdulen+2] = (crc>>8) & 0xFF;      
  rs485TxMode();
  MODBUS_PORT.write(frame, pdulen+3);
  MODBUS_PORT.flush();
  rs485RxMode();
  delayMicroseconds(MB_CHAR_US);          
}
void mb_send_exc(const uint8_t addr, uint8_t func, uint8_t code){
  uint8_t resp[3]; resp[0] = (uint8_t)(func | 0x80); resp[1] = code;
  mb_send_ok(addr, resp, 2);
}


static inline uint16_t regRead(uint16_t addr){ return (addr<REG_SPACE_SIZE)? regSpace[addr] : 0; }
static inline void     regWrite(uint16_t addr, uint16_t val){ if(addr<REG_SPACE_SIZE) regSpace[addr]=val; }


static bool motorsEnabled = false;

static inline void enableMotors(){
  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); digitalWrite(ENB_PIN, HIGH);
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  motorsEnabled = true;
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
  motorsEnabled = false;
}
static inline void ensureMotorsEnabled(){ if (!motorsEnabled) enableMotors(); }

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

static inline void standbyDrive(){
  ensureMotorsEnabled();
  M2_Fwd();
  M1_Rev();
}

static inline unsigned long sec10_to_ms(uint16_t s_x10){
  
  unsigned long ms = (unsigned long)((s_x10 * 100UL)); 
  if (ms > MAX_TIMEOUT_MS) ms = MAX_TIMEOUT_MS;
  return ms;
}


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

bool tc_tryReadC_nonblocking(float &c){
  unsigned long now = millis();
  if (now - lastTcReadMs >= 220UL){
    lastTcReadMs = now; uint16_t raw = tc_readRaw();
    if (!tc_openFault(raw)){ tempC = ((raw>>3)&0x0FFF) * 0.25f; tempOk = true; }
    else { tempOk=false; tempC=NAN; }
  }
  c = tempC; return tempOk;
}


void allOutputsSafe(){
  solenoidOff();
  if (motorsEnabled){ M1_Stop(); M2_Stop(); }
  disableMotors(); 
  currentMode = MODE_OFF;
}


void clearSystem(){
  allOutputsSafe();

  
  opBusy = false; abortRequested = false;
  activeOp = OP_NONE; cleanPhase = 0;
  opStartMs = 0; opPhaseEndMs = 0;

  
  targetC_x10 = timeout_s_x10 = 0;
  initSeconds_x10 = 0;
  clean_tValve_x10 = 0; clean_tSteam_x10 = 0; clean_tStandby_x10 = 0;

  
  systemStatus = SYS_IDLE;

  
  regWrite(0x0105, 0); 

  
  Serial.print(F("[event] CLEAR"));
  printCommonNoTime();
}


void abortNow(){
  unsigned long t0 = millis();

  
  abortRequested = true;

  
  opBusy = false;
  activeOp = OP_NONE;
  cleanPhase = 0;

  
  solenoidOff();
  standbyDrive();
  currentMode = MODE_STANDBY;

  
  systemStatus = SYS_ABORTED_FAULT;

  
  uint16_t elap_x10 = (uint16_t)(((millis()-t0) + 50) / 100);
  regWrite(0x0105, elap_x10);

  
  Serial.print(F("[event] ABORTED→IDLE_POSTURE"));
  printCommonNoTime();
}


void offNow(){
  
  opBusy = false;
  activeOp = OP_NONE;
  cleanPhase = 0;
  abortRequested = false;

  
  allOutputsSafe();

  
  systemStatus = SYS_OFF;

  
  Serial.print(F("[event] OFF"));
  printCommonNoTime();
}


static uint16_t faultToApiError(uint16_t st){
  switch (st){
    case SYS_TIMEOUT_FAULT: return AE_TIMEOUT;
    case SYS_ABORTED_FAULT: return AE_ABORTED;
    case SYS_MOTOR_FAULT:   return AE_MOTOR;   
    case SYS_LEAK_FAULT:    return AE_LEAK;    
    case SYS_SCALE_FAULT:   return AE_SCALE;   
    default: return AE_NONE;
  }
}

static void buildResult(uint16_t rc, uint16_t seq, uint16_t err){
  
  uint16_t tempx10 = 0;
  if (tempOk && isfinite(tempC)){
    long v = lroundf(tempC * 10.0f);
    if (v < 0) v = 0; if (v > 65535) v = 65535;
    tempx10 = (uint16_t)v;
  }

  regWrite(0x0100, rc);
  regWrite(0x0101, err);
  regWrite(0x0102, (uint16_t)systemStatus);
  regWrite(0x0103, seq);
  regWrite(0x0104, tempx10);
  
  regWrite(0x0106, 0); regWrite(0x0107, 0); regWrite(0x0108, 0); regWrite(0x0109, 0); regWrite(0x010A, 0);
}


static void startInitSeconds(uint16_t seconds_x10){
  if (seconds_x10 == 0){ return; }
  initSeconds_x10 = seconds_x10;

  solenoidOff(); M2_Rev();

  currentMode = MODE_INIT;
  systemStatus = SYS_ACTIVE;
  activeOp = OP_INIT_NB;
  opBusy = true; abortRequested = false;
  opStartMs = millis();
  opPhaseEndMs = opStartMs + sec10_to_ms(initSeconds_x10);

  logBegin_INIT(initSeconds_x10);
}

static void startFroth(uint16_t targetCx10, uint16_t timeoutSx10){
  targetC_x10 = targetCx10;
  timeout_s_x10 = timeoutSx10;

  solenoidOff(); M2_Rev();

  currentMode = MODE_FROTH;
  systemStatus = SYS_ACTIVE;
  activeOp = OP_FROTH_NB;
  opBusy = true; abortRequested = false;
  opStartMs = millis();
  opPhaseEndMs = opStartMs + sec10_to_ms(timeout_s_x10);

  logBegin_FROTH(targetC_x10, timeout_s_x10);
}

static void startClean(uint16_t tValve_x10, uint16_t tSteam_x10, uint16_t tStandby_x10){
  clean_tValve_x10 = tValve_x10;
  clean_tSteam_x10 = tSteam_x10;
  clean_tStandby_x10 = tStandby_x10;

  currentMode = MODE_CLEAN;
  systemStatus = SYS_ACTIVE;
  activeOp = OP_CLEAN_NB;
  opBusy = true; abortRequested = false;
  opStartMs = millis();

  
  M1_Fwd(); solenoidOff();
  cleanPhase = 0;
  opPhaseEndMs = opStartMs + sec10_to_ms(clean_tValve_x10);
  logBegin_CLEAN(clean_tValve_x10, clean_tSteam_x10, clean_tStandby_x10);
}


static void finishOpSuccess(const __FlashStringHelper* tag){
  solenoidOff();
  standbyDrive();                  
  currentMode = MODE_STANDBY;

  systemStatus = SYS_IDLE;
  opBusy = false; activeOp = OP_NONE; cleanPhase = 0;

  uint16_t elap_x10 = (uint16_t)(((millis()-opStartMs)+50)/100);
  regWrite(0x0105, elap_x10);

  logEnd(tag);
}


static void finishOpFault(SystemStatus st, const __FlashStringHelper* tag){
  if (st != SYS_ABORTED_FAULT) {
    allOutputsSafe();              
  } 

  systemStatus = st;
  opBusy = false; activeOp = OP_NONE; cleanPhase = 0;

  uint16_t elap_x10 = (uint16_t)(((millis()-opStartMs)+50)/100);
  regWrite(0x0105, elap_x10);

  logEnd(tag);
}

static void pumpCore(){
  
  float c; (void)tc_tryReadC_nonblocking(c);

  if (activeOp == OP_NONE) return;

  
  if (abortRequested){
    finishOpFault(SYS_ABORTED_FAULT, F("ABORTED_DURING_OP"));
    return;
  }

  unsigned long now = millis();

  switch (activeOp){
    case OP_INIT_NB: {
      if ((long)(now - opPhaseEndMs) >= 0){
        finishOpSuccess(F("INIT_DONE"));
      }
    } break;

    case OP_FROTH_NB: {
      if (tempOk){
        long curCx10 = lroundf(tempC * 10.0f);
        if ((long)curCx10 >= (long)targetC_x10){
          solenoidOff();
          standbyDrive();          
          currentMode = MODE_STANDBY;
          systemStatus = SYS_IDLE;
          opBusy = false; activeOp = OP_NONE; cleanPhase = 0;
          uint16_t elap_x10 = (uint16_t)(((now - opStartMs)+50)/100);
          regWrite(0x0105, elap_x10);
          logEnd_FrothTargetReached(tempC);
          break;
        }
      }
      if ((long)(now - opPhaseEndMs) >= 0){
        finishOpFault(SYS_TIMEOUT_FAULT, F("FROTH_TIMEOUT"));
        break;
      }
    } break;

    case OP_CLEAN_NB: {
      if (cleanPhase == 0){
        if ((long)(now - opPhaseEndMs) >= 0){
          
          solenoidOn();
          cleanPhase = 1;
          opPhaseEndMs = now + sec10_to_ms(clean_tSteam_x10);
          logBegin_CLEAN_P1(clean_tSteam_x10);
        }
      } else if (cleanPhase == 1){
        if ((long)(now - opPhaseEndMs) >= 0){
          
          solenoidOff(); M2_Rev();
          cleanPhase = 2;
          opPhaseEndMs = now + sec10_to_ms(clean_tStandby_x10);
          logBegin_CLEAN_P2(clean_tStandby_x10);
        }
      } else { 
        if ((long)(now - opPhaseEndMs) >= 0){
          finishOpSuccess(F("CLEAN_DONE"));
        }
      }
    } break;

    default: break;
  }
}


static void executeOpcode(uint16_t opcode, uint16_t seq){
  
  bool hasFault =
    (systemStatus == SYS_TIMEOUT_FAULT) ||
    (systemStatus == SYS_ABORTED_FAULT);

  if (hasFault && !(opcode == OP_CLEAR || opcode == OP_ABORT || opcode == OP_STATUS || opcode == OP_OFF)){
    buildResult(RC_FAIL, seq, faultToApiError(systemStatus));
    return;
  }

  if (opBusy && !(opcode == OP_STATUS || opcode == OP_ABORT || opcode == OP_OFF)){
    buildResult(RC_FAIL, seq, AE_BUSY);
    return;
  }

  switch (opcode){
    case OP_STATUS: {
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_CLEAR: {
      clearSystem();
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_ABORT: {
      abortNow();
      buildResult(RC_FAIL, seq, AE_ABORTED);
    } break;

    case OP_OFF: {
      offNow();
      buildResult(RC_OK, seq, AE_NONE);   
    } break;

    case OP_INIT: {
      uint16_t seconds_x10 = regRead(0x0002);
      if (seconds_x10 == 0){
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }
      startInitSeconds(seconds_x10);
      if (opBusy || systemStatus == SYS_ACTIVE){
        buildResult(RC_OK, seq, AE_NONE); 
      } else {
        uint16_t err = faultToApiError(systemStatus);
        buildResult(RC_FAIL, seq, err ? err : AE_INVALID_CMD);
      }
    } break;

    case OP_FROTH: {
      uint16_t tgtCx10 = regRead(0x0002);
      uint16_t tout_x10 = regRead(0x0003);
      if (tout_x10 == 0){
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }
      startFroth(tgtCx10, tout_x10);
      if (opBusy || systemStatus == SYS_ACTIVE){
        buildResult(RC_OK, seq, AE_NONE);
      } else {
        uint16_t err = faultToApiError(systemStatus);
        buildResult(RC_FAIL, seq, err ? err : AE_INVALID_CMD);
      }
    } break;

    case OP_CLEAN: {
      uint16_t tValve_x10   = regRead(0x0002);
      uint16_t tSteam_x10   = regRead(0x0003);
      uint16_t tStandby_x10 = regRead(0x0004);
      if (tValve_x10==0 && tSteam_x10==0 && tStandby_x10==0){
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }
      startClean(tValve_x10, tSteam_x10, tStandby_x10);
      if (opBusy || systemStatus == SYS_ACTIVE){
        buildResult(RC_OK, seq, AE_NONE);
      } else {
        uint16_t err = faultToApiError(systemStatus);
        buildResult(RC_FAIL, seq, err ? err : AE_INVALID_CMD);
      }
    } break;

    default:
      buildResult(RC_FAIL, seq, AE_INVALID_CMD);
      break;
  }
}


static void process0x17(uint8_t* frame, uint16_t len){
  uint8_t addr = frame[0];
  uint8_t func = frame[1];
  if (addr != MODBUS_SLAVE_ID){ return; }
  if (func != 0x17){ mb_send_exc(addr, func, 0x01); return; }

  if (len < 13){ return; } 

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

  apiBusy = true;
  executeOpcode(opcode, seq);
  apiBusy = false;

  
  if (readQty > MB_MAX_READ_QTY){ mb_send_exc(addr, func, 0x03); return; }

  const uint16_t respBytes = (uint16_t)(readQty*2);
  uint16_t pdulen = (uint16_t)(2 + respBytes);     

  static uint8_t pdu[2 + 2*MB_MAX_READ_QTY];
  pdu[0] = func; pdu[1] = (uint8_t)respBytes;
  for (uint16_t i=0;i<readQty;i++){
    uint16_t v = regRead((uint16_t)(readStart + i));
    pdu[2 + 2*i    ] = (uint8_t)(v >> 8);
    pdu[2 + 2*i + 1] = (uint8_t)(v & 0xFF);
  }
  mb_send_ok(addr, pdu, pdulen);
}


void handleModbus(){
  static uint8_t  rxBuf[260];
  static uint16_t rxLen = 0; static unsigned long lastByteUs = 0;

  while (MODBUS_PORT.available()){
    int b = MODBUS_PORT.read(); if (b < 0) break;
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = (uint8_t)b;
    lastByteUs = micros();
  }
  if (rxLen > 0){
    unsigned long gap = micros() - lastByteUs;
    if (gap >= MB_T3P5_US){
      if (rxLen >= 5){
        uint16_t crcCalc = mb_crc16(rxBuf, (uint16_t)(rxLen-2));
        uint16_t crcRx = (uint16_t)(rxBuf[rxLen-2] | (uint16_t)(rxBuf[rxLen-1]<<8));
        if (crcCalc == crcRx){
          process0x17(rxBuf, rxLen);
        }
      }
      rxLen = 0;
    }
  }
}


void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  clear"));
  Serial.println(F("  abort"));
  Serial.println(F("  off"));
  Serial.println(F("  init  <seconds>"));
  Serial.println(F("  froth <targetC> <timeout_s>"));
  Serial.println(F("  clean <tValve> <tSteam> <tStandby>"));
}

void execCliOpcode(uint16_t opcode){
  executeOpcode(opcode, 0);
}

String readLineNonBlocking_USB(){
  static String buf; while (Serial.available()){ char ch=(char)Serial.read(); if (ch=='\r') continue; if (ch=='\n'){ String out=buf; buf=""; out.trim(); return out; } if (buf.length()<120) buf += ch; } return String();
}


void printStatusBlocking_USB(){
  Serial.print(F("Sys=")); Serial.print(systemStatusStr());
  Serial.print(F(" | Busy=")); Serial.print(opBusy ? F("YES") : F("NO"));
  Serial.print(F(" | Mode="));
  switch (currentMode){
    case MODE_OFF:     Serial.print(F("OFF")); break;
    case MODE_STANDBY: Serial.print(F("STANDBY")); break;
    case MODE_INIT:    Serial.print(F("INIT")); break;
    case MODE_FROTH:   Serial.print(F("FROTH")); break;
    case MODE_CLEAN:   Serial.print(F("CLEAN")); break;
  }
  Serial.print(F(" | Temp="));
  if (tempOk && isfinite(tempC)) { Serial.print(tempC, 1); Serial.print(F(" C")); }
  else                           { Serial.print(F("invalid/open")); }
  Serial.println();
}

void handleCli(){
  String cmd = readLineNonBlocking_USB();
  if (!cmd.length()) return;
  String lower = cmd; lower.toLowerCase(); lower.trim();

  if (lower == "help"){ printHelp(); }
  else if (lower == "status"){
    execCliOpcode(OP_STATUS);
    printStatusBlocking_USB();
  }
  else if (lower == "clear"){ execCliOpcode(OP_CLEAR); }
  else if (lower == "abort"){ execCliOpcode(OP_ABORT); }
  else if (lower == "off"){ execCliOpcode(OP_OFF); }
  else if (lower.startsWith("init ")){
    float secs = lower.substring(5).toFloat();
    uint16_t sx10 = (uint16_t)lroundf(secs * 10.0f);
    regWrite(0x0000, OP_INIT); regWrite(0x0001, 0); regWrite(0x0002, sx10);
    executeOpcode(OP_INIT, 0);
  }
  else if (lower.startsWith("froth ")){
    int sp = lower.indexOf(' ', 6); if (sp<0){ Serial.println(F("Use: froth <targetC> <timeout_s>")); return; }
    float targetC = lower.substring(6, sp).toFloat();
    float tout    = lower.substring(sp+1).toFloat();
    regWrite(0x0000, OP_FROTH); regWrite(0x0001, 0);
    regWrite(0x0002, (uint16_t)lroundf(targetC*10.0f));
    regWrite(0x0003, (uint16_t)lroundf(tout*10.0f));
    executeOpcode(OP_FROTH, 0);
  }
  else if (lower.startsWith("clean ")){
    String rest = lower.substring(6); rest.trim();
    int s1 = rest.indexOf(' '); if (s1<0){ Serial.println(F("Use: clean <tValve> <tSteam> <tStandby>")); return; }
    int s2 = rest.indexOf(' ', s1+1); if (s2<0){ Serial.println(F("Use: clean <tValve> <tSteam> <tStandby>")); return; }
    float tValve = rest.substring(0, s1).toFloat();
    float tSteam = rest.substring(s1+1, s2).toFloat();
    float tStby  = rest.substring(s2+1).toFloat();
    regWrite(0x0000, OP_CLEAN); regWrite(0x0001, 0);
    regWrite(0x0002, (uint16_t)lroundf(tValve*10.0f));
    regWrite(0x0003, (uint16_t)lroundf(tSteam*10.0f));
    regWrite(0x0004, (uint16_t)lroundf(tStby*10.0f));
    executeOpcode(OP_CLEAN, 0);
  }
  else {
    Serial.println(F("Unknown command. Type 'help'."));
  }
}


void setup(){
  pinMode(SOLENOID_PIN, OUTPUT); solenoidOff();
  disableMotors(); 

  pinMode(TC_CS, OUTPUT); pinMode(TC_SCK, OUTPUT); pinMode(TC_SO, INPUT);
  digitalWrite(TC_CS, HIGH); digitalWrite(TC_SCK, LOW);

  pinMode(RS485_RE_PIN, OUTPUT); pinMode(RS485_DE_PIN, OUTPUT); rs485RxMode();

  Serial.begin(115200);
  MODBUS_PORT.begin(MODBUS_BAUD, MODBUS_CONFIG);
  memset(regSpace, 0, sizeof(regSpace));

  Serial.println(F("=== Steam Wand Modbus RTU (v2.4 | OFF status; ABORT→IDLE posture; no-malloc TX) ==="));
  Serial.println(F("Opcodes: STATUS(1) CLEAR(2) ABORT(3) OFF(4) INIT(10) FROTH(11) CLEAN(12)"));
  Serial.println(F("Type 'help' for CLI."));
}

void loop(){
  handleModbus();   
  pumpCore();       
  handleCli();      
}
