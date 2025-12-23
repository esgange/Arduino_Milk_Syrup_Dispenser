

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <math.h>


#define FAULT_DETECT_ENABLED 1   

const unsigned long FAULT_DEBOUNCE_MS = 500;
const unsigned long LEAK_SAMPLE_MS    = 1000;
const unsigned long TRIGGER_FAULT_DEBOUNCE_MS = 3000;


#define MODBUS_SLAVE_ID  1
#define RS485_DE_PIN     17
#define RS485_RE_PIN     17
#define MODBUS_BAUD      19200
#define MODBUS_SERIAL    Serial1


#define REG_SPACE_SIZE   0x0120
static uint16_t regSpace[REG_SPACE_SIZE]; 


enum ResultCode : uint16_t { RC_OK=0, RC_FAIL=1 };


enum ApiError : uint16_t {
  AE_NONE=0, AE_BUSY=1, AE_MOTOR=2, AE_LEAK=3, AE_SCALE=4, AE_TIMEOUT=5, AE_BAD_ARGS=6, AE_INVALID_CMD=7, AE_ABORTED=8
};


enum Opcode : uint16_t { OP_STATUS=1, OP_CLEAR=2, OP_ABORT=3, OP_DISPENSE=10, OP_RINSE=11, OP_TRIGGER=12 };


volatile bool apiBusy = false;


static uint16_t mb_crc16(const uint8_t* data, uint16_t len);
static void rs485RxMode();
static void rs485TxMode();
static void handleModbus();         
static void buildResult(uint16_t rc, uint16_t seq, uint16_t err);
static uint16_t faultToApiError(uint8_t systemStatus);
const uint32_t MB_CHAR_US = (11UL * 1000000UL) / MODBUS_BAUD; 
const uint32_t MB_T3P5_US = (uint32_t)(MB_CHAR_US * 3.5);     

#define MB_MAX_READ_QTY  64  
static uint8_t mb_resp_buf[3 + 2*MB_MAX_READ_QTY + 2];


const uint8_t MILK_SCALE_ADDR  = 0x2A; 
const uint8_t SAUCE_SCALE_ADDR = 0x2B; 
const uint8_t MILK_SCALE_ID    = 1;
const uint8_t SAUCE_SCALE_ID   = 2;

enum ScaleSel : uint8_t { SCALE_MILK=0, SCALE_SAUCE=1 };
volatile ScaleSel activeScale = SCALE_MILK;

uint16_t lastWeightMilk_x10  = 0;
uint16_t lastWeightSauce_x10 = 0;

inline uint8_t activeScaleAddr() { return (activeScale == SCALE_MILK) ? MILK_SCALE_ADDR : SAUCE_SCALE_ADDR; }
inline uint8_t activeScaleId()   { return (activeScale == SCALE_MILK) ? MILK_SCALE_ID   : SAUCE_SCALE_ID; }
const __FlashStringHelper* activeScaleName() { return (activeScale == SCALE_MILK) ? F("MILK") : F("SAUCE"); }


const uint8_t PIN_MOTOR_FAULT  = 51;    
const uint8_t PIN_RINSER       = 49;    


const uint8_t PIN_SPEEDSEL     = 23;


const uint8_t MILK_PINS[8]  = {2,3,4,5,6,7,8,9};

const uint8_t SAUCE_PINS[15]= {10,11,12,25,27,29,31,33,35,37,39,41,43,45,47};

const char* MILK_NAMES[8] = {
  "Milk1","Milk2","Milk3","Milk4","Milk5","Milk6","Milk7","Milk8"
};
const char* SAUCE_NAMES[15] = {
  "Sauce1","Sauce2","Sauce3","Sauce4","Sauce5","Sauce6","Sauce7","Sauce8",
  "Sauce9","Sauce10","Sauce11","Sauce12","Sauce13","Sauce14","Sauce15"
};


struct LeakChannel {
  uint8_t inPin;     
  uint8_t vccPin;    
  const char* label;
};

LeakChannel LEAK[2] = {
  { A5, A6, "CH0(A5)" },
  { A8, A9, "CH1(A8)" }
};

unsigned long leakLastSampleMs = 0;
unsigned long leakStartMs = 0;


const unsigned long POLL_INTERVAL_MS = 10;
const unsigned long SETTLE_MS        = 150;
const unsigned long MAX_TIMEOUT_MS   = 10UL * 60UL * 1000UL;

enum SysState : uint8_t { ST_IDLE, ST_DISPENSING, ST_COMPLETE };
SysState state = ST_IDLE;

uint16_t lastWeight_x10 = 0;
unsigned long lastPollMs = 0;

int   activeMotorPin = -1;
char  activeMotorName[16] = "";

uint16_t target_x10        = 0;
uint16_t softThreshold_x10 = 0;   
uint16_t lowSpeed_x10      = 0;   

unsigned long timeout_ms   = 30000;
unsigned long startMs      = 0;
unsigned long softStopMs   = 0;

bool lowSpeedApplied   = false;
bool softCutTriggered  = false;

bool timeoutRetryActive = false;
unsigned long timeoutRetryEndMs = 0;

float    motorStartDelay_s = 0.5f;
bool     motorStartPending = false;
unsigned long motorStartDueMs = 0;

unsigned long motorFaultStartMs = 0;
unsigned long motorFaultStartMsTrigger = 0;
bool motorFaultDebouncedTrigger = false;

bool rinseActive = false;
unsigned long rinseEndMs = 0;

bool diagTriggerActive = false;
uint8_t diagTriggerPin = 255;
unsigned long diagTriggerEndMs = 0;
bool diagRevertToInput = false;


enum SystemStatus : uint8_t {
  SYS_IDLE=0,
  SYS_ACTIVE=1,
  SYS_MOTOR_FAULT=2,
  SYS_LEAK_FAULT=3,
  SYS_SCALE_FAULT=4,
  SYS_TIMEOUT_FAULT=5,
  SYS_ABORTED_FAULT=6
};
volatile SystemStatus systemStatus = SYS_IDLE;

volatile bool faultFlag = false;

bool motorFaultLogged  = false;
bool leakFaultLogged   = false;
bool scaleFaultLogged  = false;
bool timeoutFaultLogged= false;

const __FlashStringHelper* systemStatusStr() {
  switch (systemStatus) {
    case SYS_IDLE:           return F("IDLE");
    case SYS_ACTIVE:         return F("ACTIVE");
    case SYS_MOTOR_FAULT:    return F("MOTOR FAULT");
    case SYS_LEAK_FAULT:     return F("LEAK FAULT");
    case SYS_SCALE_FAULT:    return F("SCALE FAULT");
    case SYS_TIMEOUT_FAULT:  return F("DISPENSE TIMEOUT FAULT");
    case SYS_ABORTED_FAULT:  return F("ABORTED");
    default:                 return F("UNKNOWN");
  }
}


void logEvent(const __FlashStringHelper* label);


inline bool motorFaultActiveRaw() { return digitalRead(PIN_MOTOR_FAULT) == LOW; }

void updateMotorFaultTriggerDebounce() {
  unsigned long now = millis();
  if (motorFaultActiveRaw()) {
    if (motorFaultStartMsTrigger == 0) motorFaultStartMsTrigger = now;
    if (!motorFaultDebouncedTrigger &&
        (long)(now - motorFaultStartMsTrigger) >= (long)TRIGGER_FAULT_DEBOUNCE_MS) {
      motorFaultDebouncedTrigger = true;
    }
  } else {
    motorFaultStartMsTrigger = 0;
    motorFaultDebouncedTrigger = false;
  }
}

bool isMotorOrSystemOutputPin(uint8_t pin) {
  if (pin == PIN_RINSER || pin == PIN_SPEEDSEL) return true;
  for (uint8_t i=0;i<8;i++)  if (pin == MILK_PINS[i])  return true;
  for (uint8_t i=0;i<15;i++) if (pin == SAUCE_PINS[i]) return true;
  return false;
}

bool isMilkPin(uint8_t pin)  { for (uint8_t i=0;i<8;i++)  if (pin == MILK_PINS[i])  return true; return false; }
bool isSaucePin(uint8_t pin) { for (uint8_t i=0;i<15;i++) if (pin == SAUCE_PINS[i]) return true; return false; }


inline void setSpeedHigh() { digitalWrite(PIN_SPEEDSEL, HIGH); }   
inline void setSpeedLow()  { digitalWrite(PIN_SPEEDSEL, LOW); }    

void allMotorsOff() {
  for (uint8_t i=0;i<8;i++)  digitalWrite(MILK_PINS[i], LOW);
  for (uint8_t i=0;i<15;i++) digitalWrite(SAUCE_PINS[i], LOW);
}


void allOutputsSafe() {
  allMotorsOff();
  digitalWrite(PIN_RINSER, LOW);
  setSpeedLow();    
}

void stopActiveMotor() { if (activeMotorPin >= 0) digitalWrite(activeMotorPin, LOW); }
void startActiveMotor(){ if (activeMotorPin >= 0) digitalWrite(activeMotorPin, HIGH); }


bool readScaleOnce(uint16_t& w_x10) {
  if (Wire.requestFrom((int)activeScaleAddr(), 3) == 3) {
    (void)Wire.read();
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    w_x10 = (uint16_t)(lo | (uint16_t(hi) << 8));
    return true;
  } else {
    while (Wire.available()) (void)Wire.read();
    faultFlag = true;

    if (!scaleFaultLogged) {
#if FAULT_DETECT_ENABLED
      if (state == ST_DISPENSING) logEvent(F("SCALE_FAULT_ABORT"));
      else                        logEvent(F("SCALE_FAULT"));
#endif
      scaleFaultLogged = true;
    }

#if FAULT_DETECT_ENABLED
    systemStatus = SYS_SCALE_FAULT;
    if (state == ST_DISPENSING) {
      stopActiveMotor();
      allOutputsSafe();
      state = ST_IDLE;
    }
#endif
    return false;
  }
}

void pollScaleIfDue() {
  if (millis() - lastPollMs < POLL_INTERVAL_MS) return;
  lastPollMs = millis();
  uint16_t w;
  if (readScaleOnce(w)) {
    lastWeight_x10 = w;
    if (activeScale == SCALE_MILK)  lastWeightMilk_x10  = w;
    else                            lastWeightSauce_x10 = w;
  }
}


bool parseMotorSelector(const String& token, int &pinOut, char *nameOut, size_t nameCap) {
  String t = token; t.trim(); t.toLowerCase();

  auto pick = [&](int idx, const char* const* names, const uint8_t* pins, int max)->bool{
    if (idx>=1 && idx<=max) { pinOut = pins[idx-1]; strncpy(nameOut, names[idx-1], nameCap); return true; }
    return false;
  };

  if (t.startsWith("milk"))  return pick(t.substring(4).toInt(), MILK_NAMES,  MILK_PINS,  8);
  if (t.startsWith("sauce")) return pick(t.substring(5).toInt(), SAUCE_NAMES, SAUCE_PINS, 15);
  if (t.startsWith("m"))     return pick(t.substring(1).toInt(), MILK_NAMES,  MILK_PINS,  8);
  if (t.startsWith("s"))     return pick(t.substring(1).toInt(), SAUCE_NAMES, SAUCE_PINS, 15);

  bool numeric = true;
  for (uint16_t i=0;i<t.length();++i) if (!isDigit(t[i])) { numeric=false; break; }
  if (numeric && t.length()>0) {
    int pin = t.toInt();
    for (uint8_t i=0;i<8;i++)  if (pin == (int)MILK_PINS[i])  { pinOut=pin; strncpy(nameOut, MILK_NAMES[i], nameCap);  return true; }
    for (uint8_t i=0;i<15;i++) if (pin == (int)SAUCE_PINS[i]) { pinOut=pin; strncpy(nameOut, SAUCE_NAMES[i], nameCap); return true; }
  }
  return false;
}


void logEvent(const __FlashStringHelper* label) {
  Serial.print(F("[event] "));
  Serial.print(label);
  Serial.print(F(" g, t=")); Serial.print((startMs ? (millis()-startMs)/1000.0f : 0.0f),3);
  Serial.println(F(" s"));
}


void sampleLeaksIfDue() {
  const unsigned long now = millis();
  if (now - leakLastSampleMs < LEAK_SAMPLE_MS) return;
  leakLastSampleMs = now;

  int d0 = digitalRead(LEAK[0].inPin);
  int d1 = digitalRead(LEAK[1].inPin);
  bool anyLeak = (d0 == HIGH) || (d1 == HIGH);

  if (anyLeak) {
    if (leakStartMs == 0) leakStartMs = now;
    if (now - leakStartMs >= FAULT_DEBOUNCE_MS) {
      faultFlag = true;

      if (!leakFaultLogged) {
#if FAULT_DETECT_ENABLED
        if (state == ST_DISPENSING) logEvent(F("LEAK_FAULT_ABORT"));
        else                        logEvent(F("LEAK_FAULT"));
#endif
        leakFaultLogged = true;
      }

#if FAULT_DETECT_ENABLED
      if (state == ST_DISPENSING) {
        stopActiveMotor();
        allOutputsSafe();
        systemStatus = SYS_LEAK_FAULT;
        state = ST_IDLE;
      } else {
        systemStatus = SYS_LEAK_FAULT;
      }
#endif
    }
  } else {
    leakStartMs = 0;
  }
}


void printStatus() {
  Serial.print(F("systemStatus: "));
  Serial.print(systemStatusStr());
  Serial.print(F(" (code=")); Serial.print((int)systemStatus);
  Serial.print(F(")  faultFlag=")); Serial.println(faultFlag ? F("ON") : F("OFF"));
}

void printList() {
  Serial.println(F("=== Motors ==="));
  for (uint8_t i=0;i<8;i++){
    Serial.print(MILK_NAMES[i]); Serial.print(F(" -> pin ")); Serial.print(MILK_PINS[i]);
    Serial.print(F(" : ")); Serial.println(digitalRead(MILK_PINS[i])?F("ON"):F("OFF"));
  }
  for (uint8_t i=0;i<15;i++){
    Serial.print(SAUCE_NAMES[i]); Serial.print(F(" -> pin ")); Serial.print(SAUCE_PINS[i]);
    Serial.print(F(" : ")); Serial.println(digitalRead(SAUCE_PINS[i])?F("ON"):F("OFF"));
  }
  if (diagTriggerActive) {
    Serial.print(F("diag trigger: pin ")); Serial.print(diagTriggerPin);
    Serial.print(F(" ends in ~")); Serial.print((long)(diagTriggerEndMs - millis())/1000.0f,1);
    Serial.println(F(" s"));
  }
}




void startDispense(int pin, const char* name, float slowOffset_g, float softCutOffset_g, unsigned long timeout_s, uint16_t tgt_x10) {
  if (timeout_s == 0) timeout_s = 30;
  if (timeout_s*1000UL > MAX_TIMEOUT_MS) timeout_s = MAX_TIMEOUT_MS/1000UL;
  timeout_ms = timeout_s * 1000UL;

  if (slowOffset_g < 0) slowOffset_g = 0;
  if (slowOffset_g > 100.0f) slowOffset_g = 100.0f;
  if (softCutOffset_g < 0) softCutOffset_g = 0;
  if (softCutOffset_g > 100.0f) softCutOffset_g = 100.0f;

  activeScale = isMilkPin((uint8_t)pin) ? SCALE_MILK : SCALE_SAUCE;

  
  Wire.beginTransmission(activeScaleAddr());
  Wire.write('R');
  Wire.write((uint8_t)(tgt_x10 & 0xFF));
  Wire.write((uint8_t)((tgt_x10 >> 8) & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    faultFlag = true;
    if (!scaleFaultLogged) {
      logEvent(F("SCALE_FAULT"));
      scaleFaultLogged = true;
    }
#if FAULT_DETECT_ENABLED
    systemStatus = SYS_SCALE_FAULT;
#endif
    return;
  }

  target_x10 = tgt_x10;
  float target_g = target_x10 / 10.0f;

  
  float slowThreshold_g = (slowOffset_g / 100.0f) * target_g;
  if (slowThreshold_g < 0.0f) slowThreshold_g = 0.0f;
  if (slowThreshold_g > target_g) slowThreshold_g = target_g;
  lowSpeed_x10 = (uint16_t)(slowThreshold_g * 10.0f + 0.5f);

  
  const float maxSoftOffset_g = 0.5f;
  float softOffset_g = maxSoftOffset_g * (0.5f - (softCutOffset_g / 100.0f));
  if (softOffset_g < 0.0f) softOffset_g = 0.0f;
  uint16_t softOff_x10 = (uint16_t)(softOffset_g * 10.0f + 0.5f);

  softThreshold_x10 = (softOff_x10 >= target_x10) ? 0 : (uint16_t)(target_x10 - softOff_x10);

  activeMotorPin = pin;
  strncpy(activeMotorName, name, sizeof(activeMotorName)-1);
  activeMotorName[sizeof(activeMotorName)-1] = '\0';

  
  setSpeedHigh();
  lowSpeedApplied = false;

  digitalWrite(PIN_RINSER, LOW);

  unsigned long delayMs = (unsigned long)(motorStartDelay_s * 1000.0f + 0.5f);
  if (delayMs == 0) {
    startActiveMotor();
    motorStartPending = false;
    motorStartDueMs = 0;
  } else {
    motorStartPending = true;
    motorStartDueMs = millis() + delayMs;
  }
  motorFaultStartMs = 0;

  softCutTriggered = false;
  timeoutRetryActive = false;
  timeoutRetryEndMs = 0;
  lastPollMs = millis() - POLL_INTERVAL_MS;
  startMs = millis();
  softStopMs = 0;
  state = ST_DISPENSING;
  systemStatus = SYS_ACTIVE;

  if (softThreshold_x10 < lowSpeed_x10 && softThreshold_x10 != 0) {
    Serial.println(F("[warn] softCutAt < slowAt: soft cut may occur before slow-down"));
  }

  Serial.print(F("[cfg][scale=")); Serial.print(activeScaleName()); Serial.print(F(" id=")); Serial.print(activeScaleId());
  Serial.print(F(" addr=0x")); Serial.print(activeScaleAddr(),HEX); Serial.print(F("] slowAt="));
  Serial.print(lowSpeed_x10/10.0f,1); Serial.print(F(" g, "));
  Serial.print(F("softCutAt=")); Serial.print(softThreshold_x10/10.0f,1); Serial.print(F(" g, "));
  Serial.print(F("target=")); Serial.print(target_x10/10.0f,1); Serial.println(F(" g"));

  Serial.print(F("[dispense] ")); Serial.print(activeMotorName);
  Serial.print(F(" target=")); Serial.print(target_x10/10.0f,1); Serial.print(F(" g"));
  Serial.print(F(" slowPct=")); Serial.print(slowOffset_g,1); Serial.print(F(" %"));
  Serial.print(F(" viscousPct=")); Serial.print(softCutOffset_g,1); Serial.print(F(" %"));
  Serial.print(F(" timeout=")); Serial.print(timeout_s);
  Serial.print(F(" [scale=")); Serial.print(activeScaleName()); Serial.print(F(" id=")); Serial.print(activeScaleId());
  Serial.print(F(" addr=0x")); Serial.print(activeScaleAddr(),HEX); Serial.println(F("]"));
}

void handleDispensing() {
  const unsigned long now = millis();

  
  if (!timeoutRetryActive) {
    if (now - startMs > timeout_ms) {
      
      timeoutRetryActive = true;
      timeoutRetryEndMs = now + 3000UL;   

      
      setSpeedLow();
      startActiveMotor();                 
      logEvent(F("TIMEOUT_RETRY"));
    }
  } else {
    if ((long)(now - timeoutRetryEndMs) >= 0) {
      
      stopActiveMotor();
      allOutputsSafe();
      faultFlag = true;
      systemStatus = SYS_TIMEOUT_FAULT;
      if (!timeoutFaultLogged) {
        logEvent(F("TIMEOUT_FAULT"));
        timeoutFaultLogged = true;
      }
      state = ST_IDLE;
      timeoutRetryActive = false;
      return;
    }
  }

  pollScaleIfDue();

  if (motorStartPending) {
    if ((long)(now - motorStartDueMs) >= 0) {
      startActiveMotor();
      motorStartPending = false;
      logEvent(F("MOTOR_START"));
      motorFaultStartMs = 0;
    } else {
      return;
    }
  }

  if (motorFaultActiveRaw()) {
    if (motorFaultStartMs == 0) motorFaultStartMs = now;
    if (now - motorFaultStartMs >= FAULT_DEBOUNCE_MS) {
      faultFlag = true;

      if (!motorFaultLogged) {
#if FAULT_DETECT_ENABLED
        logEvent(F("MOTOR_FAULT_ABORT"));
#else
        logEvent(F("MOTOR_FAULT"));
#endif
        motorFaultLogged = true;
      }

#if FAULT_DETECT_ENABLED
      stopActiveMotor();
      allOutputsSafe();
      systemStatus = SYS_MOTOR_FAULT;
      state = ST_IDLE;
      return;
#endif
    }
  } else {
    motorFaultStartMs = 0;
  }

  if (!softCutTriggered) {
    if (!lowSpeedApplied && lastWeight_x10 >= lowSpeed_x10) {
      
      setSpeedLow();
      lowSpeedApplied = true;
      logEvent(F("SPEED_LOW"));
    }

    if (lastWeight_x10 >= target_x10) {
      stopActiveMotor();
      allOutputsSafe();
      systemStatus = SYS_IDLE;  
      logEvent(F("DONE_HARDCUT"));
      state = ST_COMPLETE;
      return;
    }

    if (softThreshold_x10 != 0 && lastWeight_x10 >= softThreshold_x10) {
      stopActiveMotor();
      softCutTriggered = true;
      softStopMs = now;
      logEvent(F("softCut_STOP"));
      return;
    }

    return;
  }

  if (now - softStopMs < SETTLE_MS) return;

  if (lastWeight_x10 >= target_x10) {
    allOutputsSafe();
    systemStatus = SYS_IDLE;
    logEvent(F("DONE_COAST"));
    state = ST_COMPLETE;
    return;
  }
}


void startRinseSeconds(float seconds) {
  if (seconds <= 0) { Serial.println(F("rinse time must be >0")); return; }
  if (state == ST_DISPENSING) { Serial.println(F("[blocked] rinse disabled during dispensing")); return; }
  unsigned long ms = (unsigned long)(seconds * 1000.0f);
  if (ms > MAX_TIMEOUT_MS) ms = MAX_TIMEOUT_MS;
  setSpeedHigh();
  digitalWrite(PIN_RINSER, HIGH);
  rinseActive = true;
  systemStatus = SYS_ACTIVE;
  rinseEndMs = millis() + ms;
  Serial.print(F("[rinse] ON for ")); Serial.print(ms/1000.0f,1); Serial.println(F(" s"));
}

void handleRinseTimer() {
  if (!rinseActive) return;

  if (motorFaultActiveRaw()) {
    const unsigned long now = millis();
    if (motorFaultStartMs == 0) motorFaultStartMs = now;
    if (now - motorFaultStartMs >= FAULT_DEBOUNCE_MS) {
      faultFlag = true;

      if (!motorFaultLogged) {
#if FAULT_DETECT_ENABLED
        logEvent(F("MOTOR_FAULT_ABORT"));
#else
        logEvent(F("MOTOR_FAULT"));
#endif
        motorFaultLogged = true;
      }

#if FAULT_DETECT_ENABLED
      rinseActive = false;
      rinseEndMs = 0;
      allOutputsSafe();
      systemStatus = SYS_MOTOR_FAULT;
      return;
#endif
    }
  } else {
    motorFaultStartMs = 0;
  }

  if ((long)(millis() - rinseEndMs) >= 0) {
    digitalWrite(PIN_RINSER, LOW);
    setSpeedLow();
    rinseActive = false;
    Serial.println(F("[rinse] OFF"));
    if (state != ST_DISPENSING && !diagTriggerActive) systemStatus = SYS_IDLE;
  }
}



void startDiagTrigger(uint8_t pin, float seconds, bool highSpeed) {
  if (state == ST_DISPENSING) { Serial.println(F("[blocked] trigger disabled during dispensing")); return; }
  if (seconds <= 0)          { Serial.println(F("trigger seconds must be >0")); return; }
  if (pin > 53)              { Serial.println(F("pin must be 0..53")); return; }
  unsigned long ms = (unsigned long)(seconds * 1000.0f);
  if (ms > MAX_TIMEOUT_MS) ms = MAX_TIMEOUT_MS;

#if FAULT_DETECT_ENABLED
  if (pin == PIN_MOTOR_FAULT) {
    Serial.println(F("[warn] pin 51 is MOTOR FAULT (ACTIVE-LOW). Driving it LOW will assert fault."));
  }
#endif

  if (highSpeed) setSpeedHigh();
  else           setSpeedLow();

  diagRevertToInput = !isMotorOrSystemOutputPin(pin);
  if (diagRevertToInput) pinMode(pin, OUTPUT);

  digitalWrite(pin, HIGH);
  diagTriggerActive = true;
  systemStatus = SYS_ACTIVE;
  diagTriggerPin = pin;
  diagTriggerEndMs = millis() + ms;

  Serial.print(F("[trigger] pin ")); Serial.print(pin); Serial.print(F(" ON for "));
  Serial.print(ms/1000.0f,1); Serial.println(F(" s"));
}

void handleDiagTriggerTimer() {
  if (!diagTriggerActive) return;
  if (motorFaultDebouncedTrigger) {
    faultFlag = true;
    if (!motorFaultLogged) { logEvent(F("MOTOR_FAULT_ABORT")); motorFaultLogged = true; }
    digitalWrite(diagTriggerPin, LOW);
    if (diagRevertToInput) pinMode(diagTriggerPin, INPUT);
    diagTriggerActive = false;
    diagTriggerPin = 255;
    diagTriggerEndMs = 0;
    diagRevertToInput = false;
    setSpeedLow();
    systemStatus = SYS_MOTOR_FAULT;
    return;
  }
  if ((long)(millis() - diagTriggerEndMs) >= 0) {
    digitalWrite(diagTriggerPin, LOW);
    if (diagRevertToInput) pinMode(diagTriggerPin, INPUT);
    Serial.print(F("[trigger] pin ")); Serial.print(diagTriggerPin); Serial.println(F(" OFF"));
    diagTriggerActive = false;
    diagTriggerPin = 255;
    if (state != ST_DISPENSING && !rinseActive) setSpeedLow();
    if (state != ST_DISPENSING && !rinseActive) systemStatus = SYS_IDLE;
  }
}


void abortNow() {
  stopActiveMotor();
  allOutputsSafe();

  rinseActive = false; rinseEndMs = 0;

  if (diagTriggerActive) {
    digitalWrite(diagTriggerPin, LOW);
    if (diagRevertToInput) pinMode(diagTriggerPin, INPUT);
    diagTriggerActive = false;
    diagTriggerPin = 255;
    diagTriggerEndMs = 0;
    diagRevertToInput = false;
  }

  state = ST_IDLE;
  motorStartPending = false;
  motorStartDueMs = 0;

  timeoutRetryActive = false;
  timeoutRetryEndMs = 0;

  systemStatus = SYS_ABORTED_FAULT;
  faultFlag = true;

  logEvent(F("ABORTED"));
}


void clearSystem() {
  stopActiveMotor();
  allOutputsSafe();

  state = ST_IDLE;
  systemStatus = SYS_IDLE;

  lastWeight_x10 = 0;
  lastPollMs = 0;

  activeMotorPin = -1;
  activeMotorName[0] = '\0';

  target_x10 = 0;
  softThreshold_x10 = 0;
  lowSpeed_x10 = 0;

  timeout_ms   = 30000;
  startMs = 0;
  softStopMs = 0;

  lowSpeedApplied = false;
  softCutTriggered = false;

  timeoutRetryActive = false;
  timeoutRetryEndMs = 0;

  motorStartPending = false;
  motorStartDueMs = 0;

  motorFaultStartMs = 0;
  leakStartMs = 0;

  lastWeightMilk_x10  = 0;
  lastWeightSauce_x10 = 0;
  activeScale = SCALE_MILK;

  rinseActive = false; rinseEndMs = 0;

  if (diagTriggerActive) {
    digitalWrite(diagTriggerPin, LOW);
    if (diagRevertToInput) pinMode(diagTriggerPin, INPUT);
  }
  diagTriggerActive = false;
  diagTriggerPin = 255;
  diagTriggerEndMs = 0;
  diagRevertToInput = false;

  leakLastSampleMs = 0;

  faultFlag = false;
  motorFaultLogged = false;
  leakFaultLogged = false;
  scaleFaultLogged = false;
  timeoutFaultLogged = false;

  Serial.println(F("[clear] system state reset; outputs off; speed=LOW (standby, coil OFF); faultFlag=OFF"));
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(PIN_MOTOR_FAULT, INPUT);

  pinMode(PIN_RINSER, OUTPUT);

  pinMode(PIN_SPEEDSEL, OUTPUT);
  
  setSpeedLow();

  for (uint8_t i=0;i<8;i++)  { pinMode(MILK_PINS[i], OUTPUT);  digitalWrite(MILK_PINS[i], LOW); }
  for (uint8_t i=0;i<15;i++) { pinMode(SAUCE_PINS[i], OUTPUT); digitalWrite(SAUCE_PINS[i], LOW); }

  for (uint8_t i=0;i<2;i++) {
    pinMode(LEAK[i].vccPin, OUTPUT);
    digitalWrite(LEAK[i].vccPin, HIGH);
    pinMode(LEAK[i].inPin, INPUT);
  }

  Wire.begin();
  Wire.setClock(400000);

  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_RE_PIN, OUTPUT);
  rs485RxMode();
  MODBUS_SERIAL.begin(MODBUS_BAUD);

  Serial.println(F("Mega One-Shot Dispenser + Leak Monitor + Modbus RTU ready. Type 'help'."));
}

static inline void pumpCore() {
  updateMotorFaultTriggerDebounce();
  sampleLeaksIfDue();
  handleRinseTimer();
  handleDiagTriggerTimer();
  if (state == ST_DISPENSING) handleDispensing();
}

void loop() {
  handleModbus();
  pumpCore();
  rs485RxMode();

  static String line;
  if (!apiBusy) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c=='\r') continue;
      if (c=='\n') {
        line.trim();
        if (line.length()) {
          String cmd = line; line = "";
          String lower = cmd; lower.toLowerCase();

          if (lower == "help") {
            Serial.println(F("Commands:"));
            Serial.println(F("  list"));
            Serial.println(F("  status"));
            Serial.println(F("  clear"));
            Serial.println(F("  abort"));
            Serial.println(F("  dispense <motor> <target_g> <slowPct0-100> <softCut0-100> <timeout_s>"));
            Serial.println(F("  rinse <seconds>"));
            Serial.println(F("  trigger <motor> <seconds> [s|speed]"));
          }
          else if (lower == "list") {
            printList();
          }
          else if (lower == "status") {
            printStatus();
          }
          else if (lower == "clear") {
            clearSystem();
          }
          else if (lower == "abort") {
            abortNow();
          }
          else if (lower.startsWith("rinse ")) {
            String s = cmd.substring(6); s.trim();
            float secs = s.toFloat();
            startRinseSeconds(secs);
          }
          else if (lower.startsWith("trigger ")) {
            String rest = cmd.substring(8); rest.trim();

            int sp1 = rest.indexOf(' ');
            if (sp1 < 0) {
              Serial.println(F("Use: trigger <motor> <seconds> [s|speed]"));
            } else {
              updateMotorFaultTriggerDebounce();
              if (motorFaultDebouncedTrigger) {
                faultFlag = true;
                if (!motorFaultLogged) { logEvent(F("MOTOR_FAULT_ABORT")); motorFaultLogged = true; }
                systemStatus = SYS_MOTOR_FAULT;
                Serial.println(F("[fault] motor fault; trigger blocked"));
                goto _after_cli;
              }
              String smotor = rest.substring(0, sp1); smotor.trim();
              String tail   = rest.substring(sp1+1); tail.trim();

              int sp2 = tail.indexOf(' ');
              String ssec, smod;
              if (sp2 < 0) { ssec = tail; smod = ""; }
              else { ssec = tail.substring(0, sp2); smod = tail.substring(sp2+1); smod.trim(); }

              bool highSpeed = false;
              if (smod.length()) {
                String ml = smod; ml.toLowerCase();
                if (ml == "s" || ml == "speed") highSpeed = true;
                else {
                  Serial.println(F("Use: trigger <motor> <seconds> [s|speed]"));
                  continue;
                }
              }

              int pin; char nm[16];
              if (!parseMotorSelector(smotor, pin, nm, sizeof(nm))) { Serial.println(F("Unknown motor.")); }
              else {
                float secs = ssec.toFloat();
                startDiagTrigger((uint8_t)pin, secs, highSpeed);
              }
            }
          }
          else if (lower.startsWith("dispense ")) {
            if (state == ST_DISPENSING) { Serial.println(F("[busy] already dispensing")); break; }
            if (diagTriggerActive) { Serial.println(F("[blocked] wait for trigger to complete")); break; }

            String rest = cmd.substring(9); rest.trim();
            int sp1 = rest.indexOf(' ');
            int sp2 = rest.indexOf(' ', sp1+1);
            int sp3 = rest.indexOf(' ', sp2+1);
            if (sp1<0 || sp2<0 || sp3<0) { Serial.println(F("Use: dispense <motor> <target_g> <slowPct0-100> <softCut0-100> <timeout_s>")); break; }

            String smotor  = rest.substring(0, sp1); smotor.trim();
            String starget = rest.substring(sp1+1, sp2); starget.trim();
            String sslow   = rest.substring(sp2+1, sp3); sslow.trim();
            String sto     = rest.substring(sp3+1); sto.trim();

            int sp4 = sto.indexOf(' ');
            if (sp4 < 0) { Serial.println(F("Use: dispense <motor> <target_g> <slowPct0-100> <softCut0-100> <timeout_s>")); break; }
            String ssoft   = sto.substring(0, sp4); ssoft.trim();
            String stime   = sto.substring(sp4+1); stime.trim();

            int pin; char nm[16];
            if (!parseMotorSelector(smotor, pin, nm, sizeof(nm))) { Serial.println(F("Unknown motor.")); break; }

            float grams = starget.toFloat();
            if (grams <= 0.0f || grams > 999.0f) {
              Serial.println(F("[error] target_g must be between 0 and 999 g"));
              break;
            }
            uint16_t tx10 = (uint16_t)(grams*10.0f + 0.5f);

            float slowOffset_g = sslow.toFloat();       
            float softCutOffset_g = ssoft.toFloat();    
            unsigned long timeout_s = (unsigned long)stime.toInt();

            startDispense(pin, nm, slowOffset_g, softCutOffset_g, timeout_s, tx10);
          }
          else {
            Serial.println(F("Unknown command. Type 'help'."));
          }
_after_cli: ;
        }
      } else {
        if (line.length() < 120) line += c;
      }
    }
  }
}



static uint16_t mb_crc16(const uint8_t* data, uint16_t len) {
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

static void rs485RxMode() {
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_RE_PIN, LOW);   
}

static void rs485TxMode() {
  digitalWrite(RS485_RE_PIN, HIGH);  
  digitalWrite(RS485_DE_PIN, HIGH);  
  delayMicroseconds(150);            
}


static inline uint16_t regRead(uint16_t addr) {
  if (addr < REG_SPACE_SIZE) return regSpace[addr];
  return 0;
}
static inline void regWrite(uint16_t addr, uint16_t val) {
  if (addr < REG_SPACE_SIZE) regSpace[addr] = val;
}


static uint16_t faultToApiError(uint8_t st) {
  switch (st) {
    case SYS_MOTOR_FAULT:   return AE_MOTOR;
    case SYS_LEAK_FAULT:    return AE_LEAK;
    case SYS_SCALE_FAULT:   return AE_SCALE;
    case SYS_TIMEOUT_FAULT: return AE_TIMEOUT;
    case SYS_ABORTED_FAULT: return AE_ABORTED;
    default: return AE_NONE;
  }
}


static void buildResult(uint16_t rc, uint16_t seq, uint16_t err) {
  regWrite(0x0100, rc);
  regWrite(0x0101, err);
  regWrite(0x0102, (uint16_t)systemStatus);
  regWrite(0x0103, seq);
  regWrite(0x0104, lastWeight_x10);
  regWrite(0x0106, activeScaleId());
  regWrite(0x0107, 0);
  regWrite(0x0108, 0);
  regWrite(0x0109, 0);
  regWrite(0x010A, 0);
}




static bool motorIdToPinName(uint16_t motorId, int &pin, const char* &nm) {
  if (motorId >= 11 && motorId <= 18) {
    uint8_t idx = (uint8_t)(motorId - 11); 
    pin = MILK_PINS[idx];
    nm  = MILK_NAMES[idx];
    return true;
  }
  if (motorId >= 21 && motorId <= 35) {
    uint8_t idx = (uint8_t)(motorId - 21); 
    pin = SAUCE_PINS[idx];
    nm  = SAUCE_NAMES[idx];
    return true;
  }
  return false;
}


static void executeOpcode(uint16_t opcode, uint16_t seq) {
  updateMotorFaultTriggerDebounce();

  bool hasFault =
      (systemStatus == SYS_MOTOR_FAULT)   ||
      (systemStatus == SYS_LEAK_FAULT)    ||
      (systemStatus == SYS_SCALE_FAULT)   ||
      (systemStatus == SYS_TIMEOUT_FAULT) ||
      (systemStatus == SYS_ABORTED_FAULT);

  if (hasFault && !(opcode == OP_CLEAR || opcode == OP_ABORT || opcode == OP_STATUS)) {
    buildResult(RC_FAIL, seq, faultToApiError(systemStatus));
    return;
  }

  switch (opcode) {
    case OP_STATUS: {
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_CLEAR: {
      clearSystem();
      buildResult(RC_OK, seq, AE_NONE);
    } break;

    case OP_ABORT: {
      unsigned long t0 = millis();
      abortNow();
      buildResult(RC_FAIL, seq, AE_ABORTED);
      uint16_t elap_x10 = (uint16_t)(((millis()-t0)+50)/100);
      regWrite(0x0105, elap_x10);
    } break;

    case OP_RINSE: {
      if (state == ST_DISPENSING || diagTriggerActive) {
        buildResult(RC_FAIL, seq, AE_BUSY);
        break;
      }
      uint16_t seconds_x10 = regRead(0x0002);
      float secs = seconds_x10 / 10.0f;
      startRinseSeconds(secs);
      if (!rinseActive) {
        buildResult(RC_FAIL, seq, AE_BUSY);
      } else {
        buildResult(RC_OK, seq, AE_NONE); 
      }
    } break;

    case OP_TRIGGER: {
      if (state == ST_DISPENSING || rinseActive) {
        buildResult(RC_FAIL, seq, AE_BUSY);
        break;
      }
      if (motorFaultDebouncedTrigger) {
        faultFlag = true;
        if (!motorFaultLogged) { logEvent(F("MOTOR_FAULT_ABORT")); motorFaultLogged = true; }
        systemStatus = SYS_MOTOR_FAULT;
        buildResult(RC_FAIL, seq, AE_MOTOR);
        break;
      }
      uint16_t motorId = regRead(0x0002);
      uint16_t seconds_x10 = regRead(0x0003);
      uint16_t flags = regRead(0x0004);
      regWrite(0x0004, 0);
      bool highSpeed = (flags & 0x0001) != 0;

      int pin; const char* nm;
      if (!motorIdToPinName(motorId, pin, nm)) {
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }
      float secs = seconds_x10 / 10.0f;
      startDiagTrigger((uint8_t)pin, secs, highSpeed);
      if (!diagTriggerActive) {
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
      } else {
        buildResult(RC_OK, seq, AE_NONE); 
      }
    } break;

    case OP_DISPENSE: {
      if (state == ST_DISPENSING || diagTriggerActive) {
        buildResult(RC_FAIL, seq, AE_BUSY);
        break;
      }
      uint16_t motorId      = regRead(0x0002);
      uint16_t tgt_x10      = regRead(0x0003);
      uint16_t slowOff_x10  = regRead(0x0004);
      uint16_t softOff_x10  = regRead(0x0005);
      uint16_t timeout_s    = regRead(0x0006);

      
      if (tgt_x10 == 0 || tgt_x10 > 9990) {
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }

      int pin; const char* nm;
      if (!motorIdToPinName(motorId, pin, nm)) {
        buildResult(RC_FAIL, seq, AE_BAD_ARGS);
        break;
      }

      float slowParam = slowOff_x10 / 10.0f;  
      float softParam = softOff_x10 / 10.0f;  

      startDispense(pin, nm, slowParam, softParam, timeout_s, tgt_x10);

      if (state != ST_DISPENSING && systemStatus == SYS_SCALE_FAULT) {
        buildResult(RC_FAIL, seq, AE_SCALE);
      } else if (state == ST_DISPENSING || systemStatus == SYS_ACTIVE) {
        buildResult(RC_OK, seq, AE_NONE); 
      } else {
        uint16_t err = faultToApiError(systemStatus);
        if (err == AE_NONE) err = AE_INVALID_CMD;
        buildResult(RC_FAIL, seq, err);
      }
    } break;

    default:
      buildResult(RC_FAIL, seq, AE_INVALID_CMD);
      break;
  }
}


static void handleModbus() {
  if (apiBusy) return;

  static uint8_t  rxBuf[260];
  static uint16_t rxLen = 0;
  static unsigned long lastByteUs = 0;

  
  while (MODBUS_SERIAL.available()) {
    int b = MODBUS_SERIAL.read();
    if (b < 0) break;
    if (rxLen < sizeof(rxBuf)) rxBuf[rxLen++] = (uint8_t)b;
    lastByteUs = micros();
  }

  
  if (rxLen > 0) {
    unsigned long gap = micros() - lastByteUs;
    if (gap >= MB_T3P5_US) {
      
      if (rxLen >= 5) {
        
        uint16_t crcCalc = mb_crc16(rxBuf, (uint16_t)(rxLen - 2));
        uint16_t crcRx   = (uint16_t)(rxBuf[rxLen - 2] | ((uint16_t)rxBuf[rxLen - 1] << 8));
        if (crcCalc == crcRx) {
          uint8_t addr = rxBuf[0];
          uint8_t func = rxBuf[1];

          if (addr == MODBUS_SLAVE_ID) {
            if (func != 0x17) {
              
              uint8_t resp[5];
              resp[0] = addr; resp[1] = (uint8_t)(func | 0x80); resp[2] = 0x01;
              uint16_t ecrc = mb_crc16(resp, 3);
              resp[3] = (uint8_t)(ecrc & 0xFF); resp[4] = (uint8_t)(ecrc >> 8);
              rs485TxMode(); MODBUS_SERIAL.write(resp, 5); MODBUS_SERIAL.flush(); delayMicroseconds(MB_T3P5_US); rs485RxMode();
            } else {
              
              if (rxLen >= 13) {
                uint16_t readStart  = (uint16_t)((rxBuf[2] << 8) | rxBuf[3]);
                uint16_t readQty    = (uint16_t)((rxBuf[4] << 8) | rxBuf[5]);
                uint16_t writeStart = (uint16_t)((rxBuf[6] << 8) | rxBuf[7]);
                uint16_t writeQty   = (uint16_t)((rxBuf[8] << 8) | rxBuf[9]);
                uint8_t  writeBytes = rxBuf[10];

                
                if (writeBytes != (uint8_t)(writeQty * 2)) {
                  uint8_t ex[5];
                  ex[0] = addr; ex[1] = (uint8_t)(func | 0x80); ex[2] = 0x03; 
                  uint16_t ecrc = mb_crc16(ex, 3);
                  ex[3] = (uint8_t)(ecrc & 0xFF); ex[4] = (uint8_t)(ecrc >> 8);
                  rs485TxMode(); MODBUS_SERIAL.write(ex, 5); MODBUS_SERIAL.flush(); delayMicroseconds(MB_T3P5_US); rs485RxMode();
                } else {
                  uint16_t expectedLen = (uint16_t)(13 + writeBytes);
                  if (rxLen == expectedLen) {
                    
                    const uint8_t* writeData = rxBuf + 11;
                    for (uint16_t i = 0; i < writeQty; i++) {
                      uint16_t val = (uint16_t)((writeData[2 * i] << 8) | writeData[2 * i + 1]);
                      regWrite((uint16_t)(writeStart + i), val);
                    }

                    uint16_t opcode = regRead(0x0000);
                    uint16_t seq    = regRead(0x0001);

                    bool busyNow = (state == ST_DISPENSING) || rinseActive || diagTriggerActive;

                    if (busyNow) {
                      if (opcode == OP_ABORT || opcode == OP_STATUS) {
                        apiBusy = true; executeOpcode(opcode, seq); apiBusy = false;
                      } else {
                        buildResult(RC_FAIL, seq, AE_BUSY);
                      }
                    } else {
                      apiBusy = true; executeOpcode(opcode, seq); apiBusy = false;
                    }

                    
                    if (readQty > MB_MAX_READ_QTY) {
                      uint8_t ex[5];
                      ex[0] = addr; ex[1] = (uint8_t)(func | 0x80); ex[2] = 0x03;
                      uint16_t ecrc = mb_crc16(ex, 3);
                      ex[3] = (uint8_t)(ecrc & 0xFF); ex[4] = (uint8_t)(ecrc >> 8);
                      rs485TxMode(); MODBUS_SERIAL.write(ex, 5); MODBUS_SERIAL.flush(); delayMicroseconds(MB_T3P5_US); rs485RxMode();
                    } else {
                      const uint16_t respBytes = (uint16_t)(readQty * 2);
                      uint16_t outLen = (uint16_t)(3 + respBytes + 2);
                      uint8_t* resp = mb_resp_buf;

                      resp[0] = addr;
                      resp[1] = func;
                      resp[2] = (uint8_t)respBytes;

                      for (uint16_t i = 0; i < readQty; i++) {
                        uint16_t v = regRead((uint16_t)(readStart + i));
                        resp[3 + 2 * i]     = (uint8_t)(v >> 8);
                        resp[3 + 2 * i + 1] = (uint8_t)(v & 0xFF);
                      }

                      uint16_t rcrc = mb_crc16(resp, (uint16_t)(outLen - 2));
                      resp[outLen - 2] = (uint8_t)(rcrc & 0xFF);
                      resp[outLen - 1] = (uint8_t)(rcrc >> 8);

                      rs485TxMode();
                      MODBUS_SERIAL.write(resp, outLen);
                      MODBUS_SERIAL.flush();
                      delayMicroseconds(MB_T3P5_US); 
                      rs485RxMode();
                    }
                  }
                  
                }
              }
              
            }
          }
          
        }
        
      }
      
      rxLen = 0;
    }
  }
}
