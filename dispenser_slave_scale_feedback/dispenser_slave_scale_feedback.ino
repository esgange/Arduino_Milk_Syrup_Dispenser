

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>


const byte PIN_DOUT   = A2;
const byte PIN_SCK    = A3;
const byte PIN_BTN    = A1;
const byte PIN_BUZZER = A0;


const byte DIGIT_PINS[3] = {2, 3, 4};
const byte SEG_PINS[8]   = {5, 6, 7, 8, 9, 10, 11, 12};


const byte PIN_LED_GREEN = 0;
const byte PIN_LED_RED   = 1;


const uint8_t I2C_ADDR = 0x2B;


const uint16_t SCAN_INTERVAL_US = 2000;
const uint32_t BTN_DEBOUNCE_MS  = 50;
const uint32_t BTN_LONG_MS      = 3000;
const uint32_t INACTIVITY_MS    = 60000;
const float    MAJOR_CHANGE_G   = 1.0f;
const uint32_t BUZZER_ON_MS     = 50;   


float g_scale       = 420.0f;
const float DEAD_BAND_G = 0.10f;
long  g_tareOffset  = 0;
float CAL_WEIGHT_G  = 500.0f;


const uint8_t FILTER_WINDOW = 6;
float   g_filterAccum = 0.0f;
uint8_t g_filterCount = 0;


volatile bool     g_hxReadyFlag   = false;
volatile float    g_units_shared  = 0.0f;
volatile uint16_t g_weight_x10    = 0;
bool              g_units_init    = false;


constexpr uint8_t SEG_A = 1 << 0;
constexpr uint8_t SEG_B = 1 << 1;
constexpr uint8_t SEG_C = 1 << 2;
constexpr uint8_t SEG_D = 1 << 3;
constexpr uint8_t SEG_E = 1 << 4;
constexpr uint8_t SEG_F = 1 << 5;
constexpr uint8_t SEG_G = 1 << 6;

const uint8_t DIGIT_GLYPH[10] = {
  (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F),
  (SEG_B|SEG_C),
  (SEG_A|SEG_B|SEG_D|SEG_E|SEG_G),
  (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G),
  (SEG_F|SEG_G|SEG_B|SEG_C),
  (SEG_A|SEG_F|SEG_G|SEG_C|SEG_D),
  (SEG_A|SEG_F|SEG_E|SEG_D|SEG_C|SEG_G),
  (SEG_A|SEG_B|SEG_C),
  (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),
  (SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G)
};

const uint8_t GLYPH_BLANK = 0x00;
const uint8_t GLYPH_C     = (SEG_A|SEG_F|SEG_E|SEG_D);
const uint8_t GLYPH_A     = (SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G);
const uint8_t GLYPH_L     = (SEG_F|SEG_E|SEG_D);
const uint8_t GLYPH_S     = (SEG_A|SEG_F|SEG_G|SEG_C|SEG_D);
const uint8_t GLYPH_E     = (SEG_A|SEG_F|SEG_G|SEG_E|SEG_D);
const uint8_t GLYPH_t     = (SEG_F|SEG_E|SEG_D|SEG_G);
const uint8_t GLYPH_H     = (SEG_F|SEG_E|SEG_B|SEG_C|SEG_G);
const uint8_t GLYPH_I     = (SEG_B|SEG_C);

volatile uint8_t g_glyph[3] = {0, 0, 0};
volatile bool    g_dp[3]    = {false, false, false};
uint8_t          g_scanIdx  = 0;
uint32_t         g_lastScan = 0;
bool             g_displayEnabled = false;
volatile bool    g_overflowDisplay = false;


enum UIMode : uint8_t { MODE_IDLE = 0, MODE_READING = 1, MODE_CAL_WAIT_WEIGHT = 2 };
UIMode g_mode = MODE_IDLE;

float    g_lastSignificant = 0.0f;
uint32_t g_lastChangeMs    = 0;


bool     btn_lastReading  = HIGH;
bool     btn_stable       = HIGH;
uint32_t btn_lastDebounce = 0;
bool     btn_pressed      = false;
uint32_t btn_pressStart   = 0;
bool     btn_longFired    = false;


volatile bool     g_targetActive       = false;
volatile uint16_t g_targetX10          = 0;
volatile bool     g_targetHitLatched   = false;
volatile bool     g_i2cCmdPending      = false;
volatile uint16_t g_i2cPendingTargetX10= 0;


uint32_t g_lastPrintMs   = 0;
uint32_t g_buzzerUntilMs = 0;


const int      EEPROM_ADDR        = 0;
const uint32_t EEPROM_MAGIC_SCALE = 0x53434C45UL;

uint16_t sum16_bytes(const uint8_t* p, size_t n) {
  uint32_t s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return (uint16_t)(s & 0xFFFF);
}

void eepromSaveScaleOnly() {
  struct {
    uint32_t magic;
    float    scale;
    uint16_t sum;
  } pd;
  pd.magic = EEPROM_MAGIC_SCALE;
  pd.scale = g_scale;
  pd.sum   = sum16_bytes(reinterpret_cast<const uint8_t*>(&pd.scale), sizeof(pd.scale));
  EEPROM.put(EEPROM_ADDR, pd);
  Serial.println(F("EEPROM: saved scale."));
}

bool eepromLoadScaleOnly() {
  struct {
    uint32_t magic;
    float    scale;
    uint16_t sum;
  } pd;
  EEPROM.get(EEPROM_ADDR, pd);
  uint16_t calc = sum16_bytes(reinterpret_cast<const uint8_t*>(&pd.scale), sizeof(pd.scale));
  if (pd.magic == EEPROM_MAGIC_SCALE && pd.sum == calc && isfinite(pd.scale) && pd.scale != 0.0f) {
    g_scale = (pd.scale > 0) ? pd.scale : -pd.scale;
    Serial.println(F("EEPROM: loaded scale."));
    return true;
  }
  return false;
}


inline void setGreenLED(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
inline void setRedLED(bool on)   { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }



static const bool HX_INVERT_SIGN = true;

inline long hx_read_now_assuming_ready() {
  unsigned long v = 0;
  for (byte i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_SCK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1);
  digitalWrite(PIN_SCK, LOW);  delayMicroseconds(1);
  if (v & 0x800000UL) v |= 0xFF000000UL;
  long s = (long)v;
  return HX_INVERT_SIGN ? -s : s;
}

bool hx_wait_ready_timeout(uint32_t timeout_ms = 1000) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DOUT)) {
    if (millis() - t0 > timeout_ms) return false;
  }
  return true;
}

long hx_read_blocking() {
  if (!hx_wait_ready_timeout()) return 0;
  return hx_read_now_assuming_ready();
}

long hx_read_average_withScan(uint16_t N) {
  long long sum = 0;
  for (uint16_t i = 0; i < N; i++) {
    while (digitalRead(PIN_DOUT)) {
      uint32_t now = micros();
      if ((uint32_t)(now - g_lastScan) >= SCAN_INTERVAL_US) {
        g_lastScan = now;
        digitalWrite(DIGIT_PINS[0], LOW);
        digitalWrite(DIGIT_PINS[1], LOW);
        digitalWrite(DIGIT_PINS[2], LOW);
        if (g_displayEnabled) {
          g_scanIdx = (g_scanIdx + 1) % 3;
          for (uint8_t s = 0; s < 7; s++)
            digitalWrite(SEG_PINS[s], (g_glyph[g_scanIdx] & (1 << s)) ? HIGH : LOW);
          digitalWrite(SEG_PINS[7], g_dp[g_scanIdx] ? HIGH : LOW);
          digitalWrite(DIGIT_PINS[g_scanIdx], HIGH);
        }
      }
    }
    sum += hx_read_now_assuming_ready();
  }
  return (long)(sum / (long long)N);
}


void hx_dataReadyISR() { g_hxReadyFlag = true; }


inline void display_allDigitsOff() {
  digitalWrite(DIGIT_PINS[0], LOW);
  digitalWrite(DIGIT_PINS[1], LOW);
  digitalWrite(DIGIT_PINS[2], LOW);
}

inline void display_writeSegments(uint8_t glyph, bool dpOn) {
  for (uint8_t i = 0; i < 7; i++)
    digitalWrite(SEG_PINS[i], (glyph & (1 << i)) ? HIGH : LOW);
  digitalWrite(SEG_PINS[7], dpOn ? HIGH : LOW);
}

void display_scanStep() {
  uint32_t now = micros();
  if ((uint32_t)(now - g_lastScan) < SCAN_INTERVAL_US) return;
  g_lastScan = now;
  display_allDigitsOff();
  if (!g_displayEnabled) return;
  g_scanIdx = (g_scanIdx + 1) % 3;
  const uint8_t glyph = g_glyph[g_scanIdx];
  const bool    dp    = g_dp[g_scanIdx];
  display_writeSegments(glyph, dp);
  digitalWrite(DIGIT_PINS[g_scanIdx], HIGH);
}


void display_showCAL() {
  noInterrupts();
  g_glyph[0] = GLYPH_C; g_glyph[1] = GLYPH_A; g_glyph[2] = GLYPH_L;
  g_dp[0] = g_dp[1] = g_dp[2] = false;
  interrupts();
}

void display_showSET() {
  noInterrupts();
  g_glyph[0] = GLYPH_S; g_glyph[1] = GLYPH_E; g_glyph[2] = GLYPH_t;
  g_dp[0] = g_dp[1] = g_dp[2] = false;
  interrupts();
}

void display_showHI() {
  noInterrupts();
  g_glyph[0] = GLYPH_H; g_glyph[1] = GLYPH_I; g_glyph[2] = GLYPH_BLANK;
  g_dp[0] = g_dp[1] = g_dp[2] = false;
  interrupts();
}

void display_formatFromValue(float av) {
  noInterrupts();
  if (av >= 100.0f) {
    long iv = (long)(av + 0.5f);
    if (iv > 999) iv = 999;
    uint8_t d0 = (iv / 100) % 10;
    uint8_t d1 = (iv / 10)  % 10;
    uint8_t d2 =  iv % 10;
    g_glyph[0] = DIGIT_GLYPH[d0];
    g_glyph[1] = DIGIT_GLYPH[d1];
    g_glyph[2] = DIGIT_GLYPH[d2];
    g_dp[0] = g_dp[1] = g_dp[2] = false;
  } else {
    long scaled = (long)(av * 10.0f + 0.5f);
    if (scaled >= 1000) {
      long iv = 100;
      uint8_t d0 = (iv / 100) % 10;
      uint8_t d1 = (iv / 10)  % 10;
      uint8_t d2 =  iv % 10;
      g_glyph[0] = DIGIT_GLYPH[d0];
      g_glyph[1] = DIGIT_GLYPH[d1];
      g_glyph[2] = DIGIT_GLYPH[d2];
      g_dp[0] = g_dp[1] = g_dp[2] = false;
    } else {
      uint8_t d2 = scaled % 10;
      uint8_t d1 = (scaled / 10) % 10;
      uint8_t d0 = (scaled / 100) % 10;
      g_glyph[0] = (d0 == 0) ? GLYPH_BLANK : DIGIT_GLYPH[d0];
      g_glyph[1] = DIGIT_GLYPH[d1];
      g_glyph[2] = DIGIT_GLYPH[d2];
      g_dp[0] = false; g_dp[1] = true; g_dp[2] = false;
    }
  }
  interrupts();
}


void enterIdle() {
  g_mode = MODE_IDLE;
  g_displayEnabled = false;
  setGreenLED(false);
}

void enterReading(float currentVal) {
  g_mode = MODE_READING;
  g_displayEnabled = true;
  setGreenLED(true);
  g_lastSignificant = currentVal;
  g_lastChangeMs = millis();
}

void enterCal() {
  g_mode = MODE_CAL_WAIT_WEIGHT;
  g_displayEnabled = true;
  setGreenLED(false);
  display_showCAL();
}


void armTarget(uint16_t target_x10) {
  if (g_mode == MODE_CAL_WAIT_WEIGHT) return;
  if (target_x10 > 9990) target_x10 = 9990;

  noInterrupts();
  g_targetX10 = target_x10;
  g_targetActive = true;
  interrupts();

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);

  uint16_t w_x10 = (uint16_t)min(9990L, (long)(val * 10.0f + 0.5f));
  if (w_x10 >= target_x10) {
    noInterrupts(); g_targetActive = false; g_targetHitLatched = true; interrupts();
    setRedLED(true);
  }
}

void checkTargetHitOnUpdate(float val) {
  bool active; uint16_t t; noInterrupts(); active = g_targetActive; t = g_targetX10; interrupts();
  if (!active) return;
  uint16_t w_x10 = (uint16_t)min(9990L, (long)(val * 10.0f + 0.5f));
  if (w_x10 >= t) {
    noInterrupts(); g_targetActive = false; g_targetHitLatched = true; interrupts();
    setRedLED(true);
  }
}


void doShortPressTare() {
  g_displayEnabled = true;
  display_showSET();

  detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
  long t = hx_read_average_withScan(20);
  g_tareOffset = t;
  g_hxReadyFlag = false;
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  g_filterAccum = 0.0f;
  g_filterCount = 0;
  g_buzzerUntilMs = millis() + BUZZER_ON_MS;

  Serial.println(F("Tare done (button)."));
  setRedLED(false);
  noInterrupts(); g_targetActive = false; interrupts();
  g_overflowDisplay = false;

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);
}


void doStartupTare() {
  g_displayEnabled = true;
  display_showSET();

  long t = hx_read_average_withScan(20);
  g_tareOffset = t;
  Serial.println(F("Tare done (startup)."));

  setRedLED(false);
  g_overflowDisplay = false;

  g_filterAccum = 0.0f;
  g_filterCount = 0;
  g_buzzerUntilMs = millis() + BUZZER_ON_MS;

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);
}


void queueI2CArm(uint16_t tx10) {
  noInterrupts();
  g_i2cPendingTargetX10 = (tx10 > 9990) ? 9990 : tx10;
  g_i2cCmdPending = true;
  interrupts();
}


void captureCalibrationIfInCAL();

void pollButton() {
  bool reading = digitalRead(PIN_BTN);
  if (reading != btn_lastReading) { btn_lastDebounce = millis(); btn_lastReading = reading; }

  if (millis() - btn_lastDebounce > BTN_DEBOUNCE_MS) {
    if (reading != btn_stable) {
      btn_stable = reading;
      if (btn_stable == LOW) {
        btn_pressed = true; btn_pressStart = millis(); btn_longFired = false;
      } else {
        uint32_t dur = millis() - btn_pressStart; btn_pressed = false;
        if (!btn_longFired && dur >= BTN_DEBOUNCE_MS) {
          if (g_mode == MODE_CAL_WAIT_WEIGHT) {
            captureCalibrationIfInCAL();
          } else {
            doShortPressTare();
          }
        }
      }
    }

    if (btn_pressed && !btn_longFired) {
      uint32_t held = millis() - btn_pressStart;
      if (held >= BTN_LONG_MS) {
        btn_longFired = true;

        Serial.println(F("Entering CAL mode..."));
        enterCal();

        detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
        long t = hx_read_average_withScan(20);
        g_tareOffset = t;
        setRedLED(false);
        attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

        g_filterAccum = 0.0f;
        g_filterCount = 0;
        g_buzzerUntilMs = millis() + BUZZER_ON_MS;

        Serial.println(F("Place known weight, then short-press to confirm."));
      }
    }
  }
}


void onI2CReceive(int numBytes) {
  if (numBytes < 1) return;
  uint8_t cmd = Wire.read(); numBytes--;
  if (cmd == 'R' && numBytes >= 2) {
    uint8_t lo = Wire.read(), hi = Wire.read();
    uint16_t tx10 = (uint16_t)(lo | (uint16_t(hi) << 8));
    queueI2CArm(tx10);
  } else {
    while (numBytes-- > 0) { (void)Wire.read(); }
  }
}

void onI2CRequest() {
  uint8_t  hit = 0;
  uint16_t w   = 0;

  noInterrupts();
  if (g_targetHitLatched) { hit = 1; g_targetHitLatched = false; }
  w = g_weight_x10;
  interrupts();

  uint8_t buf[3] = { hit, (uint8_t)(w & 0xFF), (uint8_t)((w >> 8) & 0xFF) };
  Wire.write(buf, 3);
}


void captureCalibrationIfInCAL() {
  if (g_mode != MODE_CAL_WAIT_WEIGHT) return;

  Serial.println(F("Capturing calibration weight..."));
  long avg   = hx_read_average_withScan(32);
  long delta = avg - g_tareOffset;

  if (delta != 0) {
    g_scale = (float)delta / CAL_WEIGHT_G;
    Serial.print(F("New scale factor: "));
    Serial.println(g_scale, 6);
    eepromSaveScaleOnly();
  } else {
    Serial.println(F("Calibration failed: delta=0"));
  }

  g_filterAccum = 0.0f;
  g_filterCount = 0;

  g_hxReadyFlag = false;
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);
}


void setup() {
  pinMode(PIN_SCK,  OUTPUT);
  pinMode(PIN_DOUT, INPUT);
  pinMode(PIN_BTN,  INPUT_PULLUP);
  digitalWrite(PIN_SCK, LOW);

  for (uint8_t i = 0; i < 8; i++) { pinMode(SEG_PINS[i], OUTPUT); digitalWrite(SEG_PINS[i], LOW); }
  for (uint8_t i = 0; i < 3; i++) { pinMode(DIGIT_PINS[i], OUTPUT); digitalWrite(DIGIT_PINS[i], LOW); }

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  setGreenLED(false);
  setRedLED(false);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  if (eepromLoadScaleOnly()) Serial.println(F("EEPROM OK. Will TARE at startup."));
  else                       Serial.println(F("EEPROM invalid/empty. Will TARE at startup."));

  doStartupTare();
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  Serial.println(F("Ready. In READING after startup TARE. I2C 'R'+target_x10 will TARE then arm target."));
}

void loop() {
  display_scanStep();
  pollButton();

  if (g_i2cCmdPending && g_mode != MODE_CAL_WAIT_WEIGHT) {
    noInterrupts(); g_i2cCmdPending = false; uint16_t tx10 = g_i2cPendingTargetX10; interrupts();

    g_displayEnabled = true;
    display_showSET();

    detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
    long t = hx_read_average_withScan(20);
    g_tareOffset = t;
    g_hxReadyFlag = false;
    attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

    g_filterAccum = 0.0f;
    g_filterCount = 0;
    g_buzzerUntilMs = millis() + BUZZER_ON_MS;

    Serial.println(F("Tare done (I2C 'R')."));
    setRedLED(false);
    g_overflowDisplay = false;

    float val; noInterrupts(); val = g_units_shared; interrupts();
    enterReading(val);
    armTarget(tx10);
  }

  if (g_mode == MODE_CAL_WAIT_WEIGHT) {
  } else {
    if (g_hxReadyFlag) {
      g_hxReadyFlag = false;
      if (!digitalRead(PIN_DOUT)) {
        long  raw       = hx_read_now_assuming_ready();
        float candidate = (raw - g_tareOffset) / g_scale;

        g_filterAccum += candidate;
        g_filterCount++;

        if (g_filterCount >= FILTER_WINDOW) {
          float av = g_filterAccum / g_filterCount;
          g_filterAccum = 0.0f;
          g_filterCount = 0;

          bool overflowNow = (av > 999.0f);

          if (av < 0.0f)   av = 0.0f;
          if (av > 999.0f) av = 999.0f;

          if (!g_units_init) {
            noInterrupts();
            g_units_shared = av;
            g_weight_x10   = (uint16_t)min(9990L, (long)(av * 10.0f + 0.5f));
            g_overflowDisplay = overflowNow;
            interrupts();
            g_units_init = true;

            if (g_mode == MODE_READING && fabsf(av - g_lastSignificant) >= MAJOR_CHANGE_G) {
              g_lastSignificant = av;
              g_lastChangeMs = millis();
            } else if (g_mode == MODE_READING) {
              g_lastChangeMs = millis();
            }
          } else {
            float prev; noInterrupts(); prev = g_units_shared; interrupts();
            if (fabsf(av - prev) >= DEAD_BAND_G || overflowNow != g_overflowDisplay) {
              noInterrupts();
              g_units_shared = av;
              g_weight_x10   = (uint16_t)min(9990L, (long)(av * 10.0f + 0.5f));
              g_overflowDisplay = overflowNow;
              interrupts();

              if (g_mode == MODE_READING && fabsf(av - g_lastSignificant) >= MAJOR_CHANGE_G) {
                g_lastSignificant = av;
                g_lastChangeMs = millis();
              }
            }
          }
        }
      }
    }

    float val; noInterrupts(); val = g_units_shared; interrupts();
    checkTargetHitOnUpdate(val);

    if (g_mode == MODE_READING) {
      bool of; noInterrupts(); of = g_overflowDisplay; interrupts();
      if (of) {
        display_showHI();
      } else {
        float av = val;
        noInterrupts();
        if (av >= 100.0f) {
          long iv = (long)(av + 0.5f);
          if (iv > 999) iv = 999;
          uint8_t d0 = (iv / 100) % 10, d1 = (iv / 10) % 10, d2 = iv % 10;
          g_glyph[0] = DIGIT_GLYPH[d0]; g_glyph[1] = DIGIT_GLYPH[d1]; g_glyph[2] = DIGIT_GLYPH[d2];
          g_dp[0] = g_dp[1] = g_dp[2] = false;
        } else {
          long scaled = (long)(av * 10.0f + 0.5f);
          if (scaled >= 1000) {
            long iv = 100;
            uint8_t d0 = (iv / 100) % 10, d1 = (iv / 10) % 10, d2 = iv % 10;
            g_glyph[0] = DIGIT_GLYPH[d0]; g_glyph[1] = DIGIT_GLYPH[d1]; g_glyph[2] = DIGIT_GLYPH[d2];
            g_dp[0] = g_dp[1] = g_dp[2] = false;
          } else {
            uint8_t d2 =  scaled % 10;
            uint8_t d1 = (scaled / 10) % 10;
            uint8_t d0 = (scaled / 100) % 10;
            g_glyph[0] = (d0 == 0) ? GLYPH_BLANK : DIGIT_GLYPH[d0];
            g_glyph[1] = DIGIT_GLYPH[d1];
            g_glyph[2] = DIGIT_GLYPH[d2];
            g_dp[0] = false; g_dp[1] = true; g_dp[2] = false;
          }
        }
        interrupts();
      }

      if (millis() - g_lastChangeMs >= INACTIVITY_MS) enterIdle();
    } else if (g_mode == MODE_IDLE) {
    }

    uint32_t now = millis();
    if (now - g_lastPrintMs >= 200) {
      g_lastPrintMs = now;
      Serial.print(F("mode="));          Serial.print((int)g_mode);
      Serial.print(F(" val="));          Serial.print(val, 3);
      Serial.print(F(" overflow="));     Serial.print((int)g_overflowDisplay);
      Serial.print(F(" targetActive=")); Serial.print((int)g_targetActive);
      Serial.print(F(" targetX10="));    Serial.print((unsigned)g_targetX10);
      Serial.print(F(" hitLatched="));   Serial.println((int)g_targetHitLatched);
    }

    if (g_buzzerUntilMs && (long)(now - g_buzzerUntilMs) < 0) {
      digitalWrite(PIN_BUZZER, HIGH);
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }
  }
}
