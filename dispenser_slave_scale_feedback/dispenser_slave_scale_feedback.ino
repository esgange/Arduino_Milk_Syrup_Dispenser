/*
  ┌──────────────────────────────────────────────────────────────────────────┐
  │ Scale Slave Firmware — Arduino Nano Every + HX711 + 3-digit 7-seg + I2C  │
  └──────────────────────────────────────────────────────────────────────────┘

  Version : 1.0
  Date    : 2025-10-10  (Asia/Riyadh)
  Authors : Erdie Gange · ChatGPT 5

  What this firmware does
  ───────────────────────
  • Modes
    - IDLE:     display blank; Green LED OFF.
    - READING:  live weight (0…999 g); <100 → XX.X, ≥100 → XXX; Green LED ON.
    - CAL:      shows “CAL”; place known weight; short press computes & saves scale.

  • Tare (“SEt” shown while sampling 16 points)
    - Auto-tare at startup (shows “SEt”).
    - Physical short press (>50 ms, <3 s).
    - I2C ‘R’ (master arms read-with-target; this also tares first, shows “SEt”).
    - Long press (≥3 s) enters CAL immediately with a single silent tare at entry
      (no “SEt” during this CAL-entry tare).

  • Target & LEDs
    - Master writes ‘R’ + target_x10 (LE) → slave tares (shows “SEt”), then arms target.
    - When weight ≥ target → Red LED ON (latched), one-shot “hit” flag for next I2C read.
    - Red LED clears on any tare (startup, short press, or I2C ‘R’).

  • I2C
    - Address: 0x2B (change I2C_ADDR if needed).
    - Write: ‘R’ + uint16 target_x10 (grams×10, little-endian).
    - Read : 3 bytes [ hit_once(0/1), weight_x10_lo, weight_x10_hi ].

  • Overflow
    - If actual weight > 999 g, shows “HI” on the 7-segment; I2C clamps at 999.0 max.

  • Auto-IDLE
    - After 60 s without ≥1 g change while in READING → blank display, Green OFF.

  • Persistence
    - Scale factor is saved to EEPROM only after completing CAL.
    - Tare (offset) is NOT persisted.

  Calibration instructions
  ────────────────────────
    1) Power on: auto-tare runs (“SEt”); wait for ~0.0 g.
    2) Long-press ≥3 s to enter CAL (shows “CAL”); hands-off scale platform for accuracy.
    3) Place known weight (500.0 g default) and let it stabilize.
    4) Short-press to capture (32 samples), compute & save scale to EEPROM; exits to READING.
    5) Verify display ≈ weight, then remove weight. DONE

  Hardware
  ────────
  - HX711: DOUT=A2, SCK=A3
  - Button: A1 (active-LOW, INPUT_PULLUP)
  - 7-segment (common anode or cathode via ACTIVE HIGH enables per digit):
      DIGIT_PINS: 2,3,4 (ACTIVE HIGH)
      SEG_PINS:   5..12 = a,b,c,d,e,f,g,dp (ACTIVE HIGH)
  - LEDs on UART pins:
      RX (0) = Green, TX (1) = Red
*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

/* ────────────────────────────────── PINS ────────────────────────────────── */
const byte PIN_DOUT = A2;     // HX711 DOUT (data ready LOW)
const byte PIN_SCK  = A3;     // HX711 SCK
const byte PIN_BTN  = A1;     // Button (active LOW, INPUT_PULLUP)

// 7-segment
const byte DIGIT_PINS[3] = {2, 3, 4};                   // ACTIVE HIGH enables
const byte SEG_PINS[8]   = {5, 6, 7, 8, 9, 10, 11, 12}; // a,b,c,d,e,f,g,dp (ACTIVE HIGH)

// LEDs on UART pins
const byte PIN_LED_GREEN = 0; // RX pin -> Green LED
const byte PIN_LED_RED   = 1; // TX pin -> Red LED

/* ─────────────────────────────────── I2C ─────────────────────────────────── */
const uint8_t I2C_ADDR = 0x2A;  // Slave address //////////////////////////////// *We use 0x2B for Sauces, 0x2A for Milks. <----------

/* ───────────────────────────────── TIMING ────────────────────────────────── */
const uint16_t SCAN_INTERVAL_US = 2000;   // Per-digit scan; smaller = brighter
const uint32_t BTN_DEBOUNCE_MS  = 50;
const uint32_t BTN_LONG_MS      = 3000;
const uint32_t INACTIVITY_MS    = 60000;  // 60 s to drop to IDLEdasd
const float    MAJOR_CHANGE_G   = 1.0f;   // Significant movement threshold

/* ────────────────────────────── SCALE / FILTER ───────────────────────────── */
float g_scale       = 420.0f;  // Runtime scale factor (updated by calibration)
const float DEAD_BAND_G = 0.10f; // ±0.1 g
long  g_tareOffset  = 0;       // Raw offset (tare)
float CAL_WEIGHT_G  = 500.0f;   // Known weight for calibration (grams) //////// *Change whatever known weight you have. <----------

/* ────────────────────────────── HX711 STATE ─────────────────────────────── */
volatile bool     g_hxReadyFlag   = false;   // Set by ISR when DOUT falls
volatile float    g_units_shared  = 0.0f;    // Published (filtered & clamped) grams
volatile uint16_t g_weight_x10    = 0;       // Cached grams*10 for fast I2C onRequest
bool              g_units_init    = false;

/* ───────────────────────────── DISPLAY STATE ─────────────────────────────── */
constexpr uint8_t SEG_A = 1 << 0;
constexpr uint8_t SEG_B = 1 << 1;
constexpr uint8_t SEG_C = 1 << 2;
constexpr uint8_t SEG_D = 1 << 3;
constexpr uint8_t SEG_E = 1 << 4;
constexpr uint8_t SEG_F = 1 << 5;
constexpr uint8_t SEG_G = 1 << 6;

const uint8_t DIGIT_GLYPH[10] = {
  /*0*/ (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F),
  /*1*/ (SEG_B|SEG_C),
  /*2*/ (SEG_A|SEG_B|SEG_D|SEG_E|SEG_G),
  /*3*/ (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G),
  /*4*/ (SEG_F|SEG_G|SEG_B|SEG_C),
  /*5*/ (SEG_A|SEG_F|SEG_G|SEG_C|SEG_D),
  /*6*/ (SEG_A|SEG_F|SEG_E|SEG_D|SEG_C|SEG_G),
  /*7*/ (SEG_A|SEG_B|SEG_C),
  /*8*/ (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),
  /*9*/ (SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G)
};

const uint8_t GLYPH_BLANK = 0x00;
const uint8_t GLYPH_C     = (SEG_A|SEG_F|SEG_E|SEG_D);
const uint8_t GLYPH_A     = (SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G);
const uint8_t GLYPH_L     = (SEG_F|SEG_E|SEG_D);
const uint8_t GLYPH_S     = (SEG_A|SEG_F|SEG_G|SEG_C|SEG_D);  // 'S' like '5'
const uint8_t GLYPH_E     = (SEG_A|SEG_F|SEG_G|SEG_E|SEG_D);
const uint8_t GLYPH_t     = (SEG_F|SEG_E|SEG_D|SEG_G);        // 't' approx.
const uint8_t GLYPH_H     = (SEG_F|SEG_E|SEG_B|SEG_C|SEG_G);
const uint8_t GLYPH_I     = (SEG_B|SEG_C);                    // like digit '1'

volatile uint8_t g_glyph[3] = {0, 0, 0};
volatile bool    g_dp[3]    = {false, false, false};
uint8_t          g_scanIdx  = 0;
uint32_t         g_lastScan = 0;
bool             g_displayEnabled = false;        // OFF in IDLE
volatile bool    g_overflowDisplay = false;       // Show "HI" when true

/* ─────────────────────────────── UI / MODES ──────────────────────────────── */
enum UIMode : uint8_t { MODE_IDLE = 0, MODE_READING = 1, MODE_CAL_WAIT_WEIGHT = 2 };
UIMode g_mode = MODE_IDLE;

// Inactivity tracking (only in READING)
float    g_lastSignificant = 0.0f;
uint32_t g_lastChangeMs    = 0;

/* ───────────────────────────── BUTTON DEBOUNCE ───────────────────────────── */
bool     btn_lastReading  = HIGH;
bool     btn_stable       = HIGH;
uint32_t btn_lastDebounce = 0;
bool     btn_pressed      = false;
uint32_t btn_pressStart   = 0;
bool     btn_longFired    = false;

/* ───────────────────────── I2C / TARGET SESSION STATE ────────────────────── */
volatile bool     g_targetActive       = false;  // Armed session
volatile uint16_t g_targetX10          = 0;      // grams*10
volatile bool     g_targetHitLatched   = false;  // Event flag (one-shot on read)
volatile bool     g_i2cCmdPending      = false;  // Set by onReceive
volatile uint16_t g_i2cPendingTargetX10= 0;      // Queued target

/* ────────────────────────────────── MISC ─────────────────────────────────── */
uint32_t g_lastPrintMs = 0;

/* ────────────────────────── EEPROM (Scale only) ──────────────────────────── */
const int      EEPROM_ADDR        = 0;
const uint32_t EEPROM_MAGIC_SCALE = 0x53434C45UL; // "SCLE"

uint16_t sum16_bytes(const uint8_t* p, size_t n) {
  uint32_t s = 0; for (size_t i = 0; i < n; ++i) s += p[i];
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
    g_scale = (pd.scale > 0) ? pd.scale : -pd.scale; // force positive
    Serial.println(F("EEPROM: loaded scale."));
    return true;
  }
  return false;
}

/* ───────────────────────────── LED HELPERS ───────────────────────────────── */
inline void setGreenLED(bool on) { digitalWrite(PIN_LED_GREEN, on ? HIGH : LOW); }
inline void setRedLED(bool on)   { digitalWrite(PIN_LED_RED,   on ? HIGH : LOW); }

/* ──────────────────────────── HX711 LOW-LEVEL ────────────────────────────── */
inline long hx_read_now_assuming_ready() {
  unsigned long v = 0;
  for (byte i = 0; i < 24; i++) {
    digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1);
    v = (v << 1) | (digitalRead(PIN_DOUT) ? 1UL : 0UL);
    digitalWrite(PIN_SCK, LOW);  delayMicroseconds(1);
  }
  digitalWrite(PIN_SCK, HIGH); delayMicroseconds(1);   // gain=128
  digitalWrite(PIN_SCK, LOW);  delayMicroseconds(1);
  if (v & 0x800000UL) v |= 0xFF000000UL;              // sign-extend
  return (long)v;
}

bool hx_wait_ready_timeout(uint32_t timeout_ms = 1000) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DOUT)) { if (millis() - t0 > timeout_ms) return false; }
  return true;
}

long hx_read_blocking() {
  if (!hx_wait_ready_timeout()) return 0;
  return hx_read_now_assuming_ready();
}

// Average N samples WITHOUT freezing display (used in tare/calibration)
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

/* ────────────────────────────────── ISR ──────────────────────────────────── */
void hx_dataReadyISR() { g_hxReadyFlag = true; }

/* ───────────────────────── DISPLAY LOW-LEVEL ─────────────────────────────── */
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

/* ───────────── DISPLAY HELPERS (atomic updates to prevent tearing) ───────── */
void display_showCAL() {
  noInterrupts();
  g_glyph[0] = GLYPH_C; g_glyph[1] = GLYPH_A; g_glyph[2] = GLYPH_L;
  g_dp[0] = g_dp[1] = g_dp[2] = false;
  interrupts();
}

void display_showSET() { // "SEt"
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

// Not used directly in loop (kept for completeness; uses atomic writes).
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

/* ───────────────────────────── MODE HELPERS ──────────────────────────────── */
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

/* ───────────────────────────── TARGET HANDLING ───────────────────────────── */
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

/* ───────────── SHORT PRESS (PHYSICAL) = TARE (shows "SEt") ───────────────── */
void doShortPressTare() {
  g_displayEnabled = true;
  display_showSET();

  detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
  long t = hx_read_average_withScan(16);
  g_tareOffset = t;
  g_hxReadyFlag = false;
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  Serial.println(F("Tare done (button)."));
  setRedLED(false);
  noInterrupts(); g_targetActive = false; interrupts();
  g_overflowDisplay = false; // reset overflow indicator on tare

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);
}

/* ─────────────────────────────── STARTUP TARE ────────────────────────────── */
void doStartupTare() {
  g_displayEnabled = true;
  display_showSET();

  long t = hx_read_average_withScan(16);
  g_tareOffset = t;
  Serial.println(F("Tare done (startup)."));

  setRedLED(false);
  g_overflowDisplay = false; // reset overflow indicator on tare

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);
}

/* ───────────────────── I2C: queue 'R'+target_x10 (tare+arm) ──────────────── */
void queueI2CArm(uint16_t tx10) {
  noInterrupts();
  g_i2cPendingTargetX10 = (tx10 > 9990) ? 9990 : tx10;
  g_i2cCmdPending = true;
  interrupts();
}

/* ───────────────────────────── BUTTON HANDLER ────────────────────────────── */
void captureCalibrationIfInCAL(); // forward

void pollButton() {
  bool reading = digitalRead(PIN_BTN); // HIGH=idle, LOW=pressed
  if (reading != btn_lastReading) { btn_lastDebounce = millis(); btn_lastReading = reading; }

  if (millis() - btn_lastDebounce > BTN_DEBOUNCE_MS) {
    if (reading != btn_stable) {
      btn_stable = reading;
      if (btn_stable == LOW) {
        btn_pressed = true; btn_pressStart = millis(); btn_longFired = false;
      } else {
        // Released
        uint32_t dur = millis() - btn_pressStart; btn_pressed = false;
        if (!btn_longFired && dur >= BTN_DEBOUNCE_MS) {
          if (g_mode == MODE_CAL_WAIT_WEIGHT) {
            captureCalibrationIfInCAL(); // short press in CAL completes calibration (NO tare here)
          } else {
            doShortPressTare();          // short press outside CAL = TARE with "SEt"
          }
        }
      }
    }

    // Long press detection while held
    if (btn_pressed && !btn_longFired) {
      uint32_t held = millis() - btn_pressStart;
      if (held >= BTN_LONG_MS) {
        btn_longFired = true;

        // Auto-enter CAL immediately (NO "SEt"); quick silent tare ONCE; show "CAL"
        Serial.println(F("Entering CAL mode..."));
        enterCal();  // show CAL immediately

        detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
        long t = hx_read_average_withScan(16); // one-time silent tare at CAL entry
        g_tareOffset = t;
        setRedLED(false);
        attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

        Serial.println(F("Place known weight, then short-press to confirm."));
      }
    }
  }
}

/* ─────────────────────────────── I2C HANDLERS ────────────────────────────── */
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

/* ──────────────── CAL CONFIRM (short press while in CAL) ─────────────────── */
void captureCalibrationIfInCAL() {
  if (g_mode != MODE_CAL_WAIT_WEIGHT) return;

  Serial.println(F("Capturing calibration weight..."));
  long avg   = hx_read_average_withScan(32);   // DO NOT tare here
  long delta = avg - g_tareOffset;             // use tare captured at CAL entry

  if (delta != 0) {
    g_scale = (float)delta / CAL_WEIGHT_G;     // update scale factor
    Serial.print(F("New scale factor: "));
    Serial.println(g_scale, 6);
    eepromSaveScaleOnly();                     // save SCALE only
  } else {
    Serial.println(F("Calibration failed: delta=0"));
  }

  g_hxReadyFlag = false;
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  float val; noInterrupts(); val = g_units_shared; interrupts();
  enterReading(val);                            // resume reading with new scale
}

/* ──────────────────────────────── SETUP / LOOP ───────────────────────────── */
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

  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  // Load persisted scale factor; we still TARE at startup (shows "SEt")
  if (eepromLoadScaleOnly()) Serial.println(F("EEPROM OK. Will TARE at startup."));
  else                       Serial.println(F("EEPROM invalid/empty. Will TARE at startup."));

  // Startup TARE (shows "SEt"), then attach ISR
  doStartupTare();
  attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

  Serial.println(F("Ready. In READING after startup TARE. I2C 'R'+target_x10 will TARE then arm target."));
}

void loop() {
  display_scanStep();
  pollButton();

  // Handle queued I2C 'R': TARE (shows "SEt"), Red off, READING, then arm target
  if (g_i2cCmdPending && g_mode != MODE_CAL_WAIT_WEIGHT) {
    noInterrupts(); g_i2cCmdPending = false; uint16_t tx10 = g_i2cPendingTargetX10; interrupts();

    g_displayEnabled = true;
    display_showSET();

    detachInterrupt(digitalPinToInterrupt(PIN_DOUT));
    long t = hx_read_average_withScan(16);
    g_tareOffset = t;
    g_hxReadyFlag = false;
    attachInterrupt(digitalPinToInterrupt(PIN_DOUT), hx_dataReadyISR, FALLING);

    Serial.println(F("Tare done (I2C 'R')."));
    setRedLED(false);
    g_overflowDisplay = false; // reset overflow indicator on tare

    float val; noInterrupts(); val = g_units_shared; interrupts();
    enterReading(val);
    armTarget(tx10);
  }

  // CAL wait state (showing "CAL"); short press to capture scale
  if (g_mode == MODE_CAL_WAIT_WEIGHT) {
    // Waiting for short press; handled in pollButton()
  } else {
    // Service HX711
    if (g_hxReadyFlag) {
      g_hxReadyFlag = false;
      if (!digitalRead(PIN_DOUT)) {
        long  raw       = hx_read_now_assuming_ready();
        float candidate = (raw - g_tareOffset) / g_scale;

        // Track overflow BEFORE clamping
        bool overflowNow = (candidate > 999.0f);

        // Clamp to [0 .. 999] for numeric/I2C paths
        if (candidate < 0.0f)   candidate = 0.0f;
        if (candidate > 999.0f) candidate = 999.0f;

        if (!g_units_init) {
          noInterrupts();
          g_units_shared = candidate;
          g_weight_x10   = (uint16_t)min(9990L, (long)(candidate * 10.0f + 0.5f));
          g_overflowDisplay = overflowNow;
          interrupts();
          g_units_init = true;

          if (g_mode == MODE_READING && fabsf(candidate - g_lastSignificant) >= MAJOR_CHANGE_G) {
            g_lastSignificant = candidate;
            g_lastChangeMs = millis();
          } else if (g_mode == MODE_READING) {
            g_lastChangeMs = millis();
          }
        } else {
          float prev; noInterrupts(); prev = g_units_shared; interrupts();
          if (fabsf(candidate - prev) >= DEAD_BAND_G || overflowNow != g_overflowDisplay) {
            noInterrupts();
            g_units_shared = candidate;
            g_weight_x10   = (uint16_t)min(9990L, (long)(candidate * 10.0f + 0.5f));
            g_overflowDisplay = overflowNow;
            interrupts();

            if (g_mode == MODE_READING && fabsf(candidate - g_lastSignificant) >= MAJOR_CHANGE_G) {
              g_lastSignificant = candidate;
              g_lastChangeMs = millis();
            }
          }
        }
      }
    }

    float val; noInterrupts(); val = g_units_shared; interrupts();
    checkTargetHitOnUpdate(val);

    if (g_mode == MODE_READING) {
      // Overflow display "HI" or numeric value (atomic writes to avoid tearing)
      bool of; noInterrupts(); of = g_overflowDisplay; interrupts();
      if (of) {
        display_showHI();
      } else {
        float av = val;
        noInterrupts();  // ATOMIC: update all three digits + dps together
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
      // Keep blank
    }

    // Serial diagnostics ~5 Hz (200 ms)
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
  }
}
