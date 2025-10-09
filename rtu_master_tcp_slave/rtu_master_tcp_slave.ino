/* ---------------- Fix for Arduino auto-prototype issue ----------------
   Define ApiResult BEFORE includes so the IDE's auto-generated prototypes
   can "see" it. Only <stdint.h> is needed for the struct here.
-------------------------------------------------------------------------*/
#include <stdint.h>
struct ApiResult {
  uint16_t resultCode;
  uint16_t faultCode;
  uint16_t systemStatus;
  uint16_t seq;
  uint16_t lastWeight_x10;
  uint16_t elapsed_s_x10;
  uint16_t activeScaleId;
  uint16_t aux3;
  uint16_t list_milk;
  uint16_t list_sauce_lo;
  uint16_t list_sauce_hi;
  bool ok;
};

/* ----------------------- Modbus RTU Master ---------------------------
   Board: Arduino Nano Every (hardware Serial1)
   MAX485: DI<-TX1, RO->RX1, DE+RE -> D2
   Bus: 19200 8N1
   Protocol: Function 0x17 single-shot (write cmd @0x0000, read result @0x0100)
------------------------------------------------------------------------*/
#include <Arduino.h>

// -------------------- Link/RS485 config --------------------
#define MB_SLAVE_ID        1
#define RS485_PORT         Serial1
#define RS485_DIR_PIN      2          // DE+RE tied here
#define MB_BAUD            19200

// -------------------- Register map constants --------------------
#define REG_CMD_START      0x0000
#define REG_RES_START      0x0100

// Opcodes (must match the slave)
enum Opcode : uint16_t {
  OP_STATUS   = 1,
  OP_CLEAR    = 2,
  OP_LIST     = 3,
  OP_DISPENSE = 10,
  OP_RINSE    = 11,
  OP_TRIGGER  = 12
};

// -------------------- Low-level helpers --------------------
static uint16_t mb_crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) { crc = (crc >> 1) ^ 0xA001; }
      else         { crc >>= 1; }
    }
  }
  return crc;
}

static inline void rs485Tx() { digitalWrite(RS485_DIR_PIN, HIGH); delayMicroseconds(100); }
static inline void rs485Rx() { delayMicroseconds(100); digitalWrite(RS485_DIR_PIN, LOW); }

// Send a full 0x17 request and read the response (with long timeout)
static bool modbus17_transaction(uint8_t slaveId,
                                 uint16_t readStart, uint16_t readQty,
                                 uint16_t writeStart, const uint16_t* writeRegs, uint16_t writeQty,
                                 uint16_t* readOut, uint16_t readOutQty,
                                 uint32_t timeoutMs)
{
  const uint8_t func = 0x17;
  const uint8_t writeBytes = (uint8_t)(writeQty * 2);
  const uint16_t reqLenNoCRC = 1+1+2+2+2+2+1 + writeBytes;
  const uint16_t reqLen = reqLenNoCRC + 2;
  if (reqLen > 260) return false;

  uint8_t req[260];
  uint16_t p = 0;
  req[p++] = slaveId;
  req[p++] = func;
  req[p++] = (uint8_t)(readStart >> 8); req[p++] = (uint8_t)(readStart & 0xFF);
  req[p++] = (uint8_t)(readQty   >> 8); req[p++] = (uint8_t)(readQty   & 0xFF);
  req[p++] = (uint8_t)(writeStart>> 8); req[p++] = (uint8_t)(writeStart& 0xFF);
  req[p++] = (uint8_t)(writeQty  >> 8); req[p++] = (uint8_t)(writeQty  & 0xFF);
  req[p++] = writeBytes;

  for (uint16_t i=0;i<writeQty;i++){
    req[p++] = (uint8_t)(writeRegs[i] >> 8);
    req[p++] = (uint8_t)(writeRegs[i] & 0xFF);
  }

  const uint16_t crc = mb_crc16(req, p);
  req[p++] = (uint8_t)(crc & 0xFF);
  req[p++] = (uint8_t)(crc >> 8);

  // Purge any stale input
  while (RS485_PORT.available()) RS485_PORT.read();

  // TX
  rs485Tx();
  RS485_PORT.write(req, p);
  RS485_PORT.flush();
  delayMicroseconds(2000); // 2 char times @19200
  rs485Rx();

  // RX (single response; may be delayed up to timeoutMs)
  const uint32_t t0 = millis();
  auto waitFor = [&](size_t n)->bool{
    while (RS485_PORT.available() < (int)n) {
      if ((millis() - t0) > timeoutMs) return false;
      delay(1);
    }
    return true;
  };

  // Minimum header: addr, func, byteCount
  if (!waitFor(3)) return false;
  uint8_t addr = RS485_PORT.read();
  uint8_t fn   = RS485_PORT.read();
  uint8_t byteCount = RS485_PORT.read();
  if (addr != slaveId) return false;

  // Exception?
  if ((fn & 0x80) != 0) {
    if (!waitFor(3)) return false;
    (void)RS485_PORT.read(); // ex
    (void)RS485_PORT.read(); // crcl
    (void)RS485_PORT.read(); // crch
    return false;
  }
  if (fn != func) return false;

  const uint16_t expectedDataBytes = readQty * 2;
  if (byteCount != expectedDataBytes) return false;

  if (!waitFor(byteCount + 2)) return false;
  uint8_t dataBuf[512];
  for (uint16_t i=0;i<byteCount;i++) dataBuf[i] = RS485_PORT.read();
  uint8_t crcl = RS485_PORT.read();
  uint8_t crch = RS485_PORT.read();

  // CRC over (addr, func, byteCount, data...)
  uint8_t hdr[3] = {addr, func, byteCount};
  uint8_t tmp[3 + 512];
  memcpy(tmp, hdr, 3);
  memcpy(tmp+3, dataBuf, byteCount);
  uint16_t crcCalc = mb_crc16(tmp, 3+byteCount);
  uint16_t crcGot  = (uint16_t)crcl | ((uint16_t)crch << 8);
  if (crcCalc != crcGot) return false;

  // Unpack into readOut
  const uint16_t regsAvail = byteCount / 2;
  const uint16_t ncopy = min(readOutQty, regsAvail);
  for (uint16_t i=0;i<ncopy;i++) {
    readOut[i] = (uint16_t)( (dataBuf[2*i] << 8) | dataBuf[2*i+1] );
  }
  return true;
}

// -------------------- API wrappers --------------------
static const uint16_t READ_START = REG_RES_START;  // 0x0100
static const uint16_t READ_QTY   = 16;             // read 16 regs (0x0100..0x010F)
static const uint16_t WRITE_START= REG_CMD_START;  // 0x0000

static ApiResult parseResult(const uint16_t* r) {
  ApiResult a{};
  a.resultCode      = r[0];
  a.faultCode       = r[1];
  a.systemStatus    = r[2];
  a.seq             = r[3];
  a.lastWeight_x10  = r[4];
  a.elapsed_s_x10   = r[5];
  a.activeScaleId   = r[6];
  a.aux3            = r[7];
  a.list_milk       = r[8];
  a.list_sauce_lo   = r[9];
  a.list_sauce_hi   = r[10];
  a.ok = (a.resultCode == 0);
  return a;
}

static uint16_t seqCounter = 1;

bool api_status(ApiResult& out, uint32_t timeoutMs=3000) {
  uint16_t w[] = { OP_STATUS, seqCounter++, 0,0,0,0,0 };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 2, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

bool api_clear(ApiResult& out, uint32_t timeoutMs=3000) {
  uint16_t w[] = { OP_CLEAR, seqCounter++, 0,0,0,0,0 };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 2, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

bool api_list(ApiResult& out, uint32_t timeoutMs=3000) {
  uint16_t w[] = { OP_LIST, seqCounter++, 0,0,0,0,0 };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 2, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

bool api_rinse(float seconds, ApiResult& out, uint32_t timeoutMs=15000) {
  uint16_t sec_x10 = (uint16_t)(seconds * 10.0f + 0.5f);
  uint16_t w[] = { OP_RINSE, seqCounter++, sec_x10, 0,0,0,0 };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 3, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

bool api_trigger(uint16_t motorId, float seconds, ApiResult& out, uint32_t timeoutMs=20000) {
  uint16_t sec_x10 = (uint16_t)(seconds * 10.0f + 0.5f);
  uint16_t w[] = { OP_TRIGGER, seqCounter++, motorId, sec_x10, 0,0,0 };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 4, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

bool api_dispense(uint16_t motorId, float target_g, float slowOffset_g, float softCutOffset_g, uint16_t timeout_s,
                  ApiResult& out,
                  uint32_t timeoutMs)
{
  uint16_t tgt_x10   = (uint16_t)(target_g * 10.0f + 0.5f);
  uint16_t slow_x10  = (uint16_t)(slowOffset_g * 10.0f + 0.5f);
  uint16_t soft_x10  = (uint16_t)(softCutOffset_g * 10.0f + 0.5f);

  uint16_t w[] = { OP_DISPENSE, seqCounter++, motorId, tgt_x10, slow_x10, soft_x10, timeout_s };
  uint16_t readBuf[READ_QTY];
  if (!modbus17_transaction(MB_SLAVE_ID, READ_START, READ_QTY, WRITE_START, w, 7, readBuf, READ_QTY, timeoutMs)) return false;
  out = parseResult(readBuf); return true;
}

// -------------------- Demo / usage --------------------
static void printResult(const char* label, const ApiResult& r) {
  Serial.print(label);
  Serial.print(" -> resultCode="); Serial.print(r.resultCode);
  Serial.print(" faultCode="); Serial.print(r.faultCode);
  Serial.print(" systemStatus="); Serial.print(r.systemStatus);
  Serial.print(" seq="); Serial.print(r.seq);
  Serial.print(" lastWeight="); Serial.print(r.lastWeight_x10/10.0f, 1);
  Serial.print("g elapsed="); Serial.print(r.elapsed_s_x10/10.0f, 1);
  Serial.print("s scaleId="); Serial.print(r.activeScaleId);
  Serial.println();
}

void setup() {
  pinMode(RS485_DIR_PIN, OUTPUT);
  rs485Rx();

  Serial.begin(115200);
  while (!Serial) { /* wait for USB */ }

  RS485_PORT.begin(MB_BAUD);

  Serial.println("Modbus RTU Master ready.");

  // Quick handshake: STATUS
  ApiResult r;
  if (api_status(r)) printResult("STATUS", r); else Serial.println("STATUS failed (no response)");
}

void loop() {
  // Example: CLEAR
  ApiResult r;
  if (api_clear(r)) printResult("CLEAR", r); else Serial.println("CLEAR failed");

  delay(300);

  // Example: LIST (bitmasks)
  if (api_list(r)) {
    printResult("LIST", r);
    Serial.print("  milk mask: 0x"); Serial.println(r.list_milk, HEX);
    Serial.print("  sauce lo : 0x"); Serial.println(r.list_sauce_lo, HEX);
  } else {
    Serial.println("LIST failed");
  }

  // Example: RINSE 2.5 s
  if (api_rinse(2.5f, r, 15000)) printResult("RINSE", r); else Serial.println("RINSE failed");

  // Example: TRIGGER Milk3 for 1.2 s  (Milk3 => motorId=3)
  if (api_trigger(3, 1.2f, r, 20000)) printResult("TRIGGER", r); else Serial.println("TRIGGER failed");

  // Example: DISPENSE Milk3 100 g, slow @98 g, soft-cut @99 g, timeout 30 s
  const uint16_t timeout_s = 30;
  const uint32_t masterWaitMs = (timeout_s + 3) * 1000UL; // cushion
  if (api_dispense(3, 100.0f, 2.0f, 1.0f, timeout_s, r, masterWaitMs)) {
    printResult("DISPENSE", r);
  } else {
    Serial.println("DISPENSE failed (timeout/no response)");
  }

  Serial.println("---- cycle done ----");
  delay(5000);
}
