/*
 * Photodiode selector + LED driver for Arduino Nano ESP32
 *
 *   12x TSL2591 photodiodes (all at I2C 0x29) behind two TCA9548A muxes
 *     U1 @ 0x70 (A2=A1=A0=GND)  -> photodiodes 1-8  on channels 0-7
 *     U2 @ 0x71 (A0=3.3V)       -> photodiodes 9-12 on channels 0-3
 *   12x LEDs on a TLC6C5912 shift-register LED driver
 *     LEDs 1-8 = DRAIN0-7; LEDs 9-12 are reversed (9 = DRAIN11 ... 12 = DRAIN8)
 *
 * All numbers you type are 1-12 (not 0-based).
 *
 * Serial commands (115200 baud, type a command and press Enter):
 *   s <1-12>   select a photodiode -> its readings print continuously
 *   s off      disconnect the photodiode and stop readings
 *   p <1-12>   same as s, plus turn on the LED next to it
 *   x          stop live readings
 *   r          one reading of the selected photodiode
 *   a          one reading of every photodiode
 *   (Live readings keep running while you use l / m / g / t / r / a.)
 *   i          scan the I2C bus
 *   l <1-12>   turn on that LED only
 *   l off      all LEDs off
 *   l all      all LEDs on
 *   m <mask>   set LEDs by bitmask, bit 0 = LED 1, e.g. m 0x005 (LEDs 1 + 3)
 *   g <0-3>    sensor gain: 0=low(1x) 1=med(25x) 2=high(400x) 3=max(9200x)
 *   t <0-5>    integration time: (t+1) x 100 ms
 *   ?          help
 *
 * Internally the code still uses 0-based indexes; conversion happens only
 * where commands are parsed and where numbers are printed.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// ---------------- Pin map (from the Nano shield schematic) ----------------
const int PIN_MUX1_RESET = D3;   // RESET1 -> U1 /RESET (active low)
const int PIN_MUX2_RESET = D4;   // RESET2 -> U2 /RESET (active low)
const int PIN_LED_G      = D8;   // TLC6C5912 /G   output enable (active low)
const int PIN_LED_CLR    = D9;   // TLC6C5912 /CLR clear (active low)
const int PIN_LED_RCK    = D10;  // TLC6C5912 RCK  latch
// SER IN = D11 (COPI), SRCK = D13 (SCK) -> driven by hardware SPI
// SDA = A4, SCL = A5 -> Wire defaults

// ---------------- I2C addresses ----------------
const uint8_t MUX_ADDR[2] = {0x70, 0x71};
const uint8_t TSL_ADDR    = 0x29;
const uint8_t NUM_SENSORS = 12;
const uint8_t NUM_LEDS    = 12;

// ---------------- TSL2591 registers ----------------
const uint8_t TSL_CMD         = 0xA0;  // CMD=1, TRANSACTION=01 (normal)
const uint8_t TSL_REG_ENABLE  = 0x00;
const uint8_t TSL_REG_CONFIG  = 0x01;
const uint8_t TSL_REG_ID      = 0x12;
const uint8_t TSL_REG_STATUS  = 0x13;
const uint8_t TSL_REG_C0DATAL = 0x14;
const uint8_t TSL_PON         = 0x01;
const uint8_t TSL_AEN         = 0x02;
const uint8_t TSL_AVALID      = 0x01;
const uint8_t TSL_ID_VALUE    = 0x50;

// Typical gain multipliers from the datasheet (CH0)
const float GAIN_FACTOR[4] = {1.0f, 24.5f, 400.0f, 9200.0f};

// ---------------- State ----------------
uint8_t  tslGain  = 1;     // medium
uint8_t  tslAtime = 1;     // 200 ms
int      currentSensor = -1;
uint16_t ledState = 0;
bool     sensorPresent[NUM_SENSORS] = {false};
bool     muxPresent[2] = {false, false};
bool     liveMode = false;   // streaming readings?

// LED index (LED number - 1) -> TLC6C5912 drain output it's wired to.
// LEDs 9-12 are wired in reverse: l 9 = DRAIN11, l 10 = DRAIN10, l 11 = DRAIN9, l 12 = DRAIN8.
// LED n sits next to photodiode n.
const uint8_t LED_TO_DRAIN[NUM_LEDS] = {0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9, 8};

// =====================================================================
//                         I2C MUX (TCA9548A)
// =====================================================================
bool muxWrite(uint8_t mux, uint8_t channelMask) {
  Wire.beginTransmission(MUX_ADDR[mux]);
  Wire.write(channelMask);           // bit n = channel n connected
  return Wire.endTransmission() == 0;
}

void deselectAll() {
  for (uint8_t m = 0; m < 2; m++) {
    if (muxPresent[m]) muxWrite(m, 0x00);
  }
  currentSensor = -1;
}

// Connect photodiode n (0..11) to the main I2C bus.
// Every TSL2591 is at 0x29, so exactly ONE channel across BOTH muxes may be open.
bool selectSensor(uint8_t n, bool report = true) {
  if (n >= NUM_SENSORS) return false;
  uint8_t mux = n / 8;
  uint8_t ch  = n % 8;
  if (!muxPresent[mux]) {
    if (report) Serial.printf("  sensor %u is on mux U%u (0x%02X), which isn't fitted/detected\n",
                              (unsigned)(n + 1), mux + 1, MUX_ADDR[mux]);
    currentSensor = -1;
    return false;
  }
  // Close the other mux first (only if it's fitted).
  if (muxPresent[1 - mux]) muxWrite(1 - mux, 0x00);
  if (!muxWrite(mux, 1 << ch)) {
    if (report) Serial.printf("  mux U%u @0x%02X did not respond\n", mux + 1, MUX_ADDR[mux]);
    currentSensor = -1;
    return false;
  }
  currentSensor = n;
  return true;
}

// List every device that answers on the main I2C bus (all mux channels closed).
// Expected: 0x70 and 0x71 only.
void i2cScan() {
  deselectAll();
  Serial.println("Scanning main I2C bus (all mux channels closed)...");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  nothing found");
  Serial.println("Expected: 0x70 (U1) and 0x71 (U2)");
}

void resetMuxes() {
  digitalWrite(PIN_MUX1_RESET, LOW);
  digitalWrite(PIN_MUX2_RESET, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_MUX1_RESET, HIGH);   // must stay HIGH for the mux to work
  digitalWrite(PIN_MUX2_RESET, HIGH);
  delay(1);
}

// =====================================================================
//                         TSL2591 light sensor
// =====================================================================
bool tslWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TSL_ADDR);
  Wire.write(TSL_CMD | reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool tslRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(TSL_ADDR);
  Wire.write(TSL_CMD | reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)TSL_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Restart the ALS so the next reading uses the current gain/time settings.
bool tslStart() {
  bool ok = tslWrite(TSL_REG_ENABLE, TSL_PON);                       // ALS off
  ok &= tslWrite(TSL_REG_CONFIG, (tslGain << 4) | (tslAtime & 0x07));
  ok &= tslWrite(TSL_REG_ENABLE, TSL_PON | TSL_AEN);                 // ALS on
  return ok;
}

// Init the currently selected sensor. Returns false if it isn't a TSL2591.
bool tslInit() {
  uint8_t id;
  if (!tslRead(TSL_REG_ID, &id, 1) || id != TSL_ID_VALUE) return false;
  return tslStart();
}

// Read raw counts from the currently selected sensor.
// ch0 = visible + IR, ch1 = IR only.
bool readSensor(uint16_t &ch0, uint16_t &ch1) {
  uint8_t status = 0;
  uint32_t timeout = 110UL * (tslAtime + 1) + 50;
  uint32_t start = millis();
  do {   // wait until at least one integration cycle has completed
    if (!tslRead(TSL_REG_STATUS, &status, 1)) return false;
    if (status & TSL_AVALID) break;
    delay(5);
  } while (millis() - start < timeout);
  if (!(status & TSL_AVALID)) return false;

  uint8_t b[4];   // read all 4 data bytes in one go (datasheet recommendation)
  if (!tslRead(TSL_REG_C0DATAL, b, 4)) return false;
  ch0 = b[0] | (b[1] << 8);
  ch1 = b[2] | (b[3] << 8);
  return true;
}

// Approximate lux (empirical formula, same as Adafruit's TSL2591 library).
// Returns -1 if the sensor is saturated.
float countsToLux(uint16_t ch0, uint16_t ch1) {
  uint16_t maxCount = (tslAtime == 0) ? 36863 : 65535;
  if (ch0 >= maxCount || ch1 >= maxCount) return -1.0f;
  if (ch0 == 0) return 0.0f;
  float atimeMs = 100.0f * (tslAtime + 1);
  float cpl = (atimeMs * GAIN_FACTOR[tslGain]) / 408.0f;
  float lux = ((float)ch0 - ch1) * (1.0f - (float)ch1 / ch0) / cpl;
  return lux < 0 ? 0.0f : lux;
}

void printReading(uint8_t n) {
  if (!selectSensor(n)) return;   // selectSensor prints the reason
  tslStart();   // fresh integration, so the reading reflects the current LED state
  uint16_t ch0, ch1;
  if (!readSensor(ch0, ch1)) {
    Serial.printf("Sensor %2u: no response\n", (unsigned)(n + 1));
    return;
  }
  float lux = countsToLux(ch0, ch1);
  Serial.printf("Sensor %2u  CH0(full)=%5u  CH1(IR)=%5u  ",
                (unsigned)(n + 1), (unsigned)ch0, (unsigned)ch1);
  if (lux < 0) Serial.println("SATURATED (lower gain/time)");
  else         Serial.printf("lux=%.3f\n", lux);
}

void applyConfigToAll() {
  for (uint8_t n = 0; n < NUM_SENSORS; n++) {
    if (sensorPresent[n] && selectSensor(n, false)) tslStart();
  }
  Serial.printf("Gain=%u (%.1fx), integration=%u ms\n",
                tslGain, GAIN_FACTOR[tslGain], 100 * (tslAtime + 1));
}

void scanSensors() {
  for (uint8_t m = 0; m < 2; m++) {
    Wire.beginTransmission(MUX_ADDR[m]);
    muxPresent[m] = Wire.endTransmission() == 0;
    Serial.printf("Mux U%u @0x%02X: %s\n", m + 1, MUX_ADDR[m],
                  muxPresent[m] ? "OK" : "not found (its sensors will be skipped)");
  }
  Serial.print("Sensors found: ");
  for (uint8_t n = 0; n < NUM_SENSORS; n++) {
    sensorPresent[n] = selectSensor(n, false) && tslInit();
    if (sensorPresent[n]) Serial.printf("%u ", (unsigned)(n + 1));
  }
  Serial.println();
  deselectAll();
}

// =====================================================================
//                         LED driver (TLC6C5912)
// =====================================================================
// Raw write: bit n of mask -> DRAINn (no LED renumbering here).
void ledWrite(uint16_t mask) {
  mask &= 0x0FFF;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // 16 clocks: the top 4 bits fall off the end of the 12-bit register,
  // leaving bit 11 on DRAIN11 ... bit 0 on DRAIN0.
  SPI.transfer16(mask);
  SPI.endTransaction();
  digitalWrite(PIN_LED_RCK, HIGH);   // rising edge latches data to outputs
  delayMicroseconds(1);
  digitalWrite(PIN_LED_RCK, LOW);
  ledState = mask;
}

void ledOn(uint8_t n) {           // only LED n on (uses LED_TO_DRAIN)
  if (n < NUM_LEDS) ledWrite(1u << LED_TO_DRAIN[n]);
}

void ledsOff() { ledWrite(0); }

// Convert a mask of LED numbers (bit 0 = LED 1) to the drain outputs they're wired to.
uint16_t ledMaskToDrain(uint16_t userMask) {
  uint16_t drainMask = 0;
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    if (userMask & (1u << i)) drainMask |= 1u << LED_TO_DRAIN[i];
  }
  return drainMask;
}

// Easter egg ('w'): a wave across the board, left column to right column.
// Columns from the schematic: LEDs 1-4 | LEDs 5-8 | LEDs 9-12 (bit 0 = LED 1).
// If your board's columns are arranged differently, edit this table.
const uint16_t WAVE_COLUMNS[] = {0x00F, 0x0F0, 0xF00};
void ledWave() {
  uint16_t saved = ledState;                 // restore whatever was on before
  const uint8_t nCols = sizeof(WAVE_COLUMNS) / sizeof(WAVE_COLUMNS[0]);
  for (uint8_t pass = 0; pass < 3; pass++) {
    for (uint8_t c = 0; c < nCols; c++) {
      ledWrite(ledMaskToDrain(WAVE_COLUMNS[c]));
      delay(120);
      if (c + 1 < nCols) {                   // brief overlap = smoother wave
        ledWrite(ledMaskToDrain(WAVE_COLUMNS[c] | WAVE_COLUMNS[c + 1]));
        delay(60);
      }
    }
    ledWrite(0);
    delay(150);
  }
  ledWrite(saved);
}

void ledInit() {
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_CLR, OUTPUT);
  pinMode(PIN_LED_RCK, OUTPUT);
  digitalWrite(PIN_LED_G, HIGH);     // outputs off while we set up
  digitalWrite(PIN_LED_RCK, LOW);
  digitalWrite(PIN_LED_CLR, LOW);    // clear all registers
  delayMicroseconds(1);
  digitalWrite(PIN_LED_CLR, HIGH);
  SPI.begin();                       // SCK=D13, CIPO=D12, COPI=D11
  ledWrite(0);
  digitalWrite(PIN_LED_G, LOW);      // enable outputs
}

// =====================================================================
//                         Serial command interface
// =====================================================================
void printHelp() {
  Serial.println(F(
    "\nCommands (numbers are 1-12):\n"
    "  s <1-12>   select photodiode + start live readings\n"
    "  s off      disconnect photodiode, stop readings\n"
    "  p <1-12>   same as s, plus turn on the LED next to it\n"
    "  x          stop live readings\n"
    "  r          one reading of the selected photodiode\n"
    "  a          one reading of every photodiode\n"
    "  l <1-12>   turn on that LED only\n"
    "  l off      all LEDs off\n"
    "  l all      all LEDs on\n"
    "  m <mask>   LED bitmask, bit 0 = LED 1, e.g. m 0x005\n"
    "  g <0-3>    gain (0=low 1=med 2=high 3=max)\n"
    "  t <0-5>    integration time (t+1)*100 ms\n"
    "  i          scan the I2C bus\n"
    "  ?          this help\n"));
}

bool parseIndex(const String &arg, int maxVal, int &out) {
  if (arg.length() == 0 || !isDigit(arg[0])) return false;
  out = arg.toInt();
  return out >= 0 && out <= maxVal;
}

// Commands use numbers 1..count; this returns the 0-based index (number - 1).
bool parseNumber1(const String &arg, int count, int &index) {
  int v;
  if (!parseIndex(arg, count, v) || v < 1) return false;
  index = v - 1;
  return true;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd = tolower(line[0]);
  String arg = line.substring(1);
  arg.trim();
  arg.toLowerCase();
  int n;

  switch (cmd) {
    case 's':
      if (arg == "off") {
        liveMode = false;
        deselectAll();
        Serial.println("Sensor disconnected, live readings stopped");
        break;
      }
      if (!parseNumber1(arg, NUM_SENSORS, n)) { Serial.println("Usage: s <1-12> | s off"); break; }
      if (selectSensor(n)) {
        Serial.printf("Sensor %d connected%s - live readings on (type x to stop)\n", n + 1,
                      sensorPresent[n] ? "" : " (warning: not detected at startup)");
        liveMode = true;
      }
      break;

    case 'x':
      if (liveMode) Serial.println("Live readings stopped");
      liveMode = false;
      break;

    case 'r':
      if (currentSensor < 0) Serial.println("No sensor selected (use s <n>)");
      else printReading(currentSensor);
      break;

    case 'a': {
      int prev = currentSensor;
      bool any = false;
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorPresent[i]) { printReading(i); any = true; }
      }
      if (!any) Serial.println("No sensors were detected at startup (try i, or press reset)");
      if (prev >= 0) selectSensor(prev); else deselectAll();
      break;
    }

    case 'l':
      if (arg == "off")      { ledsOff();       Serial.println("All LEDs off"); }
      else if (arg == "all") { ledWrite(0x0FFF); Serial.println("All LEDs on"); }
      else if (parseNumber1(arg, NUM_LEDS, n)) { ledOn(n); Serial.printf("LED %d on\n", n + 1); }
      else Serial.println("Usage: l <1-12> | l off | l all");
      break;

    case 'm': {
      if (arg.length() == 0) { Serial.println("Usage: m <mask> (bit 0 = LED 1), e.g. m 0x005"); break; }
      uint16_t userMask = (uint16_t)strtol(arg.c_str(), nullptr, 0) & 0x0FFF;
      ledWrite(ledMaskToDrain(userMask));
      Serial.printf("LED mask = 0x%03X\n", userMask);
      break;
    }

    case 'g':
      if (!parseIndex(arg, 3, n)) { Serial.println("Usage: g <0-3>"); break; }
      tslGain = n;
      { int prev = currentSensor; applyConfigToAll(); if (prev >= 0) selectSensor(prev); else deselectAll(); }
      break;

    case 't':
      if (!parseIndex(arg, 5, n)) { Serial.println("Usage: t <0-5>"); break; }
      tslAtime = n;
      { int prev = currentSensor; applyConfigToAll(); if (prev >= 0) selectSensor(prev); else deselectAll(); }
      break;

    case 'i':
      i2cScan();
      break;

    case 'w':                      // easter egg :)
      Serial.println("~~~ wave ~~~");
      ledWave();
      break;

    case 'p':
      if (!parseNumber1(arg, NUM_SENSORS, n)) { Serial.println("Usage: p <1-12>"); break; }
      ledOn(n);                    // LED n is the one next to photodiode n
      if (selectSensor(n)) {
        Serial.printf("Sensor %d selected, LED %d (next to it) on - live readings on (type x to stop)\n",
                      n + 1, n + 1);
        liveMode = true;
      }
      break;

    case '?':
    case 'h':
      printHelp();
      break;

    default:
      Serial.println("Unknown command, type ? for help");
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // wait for USB serial

  pinMode(PIN_MUX1_RESET, OUTPUT);
  pinMode(PIN_MUX2_RESET, OUTPUT);
  resetMuxes();

  ledInit();

  Wire.begin();              // SDA=A4, SCL=A5
  Wire.setClock(100000);
  i2cScan();
  scanSensors();

  printHelp();
}

void loop() {
  static String buf;
  static char prevChar = 0;
  while (Serial.available()) {
    char c = Serial.read();
    // Treat CR+LF as ONE line ending (otherwise the LF looks like an extra
    // empty line, which would immediately stop live mode).
    bool skip = (c == '\n' && prevChar == '\r');
    prevChar = c;
    if (skip) continue;
    if (c == '\n' || c == '\r') {
      handleCommand(buf);
      buf = "";
    } else if (buf.length() < 40) {
      buf += c;
    }
  }

  // Live mode: one new reading per integration period (200 ms by default).
  if (liveMode) {
    if (currentSensor >= 0) {
      printReading(currentSensor);
    } else {
      Serial.println("Lost the selected sensor - live readings stopped");
      liveMode = false;
    }
  }
}