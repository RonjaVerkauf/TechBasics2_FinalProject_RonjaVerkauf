#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ===================== PINS =====================


static const int PIN_SPI_SCK  = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 20;

static const int PIN_SD_CS    = 14;
static const int PIN_SD_MISO  = 19;
static const int PIN_SD_CLK   = 18;
static const int PIN_SD_MOSI  = 20;

static const int PIN_RFID_RST = 22;
static const int PIN_RFID_SDA = 21;

static const int PIN_MPU_SDA  =  0;
static const int PIN_MPU_SCL  =  1;

static const int  PIN_LED_R   =  2;
static const int  PIN_LED_G   =  3;
static const int  PIN_LED_B   =  4;

static const int PIN_BUZZ     =  5;

// ===================== BASE64 =====================
// this function is needed for string encoding for 
// sending username and api-key over HTTP to the
// SMS service
static String base64Encode(const String& input) {
  static const char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const uint8_t* data = (const uint8_t*)input.c_str();
  size_t len = input.length();
  String out;
  out.reserve(((len + 2) / 3) * 4);
  for (size_t i = 0; i < len; i += 3) {
    uint32_t b = (uint32_t)data[i] << 16;
    if (i + 1 < len) b |= (uint32_t)data[i + 1] << 8;
    if (i + 2 < len) b |= (uint32_t)data[i + 2];
    out += table[(b >> 18) & 0x3F];
    out += table[(b >> 12) & 0x3F];
    out += (i + 1 < len) ? table[(b >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? table[(b >> 0) & 0x3F] : '=';
  }
  return out;
}

// ===================== LED =====================

// RGB LEDs exist in two types:
// | Type           | LED turns on when |
// | -------------- | ----------------- |
// | Common cathode | pin = HIGH        |
// | Common anode   | pin = LOW         |
// we are using a common adone RGB LED
static const bool RGB_COMMON_ANODE   = true;  // set false for common cathode

// This function writes a logical LED state (on/off) to one RGB LED pin.
// The electrical behavior depends on the LED wiring:
// 1. Common cathode RGB LED
//    - All cathodes connected to GND
//    - The LED turns ON when the pin is HIGH
// 2. Common anode RGB LED
//    - All anodes connected to VCC (in our case 3V3)
//    - The LED turns ON when the pin is LOW
// The variable RGB_COMMON_ANODE tells us which wiring is used.
// We convert the logical state (on/off) into the correct voltage level (LOW or HIGH).
static void writeRgbPin(int pin, bool on)
{
  if (RGB_COMMON_ANODE) {
    // Common anode:
    //   ON  -> LOW
    //   OFF -> HIGH
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    // Common cathode:
    //   ON  -> HIGH
    //   OFF -> LOW
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

// This function sets the RGB LED color.
// r, g, b represent the red, green, and blue channels:
//   true  -> LED channel should be ON
//   false -> LED channel should be OFF
static void setRgb(bool r, bool g, bool b)
{
  writeRgbPin(PIN_LED_R, r);   // control red channel
  writeRgbPin(PIN_LED_G, g);   // control green channel
  writeRgbPin(PIN_LED_B, b);   // control blue channel
}

static void ledOff()   { setRgb(false, false, false); }
static void ledBlue()  { setRgb(false, false, true);  }
static void ledGreen() { setRgb(false, true,  false); }
static void ledRed()   { setRgb(true,  false, false); }

// ===================== BUZZER =====================

// State variables controlling the buzzer
static bool     g_buzzActive   = false;  // true if the buzzer pattern is running
static bool     g_buzzOn       = false;  // current buzzer output state (ON/OFF)
static uint32_t g_buzzToggleMs = 0;      // last time the buzzer state changed

// Duration of buzzer ON and OFF phases (milliseconds)
static const uint32_t BUZZ_ON_MS  = 120;
static const uint32_t BUZZ_OFF_MS = 380;

// ===================== MPU REGISTERS =====================
// the registers for the communication with the MPU-6050 sensor are documented under:
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
static const uint8_t MPU_ADDR         = 0x68; // This is the I²C device address of the MPU-6050
static const uint8_t REG_PWR_MGMT_1   = 0x6B; // This register addresses for power management of the MPU-6050
static const uint8_t REG_ACCEL_XOUT_H = 0x3B; // This is the address of the first byte of the accelerometer X-axis measurement

// ===================== CONFIG =====================
static const int MAX_UIDS = 20; // maximal number of known IFIDs allowed

struct Config {
  String  ssid;       // the name of the WiFi
  String  pass;       // WiFi password
  String  smsUser;    // Username for the SMS REST-API
  String  smsKey;     // API key  for the SMS REST-API
  String  smsNum;     // the telephone number to send the SMS to
  int     mpuThreshold = 3000;  // the sensitivity of the gyro module
  uint8_t uidCount = 0;  // how many RFIDs are in the config
  uint8_t uidLen[MAX_UIDS]       = { 0 }; // a array keeping the length of each RFID
  uint8_t uidBytes[MAX_UIDS][10] = { { 0 } }; // the RFID's bytes
};

Config cfg;

// Forward declarations (prevents Arduino IDE auto-prototype bug with structs)
static bool loadConfig(Config& c);
static void printConfig(const Config& c);

// ===================== ARM STATE =====================
static bool g_armed = false;

// ===================== WIFI STATE =====================
static bool     g_wifiConnecting   = false;
static uint32_t g_wifiConnectStart = 0;
static uint32_t g_wifiRetryAfter   = 0;
static bool     g_armedSmsSent     = false;  // send once per arm cycle
static bool     g_motionSmsSent    = false;  // send once per arm cycle
static bool     g_motionDetected   = false;  // latched on first motion
static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const uint32_t WIFI_RETRY_MS   = 5000;

// ===================== MPU STATE =====================
static bool    g_mpuReady = false;
static float   g_baseline = 16384.0f;
static float   g_filtered = 16384.0f;

static const float    EMA_ALPHA         = 0.15f;
static const uint32_t MOTION_CONFIRM_MS = 150;
static bool     g_motionCandidate = false;
static uint32_t g_motionStartMs   = 0;

static uint16_t       g_failStreak     = 0;
static uint32_t       g_lastRecoverMs  = 0;
// I²C sensors sometimes stop responding
// this is the maximal allow failures before we reinit the sensor 
static const uint16_t FAIL_TO_RECOVER  = 25; 
static const uint32_t RECOVER_COOLDOWN = 2000;

static bool     g_calibrating       = false;
static uint32_t g_calibStartMs      = 0;
static uint32_t g_lastCalibSampleMs = 0;
static float    g_calibSum          = 0.0f;
static int      g_calibCount        = 0;
static const uint32_t CALIB_DURATION_MS = 400;
static const uint32_t CALIB_SAMPLE_MS   = 10;

// ===================== RFID =====================
MFRC522 rfid(PIN_RFID_SDA, PIN_RFID_RST);

static uint32_t       g_lastRfidMs     = 0;
static const uint32_t RFID_DEBOUNCE_MS = 800;

// ============== HELPER FUNCTIONS =================

// ================== BUZZER ======================
// Stop the buzzer and reset its state
static void buzzerStop() {
  g_buzzActive = false;        // disable buzzer pattern
  g_buzzOn     = false;        // ensure buzzer is OFF
  digitalWrite(PIN_BUZZ, LOW); // turn buzzer hardware OFF
}

// Start the buzzer pattern
static void buzzerStart() {
  g_buzzActive   = true;       // enable buzzer pattern
  g_buzzOn       = false;      // start with buzzer OFF
  g_buzzToggleMs = millis();   // record current time
  digitalWrite(PIN_BUZZ, LOW); // ensure buzzer starts OFF
}

// Handle the buzzer pattern without blocking the program
// This function must be called repeatedly inside loop()
// This function track the time and decide if the buzzer
// "needs" to be turned on or off
static void buzzerHandle() {

  if (!g_buzzActive) return;   // do nothing if buzzer is disabled

  uint32_t now = millis();     // current time in milliseconds

  // Choose the current phase duration (ON or OFF)
  uint32_t period = g_buzzOn ? BUZZ_ON_MS : BUZZ_OFF_MS;

  // Check if it is time to toggle the buzzer state
  if (now - g_buzzToggleMs >= period) {

    g_buzzToggleMs = now;      // remember the toggle time
    g_buzzOn       = !g_buzzOn;// switch ON <-> OFF

    // Apply the new state to the buzzer pin
    digitalWrite(PIN_BUZZ, g_buzzOn ? HIGH : LOW);
  }
}

// ============ CONFIG HELPERS ======================
// Extract the text that appears after the first ':' in a line.
// Example: "UID: AA BB CC"  ->  "AA BB CC"
//          "PASSWORD: myUs3rNam3" -> "myUs3rNam3"
static String valueAfterColon(const String& line) {
  int idx = line.indexOf(':');        // find position of ':'
  if (idx < 0) return "";             // if not found, return empty string

  String v = line.substring(idx + 1); // take everything after ':'
  v.trim();                           // remove leading/trailing spaces
  return v;
}

// Parse the hexadecimal UID string of the RFID into a byte array.
// Example input: "AA BB CC DD"
// Output: [0xAA, 0xBB, 0xCC, 0xDD]
static bool parseHexUid(const String& s, uint8_t* out, uint8_t& outLen) {
  outLen = 0;
  String t = s; // copy the input string
  t.trim();     // remove leading/trailing spaces
  int i = 0;    // current parsing position

  // Loop through the string token by token
  while (i < (int)t.length()) {
    // Skip spaces between tokens
    while (i < (int)t.length() && t[i] == ' ') i++;

    if (i >= (int)t.length()) break;

    int j = i;

    // Find end of current token
    while (j < (int)t.length() && t[j] != ' ') j++;

    // extract token (e.g. "AA")
    String tok = t.substring(i, j);
    tok.trim();

    // skip empty tokens
    if (tok.length() == 0) {
      i = j + 1;
      continue;
    }

    // hex byte must be max 2 chars
    if (tok.length() > 2)  return false;

    // Convert hex string to integer
    char* endptr = nullptr;
    long v = strtol(tok.c_str(), &endptr, 16);

    // Validate conversion
    if (endptr == tok.c_str() || v < 0 || v > 255) return false;
    if (outLen >= 10) return false;

    // store parsed byte
    out[outLen++] = (uint8_t)v;

    // move to next token
    i = j + 1;
  }
  return outLen > 0; // success if at least one byte parsed
}

// ===================== SD CONFIG LOAD =====================
// Load configuration values from /config.txt file on the SD card
static bool loadConfig(Config& c) {

  c = Config();  // reset configuration to default values

  // the SPI bus is shared by the SD card and the RFID modules
  // We must deselect both devices before starting communication.
  pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH);
  pinMode(PIN_RFID_SDA, OUTPUT); digitalWrite(PIN_RFID_SDA, HIGH);

  Serial.println("SD: mounting...");

  // Initialize the SD card
  if (!SD.begin(PIN_SD_CS, SPI, 4000000)) {
    Serial.println("SD: mount FAILED");
    return false;
  }

  Serial.println("SD: mounted OK");

  // Open configuration file
  File f = SD.open("/config.txt", FILE_READ);
  if (!f) {
    Serial.println("SD: /config.txt not found");
    SD.end();
    return false;
  }

  // Read the file line by line
  while (f.available()) {

    String line = f.readStringUntil('\n'); // read one line
    line.trim();                           // remove leading/trailing whitespace

    // Skip empty lines and comments
    if (line.length() == 0 || line.startsWith("#"))
      continue;

    // Parse configuration keys
    if      (line.startsWith("SSID:"))         c.ssid         = valueAfterColon(line);
    else if (line.startsWith("PASS:"))         c.pass         = valueAfterColon(line);
    else if (line.startsWith("SMS_USER:"))     c.smsUser      = valueAfterColon(line);
    else if (line.startsWith("SMS_KEY:"))      c.smsKey       = valueAfterColon(line);
    else if (line.startsWith("SMS_NUM:"))      c.smsNum       = valueAfterColon(line);
    else if (line.startsWith("MPUThreshold:")) c.mpuThreshold = valueAfterColon(line).toInt();

    // Parse RFID UID entries
    else if (line.startsWith("RFID:")) {

      // Ignore additional UIDs if storage limit is reached
      if (c.uidCount >= MAX_UIDS)
        continue;

      String uidStr = valueAfterColon(line);

      uint8_t len = 0;
      uint8_t bytes[10] = { 0 };

      // Convert hex UID string into byte values
      if (parseHexUid(uidStr, bytes, len)) {

        c.uidLen[c.uidCount] = len;

        // Copy UID bytes into configuration structure
        for (uint8_t i = 0; i < len; i++)
          c.uidBytes[c.uidCount][i] = bytes[i];

        c.uidCount++; // store next UID in next slot

      } else {
        Serial.print("Config: invalid RFID line: ");
        Serial.println(uidStr);
      }
    }

    // Any unknown line is ignored but reported
    else {
      Serial.print("Config: unknown line ignored: ");
      Serial.println(line);
    }
  }

  f.close();   // close file
  SD.end();    // unmount SD card

  Serial.println("SD: unmounted");

  return true; // configuration loaded successfully
}

// ===================== PRINT CONFIG =====================
// This is only for debug
static void printConfig(const Config& c) {
  Serial.println("---------- Config ----------");
  Serial.print("SSID:          "); Serial.println(c.ssid);
  Serial.print("PASS:          "); Serial.println(c.pass);
  Serial.print("SMS_USER:      "); Serial.println(c.smsUser);
  Serial.print("SMS_KEY:       "); Serial.println(c.smsKey);
  Serial.print("SMS_NUM:       "); Serial.println(c.smsNum);
  Serial.print("MPUThreshold:  "); Serial.println(c.mpuThreshold);
  Serial.print("RFID count:    "); Serial.println(c.uidCount);
  for (uint8_t i = 0; i < c.uidCount; i++) {
    Serial.print("  RFID["); Serial.print(i); Serial.print("]: ");
    for (uint8_t j = 0; j < c.uidLen[i]; j++) {
      if (c.uidBytes[i][j] < 0x10) Serial.print("0");
      Serial.print(c.uidBytes[i][j], HEX);
      if (j + 1 < c.uidLen[i]) Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("----------------------------");
}

// ===================== MPU =====================
// Write one byte to a register inside the MPU sensor.
// Returns true if the I²C transmission succeeded.
// I²C is a two-wire communication bus (SDA + SCL) used
// by microcontrollers to talk to sensors and other devices.
static bool mpuWriteByte(uint8_t reg, uint8_t value) {
  // Start communication with the MPU device on the I²C bus
  Wire.beginTransmission(MPU_ADDR);
  // Select the register we want to write to
  Wire.write(reg);
  // Send the value that should be stored in that register
  Wire.write(value);
  // Finish the transmission.
  // endTransmission() returns 0 if the device acknowledged the write.
  return (Wire.endTransmission() == 0);
}

// Read the 3-axis acceleration values from the MPU sensor.
// The sensor stores each axis as a 16-bit value (high byte + low byte).
// Returns true if the read was successful.
static bool mpuReadAccel(int16_t& ax, int16_t& ay, int16_t& az) {

  // Tell the MPU which register we want to start reading from
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);

  // Send the register address but keep the I²C connection active
  if (Wire.endTransmission(false) != 0) return false;

  // Request 6 bytes: X_H, X_L, Y_H, Y_L, Z_H, Z_L
  if (Wire.requestFrom((int)MPU_ADDR, 6, true) != 6) return false;

  // Combine high and low bytes to form signed 16-bit acceleration values
  ax = (int16_t)(Wire.read() << 8 | Wire.read()); // X axis
  ay = (int16_t)(Wire.read() << 8 | Wire.read()); // Y axis
  az = (int16_t)(Wire.read() << 8 | Wire.read()); // Z axis

  return true; // reading successful
}

// Compute the magnitude (total strength) of the acceleration vector.
//
// The MPU returns acceleration in three directions:
//   ax = acceleration along X axis
//   ay = acceleration along Y axis
//   az = acceleration along Z axis
//
// The magnitude combines these three values into one number using
// the vector length formula: sqrt(x² + y² + z²).
static float magnitude(int16_t ax, int16_t ay, int16_t az) {
  // Convert integer readings to floating-point values
  float x = ax, y = ay, z = az;
  // Calculate the vector length
  return sqrtf(x*x + y*y + z*z);
}

// Initialize the MPU-6050 sensor.
// The register REG_PWR_MGMT_1 controls the sensor's power state.
// Writing 0x00 wakes the sensor up from sleep mode and enables measurements.
// The function returns true if the write operation succeeded.
static bool mpuInit() {
  return mpuWriteByte(REG_PWR_MGMT_1, 0x00);
}

// I²C sensors sometimes stop responding, so implements a self-healing
// mechanism for the sensor communication
// This function tries to recover the MPU sensor if repeated communication failures occur.
// This function restarts the I²C bus and reinitializes the sensor.
static void mpuRecoverIfNeeded() {

  // Do nothing if the MPU was never initialized successfully
  if (!g_mpuReady) return;

  // Only attempt recovery after a certain number of failures
  if (g_failStreak < FAIL_TO_RECOVER) return;

  uint32_t now = millis();

  // Prevent recovery attempts from happening too frequently
  if (now - g_lastRecoverMs < RECOVER_COOLDOWN) return;

  g_lastRecoverMs = now;  // remember when the last recovery happened
  g_failStreak    = 0;    // reset failure counter

  Serial.println("MPU: recovering...");

  // Restart the I²C bus
  Wire.end();
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);

  // Restore I²C settings
  Wire.setClock(400000);  // use 400 kHz fast mode
  Wire.setTimeout(5);     // prevent long blocking if bus hangs

  // Reinitialize the MPU sensor
  g_mpuReady = mpuInit();

  Serial.println(g_mpuReady ? "MPU: recover OK" : "MPU: recover FAILED");
}

// Start the calibration process for the MPU motion sensor.
// During calibration the system measures the "normal" acceleration level
// (for example when the device is not moving).
// This baseline is later used to detect unusual movement.
static void mpuStartCalibration() {

  g_calibrating       = true;      // indicate that calibration is in progress
  g_calibStartMs      = millis();  // remember when calibration started
  g_lastCalibSampleMs = 0;         // reset time of last calibration sample
  g_calibSum          = 0.0f;      // reset accumulated acceleration values
  g_calibCount        = 0;         // reset number of collected samples
  g_motionCandidate   = false;     // clear any previous motion detection state

  Serial.println("MPU: calibration started...");
}

// Handle the calibration process for the MPU motion sensor.
// During calibration we collect several acceleration samples and compute
// an average value ("baseline"). This baseline represents normal movement
// when the device is not moving.
static void mpuHandleCalibration() {

  // Do nothing if calibration is not active
  if (!g_calibrating) return;

  uint32_t now = millis();

  // Take a new sample at regular time intervals if
  // this the first calibration sample, or has enough time passed since the last sample
  if (g_lastCalibSampleMs == 0 || (now - g_lastCalibSampleMs >= CALIB_SAMPLE_MS)) {

    g_lastCalibSampleMs = now; // remember time of this sample

    int16_t ax, ay, az;

    // Try to read accelerometer values
    if (mpuReadAccel(ax, ay, az)) {

      g_failStreak = 0; // reset communication failure counter

      // Add the magnitude of this sample to the running sum
      g_calibSum += magnitude(ax, ay, az);

      // Count how many samples we collected
      g_calibCount++;

    } else {

      // Reading failed → increase failure counter
      if (g_failStreak < 0xFFFF) g_failStreak++;

      // Try to recover the MPU if too many failures occur
      mpuRecoverIfNeeded();
    }
  }

  // Stop calibration after the configured duration
  if (now - g_calibStartMs >= CALIB_DURATION_MS) {

    // Compute the average acceleration (baseline)
    if (g_calibCount > 0)
      g_baseline = g_calibSum / g_calibCount;

    // Initialize filtered value with baseline
    g_filtered = g_baseline;

    // Calibration finished
    g_calibrating = false;

    // Print calibration result
    Serial.print("MPU: baseline = ");
    Serial.print(g_baseline, 1);
    Serial.print("  threshold = ");
    Serial.println(cfg.mpuThreshold);
    Serial.println("MPU: monitoring for motion...");
  }
}

// Check if significant motion has occurred based on MPU accelerometer data.
// The function filters the raw acceleration signal and compares it to the
// previously measured baseline. If the deviation is large enough and persists
// long enough, motion is confirmed.
static bool mpuCheckMotion() {

  int16_t ax, ay, az;

  // Try to read accelerometer values from the MPU
  if (!mpuReadAccel(ax, ay, az)) {

    // If reading fails, increase the failure counter
    if (g_failStreak < 0xFFFF) g_failStreak++;

    // Attempt to recover the sensor if failures accumulate
    mpuRecoverIfNeeded();

    return false; // no motion detection possible
  }

  // Successful read → reset failure counter
  g_failStreak = 0;

  // Calculate magnitude of acceleration vector
  float raw = magnitude(ax, ay, az);

  // Apply exponential moving average (EMA) filter to smooth the signal
  g_filtered += EMA_ALPHA * (raw - g_filtered);

  // Compute deviation from the baseline (normal resting value)
  float delta = fabsf(g_filtered - g_baseline);

  // Check if the deviation exceeds the configured motion threshold
  if (delta > (float)cfg.mpuThreshold) {

    // First detection of possible motion
    if (!g_motionCandidate) {

      g_motionCandidate = true;
      g_motionStartMs   = millis(); // remember when motion started

    }
    // Motion continues long enough → confirm motion
    else if (millis() - g_motionStartMs >= MOTION_CONFIRM_MS) {

      g_motionCandidate = false;
      return true; // motion detected
    }

  } else {

    // Signal returned to normal → cancel motion candidate
    g_motionCandidate = false;
  }

  return false; // no confirmed motion
}

// ===================== NTP =====================
// Since the ESP32 doesn't have a real-time clock with a battery and so it does
// not know the current date and time after boot. So we fetch the time through NTP.
// Fetch current time string via NTP. Returns "HH:MM:SS" or "time unknown".
// Obtain the current time from internet NTP servers and return it as a string.
// If the time cannot be retrieved within a few seconds, return "time unknown".
static String getTimeString() {

  // Configure the ESP32 time system to use NTP servers
  // (offsets are 0 because we return time in UTC)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  struct tm ti;   // structure that will hold the date and time

  // Wait up to 3 seconds for the NTP time to become available
  uint32_t start = millis();

  while (!getLocalTime(&ti) && millis() - start < 3000)
    delay(100);   // small delay to avoid busy looping

  // If time is still unavailable, return a fallback message
  if (!getLocalTime(&ti))
    return "time unknown";

  char buf[32];

  // Format the time into a human-readable string
  // Example: "2026-03-03 15:42:18 UTC"
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &ti);

  // Convert the C string into an Arduino String and return it
  return String(buf);
}

// ===================== SMS =====================
// We are using SendClick for sending SMS notifications.
// Thsi function sends an SMS message using the ClickSend REST API.
// Returns true if the message was sent successfully.
static bool sendSms(const String& body) {

  // Check that required SMS configuration values exist
  if (cfg.smsUser.isEmpty() || cfg.smsKey.isEmpty() || cfg.smsNum.isEmpty()) {
    Serial.println("SMS: config missing, skipping");
    return false;
  }

  // Create HTTP Basic Authentication header (username:key encoded as Base64)
  String auth = "Basic " + base64Encode(cfg.smsUser + ":" + cfg.smsKey);

  // Escape characters that would break the JSON payload
  String safeBody = body;
  safeBody.replace("\"", "\\\""); // escape quotation marks
  safeBody.replace("\n", "\\n");  // escape newlines

  // Build JSON request body for the ClickSend API
  String payload = "{\"messages\":[{\"source\":\"esp32\","
                   "\"body\":\"" + safeBody + "\","
                   "\"to\":\"" + cfg.smsNum + "\"}]}";

  // Create HTTP client and open connection to the SMS API
  HTTPClient http;
  http.begin("https://rest.clicksend.com/v3/sms/send");

  // Set HTTP headers
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", auth);

  // Send POST request containing the SMS message
  int code = http.POST(payload);

  // Read server response
  String resp = http.getString();

  // Close the HTTP connection
  http.end();

  // Check if the request succeeded
  if (code == 200) {
    Serial.println("SMS: sent OK");
    return true;
  }

  // Print error information if the request failed
  Serial.printf("SMS: failed  HTTP=%d  %s\n", code, resp.c_str());
  return false;
}

// ===================== WIFI =====================
// Start connecting the ESP32 to the configured Wi-Fi network
static void wifiStart() {

  // Check that an SSID was provided in the configuration
  if (cfg.ssid.length() == 0) {
    Serial.println("WiFi: SSID missing in config");
    return;
  }

  // Do not store Wi-Fi credentials in flash memory
  WiFi.persistent(false);

  // Use station mode (connect to an existing Wi-Fi network)
  WiFi.mode(WIFI_STA);

  // Disable Wi-Fi power saving to keep the connection responsive
  WiFi.setSleep(false);

  // Ensure any previous connection is cleared
  WiFi.disconnect(true, true);

  Serial.print("WiFi: connecting to ");
  Serial.println(cfg.ssid);

  // Start the Wi-Fi connection using SSID and password
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  // Remember that a connection attempt is in progress
  g_wifiConnecting   = true;

  // Store the time when the connection attempt started
  g_wifiConnectStart = millis();
}


// Stop Wi-Fi completely
static void wifiStop() {

  // Disconnect from the current network
  WiFi.disconnect(true, true);

  // Turn off the Wi-Fi hardware
  WiFi.mode(WIFI_OFF);

  // Reset connection state variables
  g_wifiConnecting = false;
  g_wifiRetryAfter = 0;

  Serial.println("WiFi: disconnected");
}

// We will call the function whenever the ESP32 is successfully connects to Wi-Fi.
// Prints connection details, sets the status LED, and sends
// an SMS notification the first time the device becomes armed.
static void onWifiConnected() {

  // Get current network information
  String ip   = WiFi.localIP().toString();     // IP address assigned by router
  String rssi = String(WiFi.RSSI()) + " dBm";  // signal strength
  String time = getTimeString();               // current time (from NTP)

  // Print connection information to the serial monitor
  Serial.println("WiFi: connected!");
  Serial.print("  IP:   "); Serial.println(ip);
  Serial.print("  SSID: "); Serial.println(cfg.ssid);
  Serial.print("  RSSI: "); Serial.println(rssi);
  Serial.print("  Time: "); Serial.println(time);

  // Turn LED green to indicate the device is online
  ledGreen();

  // Send the "device armed" SMS only once after connection
  if (!g_armedSmsSent) {

    // Build the SMS message body
    String body = "Device armed\n"
                  "Time: " + time + "\n"
                  "WiFi: " + cfg.ssid + "\n"
                  "IP: "   + ip + "\n"
                  "RSSI: " + rssi;

    // Send SMS notification
    sendSms(body);

    // Prevent sending the same message again
    g_armedSmsSent = true;
  }
}

// This function manages the Wi-Fi connection state.
// We call it every loop() while armed.
// It handles connection attempts, timeouts, retries,
// and the interaction between Wi-Fi status and the alarm buzzer.
static void wifiHandle() {

  uint32_t now = millis();  // current time (used for timeouts)

  // Case 1: Wi-Fi was connecting and is now successfully connected
  if (g_wifiConnecting && WiFi.status() == WL_CONNECTED) {

    g_wifiConnecting = false;   // connection process finished

    onWifiConnected();          // print info and send "armed" SMS

    // If motion had already triggered while Wi-Fi was down,
    // stop the buzzer now that connectivity is restored.
    if (g_motionDetected)
      buzzerStop();

    return;
  }

  // Case 2: Already connected and everything is fine
  if (!g_wifiConnecting && WiFi.status() == WL_CONNECTED)
    return;

  // Case 3: Wi-Fi lost after motion was detected
  // Start the buzzer to signal a problem
  if (g_motionDetected && !g_buzzActive) {
    Serial.println("WiFi lost after motion -> buzzer ON");
    buzzerStart();
  }

  // Case 4: Waiting before attempting the next reconnect
  if (!g_wifiConnecting && now < g_wifiRetryAfter)
    return;

  // Case 5: Start a new Wi-Fi connection attempt
  if (!g_wifiConnecting) {
    wifiStart();
    return;
  }

  // Case 6: Connection attempt took too long (timeout)
  if (now - g_wifiConnectStart >= WIFI_TIMEOUT_MS) {

    Serial.println("WiFi: connect timeout, will retry...");

    // Abort the current attempt
    WiFi.disconnect(true, true);

    g_wifiConnecting = false;

    // Schedule the next retry after a delay
    g_wifiRetryAfter = now + WIFI_RETRY_MS;
  }
}

// ===================== RFID =====================
// Convert an RFID UID into a human-readable string.
// Example output: "04 A3 7F 1B"
static String uidToString(const MFRC522::Uid& uid) {

  String s;  // result string

  // Loop through all bytes of the UID
  for (byte i = 0; i < uid.size; i++) {

    // Add a leading zero for values smaller than 0x10
    // so each byte always has two hex digits
    if (uid.uidByte[i] < 0x10) s += "0";

    // Convert the byte to hexadecimal text
    s += String(uid.uidByte[i], HEX);

    // Add a space between bytes (except after the last one)
    if (i + 1 < uid.size) s += " ";
  }

  // Convert letters A-F to uppercase for consistency
  s.toUpperCase();

  return s;  // return formatted UID string
}

// Check whether the scanned RFID UID is authorized.
// Returns true if the UID matches one stored in the configuration.
static bool uidAuthorized(const MFRC522::Uid& uid) {

  // If no UIDs are configured, allow any card (open access mode)
  if (cfg.uidCount == 0)
    return true;

  // Compare the scanned UID with each stored UID
  for (uint8_t i = 0; i < cfg.uidCount; i++) {

    // If the UID lengths differ, they cannot match
    if (cfg.uidLen[i] != uid.size)
      continue;

    bool match = true;

    // Compare each byte of the UID
    for (byte j = 0; j < uid.size; j++) {
      if (cfg.uidBytes[i][j] != uid.uidByte[j]) {
        match = false;  // mismatch found
        break;
      }
    }

    // If all bytes matched, the UID is authorized
    if (match)
      return true;
  }

  // No matching UID found → access denied
  return false;
}

// Handle RFID card scanning and arm/disarm the device.
// We call this function repeatedly in loop().
static void handleRfid() {

  uint32_t now = millis();

  // Ignore scans that happen too quickly (debounce protection)
  if (now - g_lastRfidMs < RFID_DEBOUNCE_MS)
    return;

  // Check if a new RFID card is present
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Read the card's UID
  if (!rfid.PICC_ReadCardSerial())
    return;

  g_lastRfidMs = now;  // remember the time of this scan

  // Convert UID to a readable string and check authorization
  String uidStr = uidToString(rfid.uid);
  bool   ok     = uidAuthorized(rfid.uid);

  // Print the scanned UID and its authorization status
  Serial.print("RFID: ");
  Serial.print(uidStr);
  Serial.println(ok ? "  -> authorized" : "  -> UNAUTHORIZED");

  // Properly stop communication with the card
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // Ignore unauthorized cards
  if (!ok)
    return;

  // If the system is currently disarmed → arm it
  if (!g_armed) {

    g_armed          = true;   // system is now armed
    g_armedSmsSent   = false;  // allow "armed" SMS to be sent
    g_motionSmsSent  = false;  // reset motion alert state
    g_motionDetected = false;

    Serial.println("*** ARMED ***");

    ledBlue();                 // blue LED indicates armed state

    wifiStart();               // start Wi-Fi connection

    // Start motion sensor calibration if MPU is ready
    if (g_mpuReady)
      mpuStartCalibration();

  }
  // If the system is currently armed → disarm it
  else {

    g_armed = false;

    g_motionCandidate = false; // cancel any pending motion detection

    wifiStop();                // disconnect Wi-Fi
    buzzerStop();              // stop alarm buzzer
    ledOff();                  // turn off status LED

    Serial.println("*** DISARMED ***");
  }
}

// ===================== SETUP / LOOP =====================
// Here we initialize pins, SPI/I²C buses, load config from SD,
// and initialize the RFID reader and MPU motion sensor.
void setup() {

  // Start serial output for debugging (Serial Monitor)
  Serial.begin(115200);
  delay(200); // short delay so the serial monitor can attach

  // SPI bus is shared (SD + RFID), so deselect both devices first
  pinMode(PIN_SD_CS,    OUTPUT); digitalWrite(PIN_SD_CS,    HIGH); // SD not selected
  pinMode(PIN_RFID_SDA, OUTPUT); digitalWrite(PIN_RFID_SDA, HIGH); // RFID not selected (RC522 uses SDA pin as CS)

  // Configure RGB LED pins as outputs and turn LED off
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledOff();

  // Configure buzzer pin as output and make sure it starts OFF
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  // Initialize the SPI bus (SCK, MISO, MOSI, SS/CS pin)
  // Note: the last argument is the "default" CS pin for SPI
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RFID_SDA);

  // Load configuration from SD card (/config.txt)
  bool ok = loadConfig(cfg);
  Serial.println(ok ? "Config load: OK" : "Config load: FAILED");

  // If config loaded, print it for debugging
  if (ok)
    printConfig(cfg);

  // Initialize RFID reader (MFRC522)
  Serial.println("RFID: init...");
  rfid.PCD_Init();                 // start the RFID reader
  delay(50);                       // small delay for the chip to settle
  rfid.PCD_DumpVersionToSerial();  // print reader version info
  Serial.println("RFID: ready");

  // Initialize MPU motion sensor (MPU-6050) over I²C
  Serial.println("MPU: init...");
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL); // start I²C with custom SDA/SCL pins
  Wire.setClock(400000);               // 400 kHz I²C (fast mode)
  Wire.setTimeout(5);                  // avoid long blocking if I²C gets stuck
  g_mpuReady = mpuInit();              // wake up the MPU sensor
  Serial.println(g_mpuReady ? "MPU: init OK" : "MPU: init FAILED");

  // User instruction shown on boot
  Serial.println("Disarmed. Present authorized tag to arm.");
}

// In our loop we poll RFID, then (if armed) manage Wi-Fi/buzzer
// and check for motion.
void loop() {

  // Always check RFID so the user can arm/disarm at any time
  handleRfid();

  // If not armed, do nothing else (saves power and avoids false alarms)
  if (!g_armed)
    return;

  // Maintain Wi-Fi connection state (connect/retry/timeout)
  wifiHandle();

  // Drive the non-blocking buzzer beep pattern (if active)
  buzzerHandle();

  // If the motion sensor is not ready, we cannot detect motion
  if (!g_mpuReady)
    return;

  // During calibration we only collect baseline samples
  if (g_calibrating) {
    mpuHandleCalibration();
    return; // do not run motion detection until calibration is finished
  }

  // Check if motion is confirmed (threshold exceeded long enough)
  if (mpuCheckMotion()) {

    // Print debug details about the detected motion
    Serial.print("MOTION DETECTED!  filtered=");
    Serial.print(g_filtered, 1);
    Serial.print("  baseline=");
    Serial.print(g_baseline, 1);
    Serial.print("  delta=");
    Serial.print(fabsf(g_filtered - g_baseline), 1);
    Serial.print("  threshold=");
    Serial.println(cfg.mpuThreshold);

    // Send the SMS alert only once per arming cycle
    if (!g_motionSmsSent) {

      g_motionDetected = true; // remember that motion already happened
      ledRed();                // red LED indicates alarm state

      // Build the SMS message with time + network info
      String body = "Bike alarm: motion detected\n"
                    "Time: "  + getTimeString() + "\n"
                    "WiFi: "  + cfg.ssid + "\n"
                    "IP: "    + WiFi.localIP().toString() + "\n"
                    "RSSI: "  + String(WiFi.RSSI()) + " dBm";

      sendSms(body);           // send alert to phone
      g_motionSmsSent = true;  // prevent repeated alerts
    }
  }
}
