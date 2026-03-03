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

static const bool RGB_COMMON_ANODE   = true;  // set false for common cathode

static void setRgb(bool r, bool g, bool b) {
  auto w = [](int pin, bool on) {
    digitalWrite(pin, (RGB_COMMON_ANODE ? !on : on) ? HIGH : LOW);
  };
  w(PIN_LED_R, r);
  w(PIN_LED_G, g);
  w(PIN_LED_B, b);
}

static void ledOff()   { setRgb(false, false, false); }
static void ledBlue()  { setRgb(false, false, true);  }
static void ledGreen() { setRgb(false, true,  false); }
static void ledRed()   { setRgb(true,  false, false); }

// ===================== BUZZER =====================

static bool     g_buzzActive   = false;
static bool     g_buzzOn       = false;
static uint32_t g_buzzToggleMs = 0;
static const uint32_t BUZZ_ON_MS  = 120;
static const uint32_t BUZZ_OFF_MS = 380;

static void buzzerStop() {
  g_buzzActive = false;
  g_buzzOn     = false;
  digitalWrite(PIN_BUZZ, LOW);
}

static void buzzerStart() {
  g_buzzActive   = true;
  g_buzzOn       = false;
  g_buzzToggleMs = millis();
  digitalWrite(PIN_BUZZ, LOW);
}

// Non-blocking beep pattern — call every loop()
static void buzzerHandle() {
  if (!g_buzzActive) return;
  uint32_t now    = millis();
  uint32_t period = g_buzzOn ? BUZZ_ON_MS : BUZZ_OFF_MS;
  if (now - g_buzzToggleMs >= period) {
    g_buzzToggleMs = now;
    g_buzzOn       = !g_buzzOn;
    digitalWrite(PIN_BUZZ, g_buzzOn ? HIGH : LOW);
  }
}

// ===================== MPU REGISTERS =====================
static const uint8_t MPU_ADDR          = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ===================== CONFIG =====================
static const int MAX_UIDS = 20;

struct Config {
  String  ssid;
  String  pass;
  String  smsUser;
  String  smsKey;
  String  smsNum;
  int     mpuThreshold = 3000;
  uint8_t uidCount = 0;
  uint8_t uidLen[MAX_UIDS]       = { 0 };
  uint8_t uidBytes[MAX_UIDS][10] = { { 0 } };
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

// ===================== HELPERS =====================
static String valueAfterColon(const String& line) {
  int idx = line.indexOf(':');
  if (idx < 0) return "";
  String v = line.substring(idx + 1);
  v.trim();
  return v;
}

static bool parseHexUid(const String& s, uint8_t* out, uint8_t& outLen) {
  outLen = 0;
  String t = s;
  t.trim();
  int i = 0;
  while (i < (int)t.length()) {
    while (i < (int)t.length() && t[i] == ' ') i++;
    if (i >= (int)t.length()) break;
    int j = i;
    while (j < (int)t.length() && t[j] != ' ') j++;
    String tok = t.substring(i, j);
    tok.trim();
    if (tok.length() == 0) { i = j + 1; continue; }
    if (tok.length() > 2)  return false;
    char* endptr = nullptr;
    long v = strtol(tok.c_str(), &endptr, 16);
    if (endptr == tok.c_str() || v < 0 || v > 255) return false;
    if (outLen >= 10) return false;
    out[outLen++] = (uint8_t)v;
    i = j + 1;
  }
  return outLen > 0;
}

// ===================== SD CONFIG LOAD =====================
static bool loadConfig(Config& c) {
  c = Config();

  // because SPI is shared, deselect both SPI devices first
  pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
  pinMode(PIN_RFID_SDA, OUTPUT); digitalWrite(PIN_RFID_SDA, HIGH);

  Serial.println("SD: mounting...");
  if (!SD.begin(PIN_SD_CS, SPI, 4000000)) {
    Serial.println("SD: mount FAILED");
    return false;
  }
  Serial.println("SD: mounted OK");

  File f = SD.open("/config.txt", FILE_READ);
  if (!f) {
    Serial.println("SD: /config.txt not found");
    SD.end();
    return false;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;

    if      (line.startsWith("SSID:"))         c.ssid         = valueAfterColon(line);
    else if (line.startsWith("PASS:"))         c.pass         = valueAfterColon(line);
    else if (line.startsWith("SMS_USER:"))     c.smsUser      = valueAfterColon(line);
    else if (line.startsWith("SMS_KEY:"))      c.smsKey       = valueAfterColon(line);
    else if (line.startsWith("SMS_NUM:"))      c.smsNum       = valueAfterColon(line);
    else if (line.startsWith("MPUThreshold:")) c.mpuThreshold = valueAfterColon(line).toInt();
    else if (line.startsWith("RFID:")) {
      if (c.uidCount >= MAX_UIDS) continue;
      String uidStr = valueAfterColon(line);
      uint8_t len = 0, bytes[10] = { 0 };
      if (parseHexUid(uidStr, bytes, len)) {
        c.uidLen[c.uidCount] = len;
        for (uint8_t i = 0; i < len; i++) c.uidBytes[c.uidCount][i] = bytes[i];
        c.uidCount++;
      } else {
        Serial.print("Config: invalid RFID line: ");
        Serial.println(uidStr);
      }
    } else {
      Serial.print("Config: unknown line ignored: ");
      Serial.println(line);
    }
  }

  f.close();
  SD.end();
  Serial.println("SD: unmounted");
  return true;
}

// ===================== PRINT CONFIG =====================
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
static bool mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

static bool mpuReadAccel(int16_t& ax, int16_t& ay, int16_t& az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6, true) != 6) return false;
  ax = (int16_t)(Wire.read() << 8 | Wire.read());
  ay = (int16_t)(Wire.read() << 8 | Wire.read());
  az = (int16_t)(Wire.read() << 8 | Wire.read());
  return true;
}

static float magnitude(int16_t ax, int16_t ay, int16_t az) {
  float x = ax, y = ay, z = az;
  return sqrtf(x*x + y*y + z*z);
}

static bool mpuInit() {
  return mpuWriteByte(REG_PWR_MGMT_1, 0x00);
}

static void mpuRecoverIfNeeded() {
  if (!g_mpuReady) return;
  if (g_failStreak < FAIL_TO_RECOVER) return;
  uint32_t now = millis();
  if (now - g_lastRecoverMs < RECOVER_COOLDOWN) return;
  g_lastRecoverMs = now;
  g_failStreak    = 0;
  Serial.println("MPU: recovering...");
  Wire.end();
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(5);
  g_mpuReady = mpuInit();
  Serial.println(g_mpuReady ? "MPU: recover OK" : "MPU: recover FAILED");
}

static void mpuStartCalibration() {
  g_calibrating       = true;
  g_calibStartMs      = millis();
  g_lastCalibSampleMs = 0;
  g_calibSum          = 0.0f;
  g_calibCount        = 0;
  g_motionCandidate   = false;
  Serial.println("MPU: calibration started...");
}

static void mpuHandleCalibration() {
  if (!g_calibrating) return;
  uint32_t now = millis();
  if (g_lastCalibSampleMs == 0 || (now - g_lastCalibSampleMs >= CALIB_SAMPLE_MS)) {
    g_lastCalibSampleMs = now;
    int16_t ax, ay, az;
    if (mpuReadAccel(ax, ay, az)) {
      g_failStreak = 0;
      g_calibSum  += magnitude(ax, ay, az);
      g_calibCount++;
    } else {
      if (g_failStreak < 0xFFFF) g_failStreak++;
      mpuRecoverIfNeeded();
    }
  }
  if (now - g_calibStartMs >= CALIB_DURATION_MS) {
    if (g_calibCount > 0) g_baseline = g_calibSum / g_calibCount;
    g_filtered    = g_baseline;
    g_calibrating = false;
    Serial.print("MPU: baseline = ");
    Serial.print(g_baseline, 1);
    Serial.print("  threshold = ");
    Serial.println(cfg.mpuThreshold);
    Serial.println("MPU: monitoring for motion...");
  }
}

static bool mpuCheckMotion() {
  int16_t ax, ay, az;
  if (!mpuReadAccel(ax, ay, az)) {
    if (g_failStreak < 0xFFFF) g_failStreak++;
    mpuRecoverIfNeeded();
    return false;
  }
  g_failStreak = 0;
  float raw = magnitude(ax, ay, az);
  g_filtered += EMA_ALPHA * (raw - g_filtered);
  float delta = fabsf(g_filtered - g_baseline);
  if (delta > (float)cfg.mpuThreshold) {
    if (!g_motionCandidate) {
      g_motionCandidate = true;
      g_motionStartMs   = millis();
    } else if (millis() - g_motionStartMs >= MOTION_CONFIRM_MS) {
      g_motionCandidate = false;
      return true;
    }
  } else {
    g_motionCandidate = false;
  }
  return false;
}

// ===================== NTP =====================
// Fetch current time string via NTP. Returns "HH:MM:SS" or "time unknown".
static String getTimeString() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm ti;
  // Wait up to 3 seconds for NTP
  uint32_t start = millis();
  while (!getLocalTime(&ti) && millis() - start < 3000) delay(100);
  if (!getLocalTime(&ti)) return "time unknown";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &ti);
  return String(buf);
}

// ===================== SMS =====================
static bool sendSms(const String& body) {
  if (cfg.smsUser.isEmpty() || cfg.smsKey.isEmpty() || cfg.smsNum.isEmpty()) {
    Serial.println("SMS: config missing, skipping");
    return false;
  }

  String auth = "Basic " + base64Encode(cfg.smsUser + ":" + cfg.smsKey);

  String safeBody = body;
  safeBody.replace("\"", "\\\"");
  safeBody.replace("\n", "\\n");

  String payload = "{\"messages\":[{\"source\":\"esp32\","
                   "\"body\":\"" + safeBody + "\","
                   "\"to\":\"" + cfg.smsNum + "\"}]}";

  HTTPClient http;
  http.begin("https://rest.clicksend.com/v3/sms/send");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", auth);

  int code = http.POST(payload);
  String resp = http.getString();
  http.end();

  if (code == 200) {
    Serial.println("SMS: sent OK");
    return true;
  }
  Serial.printf("SMS: failed  HTTP=%d  %s\n", code, resp.c_str());
  return false;
}

// ===================== WIFI =====================
static void wifiStart() {
  if (cfg.ssid.length() == 0) {
    Serial.println("WiFi: SSID missing in config");
    return;
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  Serial.print("WiFi: connecting to ");
  Serial.println(cfg.ssid);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  g_wifiConnecting   = true;
  g_wifiConnectStart = millis();
}

static void wifiStop() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  g_wifiConnecting = false;
  g_wifiRetryAfter = 0;
  Serial.println("WiFi: disconnected");
}

static void onWifiConnected() {
  String ip   = WiFi.localIP().toString();
  String rssi = String(WiFi.RSSI()) + " dBm";
  String time = getTimeString();

  Serial.println("WiFi: connected!");
  Serial.print("  IP:   "); Serial.println(ip);
  Serial.print("  SSID: "); Serial.println(cfg.ssid);
  Serial.print("  RSSI: "); Serial.println(rssi);
  Serial.print("  Time: "); Serial.println(time);

  ledGreen();

  if (!g_armedSmsSent) {
    String body = "Device armed\n"
                  "Time: " + time + "\n"
                  "WiFi: " + cfg.ssid + "\n"
                  "IP: "   + ip + "\n"
                  "RSSI: " + rssi;
    sendSms(body);
    g_armedSmsSent = true;
  }
}

// Call every loop() while armed — handles connect, timeout, retry
static void wifiHandle() {
  uint32_t now = millis();

  // Transition: was connecting, now connected
  if (g_wifiConnecting && WiFi.status() == WL_CONNECTED) {
    g_wifiConnecting = false;
    onWifiConnected();
    if (g_motionDetected) buzzerStop();  // WiFi back after motion — stop buzzer
    return;
  }

  // Already connected and notified — nothing to do
  if (!g_wifiConnecting && WiFi.status() == WL_CONNECTED) return;

  // WiFi lost while motion was already detected — start buzzer
  if (g_motionDetected && !g_buzzActive) {
    Serial.println("WiFi lost after motion -> buzzer ON");
    buzzerStart();
  }

  // Waiting for retry delay
  if (!g_wifiConnecting && now < g_wifiRetryAfter) return;

  // Start a new connection attempt
  if (!g_wifiConnecting) {
    wifiStart();
    return;
  }

  // Timeout — schedule retry
  if (now - g_wifiConnectStart >= WIFI_TIMEOUT_MS) {
    Serial.println("WiFi: connect timeout, will retry...");
    WiFi.disconnect(true, true);
    g_wifiConnecting = false;
    g_wifiRetryAfter = now + WIFI_RETRY_MS;
  }
}

// ===================== RFID =====================
static String uidToString(const MFRC522::Uid& uid) {
  String s;
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
    if (i + 1 < uid.size) s += " ";
  }
  s.toUpperCase();
  return s;
}

static bool uidAuthorized(const MFRC522::Uid& uid) {
  if (cfg.uidCount == 0) return true;
  for (uint8_t i = 0; i < cfg.uidCount; i++) {
    if (cfg.uidLen[i] != uid.size) continue;
    bool match = true;
    for (byte j = 0; j < uid.size; j++) {
      if (cfg.uidBytes[i][j] != uid.uidByte[j]) { match = false; break; }
    }
    if (match) return true;
  }
  return false;
}

static void handleRfid() {
  uint32_t now = millis();
  if (now - g_lastRfidMs < RFID_DEBOUNCE_MS) return;
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  g_lastRfidMs = now;
  String uidStr = uidToString(rfid.uid);
  bool   ok     = uidAuthorized(rfid.uid);

  Serial.print("RFID: ");
  Serial.print(uidStr);
  Serial.println(ok ? "  -> authorized" : "  -> UNAUTHORIZED");

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  if (!ok) return;

  if (!g_armed) {
    g_armed         = true;
    g_armedSmsSent  = false;
    g_motionSmsSent = false;
    g_motionDetected = false;
    Serial.println("*** ARMED ***");
    ledBlue();
    wifiStart();
    if (g_mpuReady) mpuStartCalibration();
  } else {
    g_armed = false;
    g_motionCandidate = false;
    wifiStop();
    buzzerStop();
    ledOff();
    Serial.println("*** DISARMED ***");
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_SD_CS,   OUTPUT); digitalWrite(PIN_SD_CS,   HIGH);
  pinMode(PIN_RFID_SDA, OUTPUT); digitalWrite(PIN_RFID_SDA, HIGH);

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledOff();

  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RFID_SDA);

  bool ok = loadConfig(cfg);
  Serial.println(ok ? "Config load: OK" : "Config load: FAILED");
  if (ok) printConfig(cfg);

  Serial.println("RFID: init...");
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_DumpVersionToSerial();
  Serial.println("RFID: ready");

  Serial.println("MPU: init...");
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(5);
  g_mpuReady = mpuInit();
  Serial.println(g_mpuReady ? "MPU: init OK" : "MPU: init FAILED");

  Serial.println("Disarmed. Present authorized tag to arm.");
}

void loop() {
  handleRfid();

  if (!g_armed) return;

  wifiHandle();
  buzzerHandle();

  if (!g_mpuReady) return;

  if (g_calibrating) {
    mpuHandleCalibration();
    return;
  }

  if (mpuCheckMotion()) {
    Serial.print("MOTION DETECTED!  filtered=");
    Serial.print(g_filtered, 1);
    Serial.print("  baseline=");
    Serial.print(g_baseline, 1);
    Serial.print("  delta=");
    Serial.print(fabsf(g_filtered - g_baseline), 1);
    Serial.print("  threshold=");
    Serial.println(cfg.mpuThreshold);

    if (!g_motionSmsSent) {
      g_motionDetected = true;
      ledRed();
      String body = "Bike alarm: motion detected\n"
                    "Time: "  + getTimeString() + "\n"
                    "WiFi: "  + cfg.ssid + "\n"
                    "IP: "    + WiFi.localIP().toString() + "\n"
                    "RSSI: "  + String(WiFi.RSSI()) + " dBm";
      sendSms(body);
      g_motionSmsSent = true;
    }
  }
}
