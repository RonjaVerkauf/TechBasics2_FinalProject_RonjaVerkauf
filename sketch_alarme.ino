#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP_Mail_Client.h>

// ===================== PINS =====================
static const int PIN_SPI_SCK = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;

static const int PIN_SD_CS = 5;

static const int PIN_RFID_SS = 17;  // RC522 SDA/SS
static const int PIN_RFID_RST = 16;

// MPU-6050 I2C (default)
static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;

// RGB LED (common anode)
static const int PIN_LED_R = 25;
static const int PIN_LED_G = 26;
static const int PIN_LED_B = 27;
static const bool RGB_COMMON_ANODE = true;

// Active buzzer signal pin
static const int PIN_BUZZ = 33;

// ===================== MPU REGISTERS =====================
// Doc: https://elektro.turanis.de/html/prj075/index.html
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ===================== MPU RECOVERY =====================
static uint16_t g_mpuReadFailStreak = 0;
static uint32_t g_mpuLastRecoverMs = 0;
static const uint16_t MPU_FAIL_STREAK_TO_RECOVER = 25;
static const uint32_t MPU_RECOVER_COOLDOWN_MS = 2000;

// ===================== CONFIG =====================
static const int MAX_UIDS = 20;

struct Config {
  String ssid;
  String pass;

  String smtpHost;
  uint16_t smtpPort = 0;
  String smtpUser;
  String smtpPass;

  int mpuThreshold = 3000;

  uint8_t uidCount = 0;
  uint8_t uidLen[MAX_UIDS] = { 0 };
  uint8_t uidBytes[MAX_UIDS][10] = { { 0 } };
};

Config cfg;

// ===================== RFID =====================
// https://github.com/miguelbalboa/rfid/tree/master
MFRC522 mfrc522(PIN_RFID_SS, PIN_RFID_RST);

// ===================== STATE MACHINE =====================
enum class State { DISARMED,
                   ARMING_WIFI,
                   ARMED_WIFI,
                   MOTION,
                   ALARM };
static State g_state = State::DISARMED;
// The state after setup is DISARMED
// The use must introduce a valid RFID to arm

// ===================== TIMING / DEBOUNCE =====================
// Timeouts are needed in order not to overload the pins
static uint32_t g_lastRfidMs = 0;
static const uint32_t RFID_DEBOUNCE_MS = 800;

static bool g_wifiConnecting = false;
static uint32_t g_wifiConnectStartMs = 0;
static uint32_t g_wifiRetryAfterMs = 0;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

// After-motion WiFi loss escalation
static uint32_t g_wifiLostSinceMs = 0;
static const uint32_t WIFI_LOST_GRACE_MS = 5000;

// Buzzer pattern
static bool g_buzzOn = false;
static uint32_t g_buzzToggleMs = 0;
static const uint32_t BUZZ_ON_MS = 120;
static const uint32_t BUZZ_OFF_MS = 380;

// ===================== MPU TRACKING =====================
static bool g_mpuReady = false;

static float g_baselineMag = 16384.0f;
static float g_filteredMag = 16384.0f;

static bool g_motionCandidate = false;
static uint32_t g_motionStartMs = 0;
static const uint32_t MOTION_CONFIRM_MS = 150;

// Non-blocking calibration
// The blocking calibration do the whole calibration 
// in one run of loop(), but block loop() until sampling is done.
// The non-blocking calibration take only one sample per run of loop(),
// so it's not blocked. Sampling might take a bit longer, but the device
// keep on being responsive.
static bool g_calibrating = false;
static uint32_t g_calibStartMs = 0;
static float g_calibSum = 0;
static int g_calibCount = 0;
static uint32_t g_lastCalibSampleMs = 0;
static const uint32_t CALIB_DURATION_MS = 400;
static const uint32_t CALIB_SAMPLE_MS = 10;

// EMA filtering (Exponential Moving Average)
// low-pass filter that smooths noisy signals by weighting recent samples
// more than older ones. For each new sample:
// filtered = alpha * new_value + (1 - alpha) * previous_filtered
// Where:
// alpha is between 0 and 1
// Smaller alpha → more smoothing
// Larger alpha → more responsive
static const float MAG_EMA_ALPHA = 0.15f;  // tune 0.05..0.25

// Stability delay before armed email
static const uint32_t STABLE_DELAY_MS = 2000;
static bool g_armStableTimerRunning = false;
static uint32_t g_armStableStartMs = 0;

// Email send flags (logical)
static bool g_armedEmailQueuedOrSent = false;
static bool g_motionEmailQueuedOrSent = false;
static bool g_motionHappenedThisArm = false;

// ===================== EMAIL QUEUE =====================
enum class EmailType : uint8_t { ARMED,
                                 MOTION };

struct EmailJob {
  EmailType type;
  uint8_t retriesLeft;
  char subject[96];
  char body[512];
};

static QueueHandle_t g_emailQueue = nullptr;
static TaskHandle_t g_emailTaskHandle = nullptr;

// --------------------- small safe copy helper ---------------------
static void safeCopy(char* dst, size_t dstSize, const String& src) {
  if (dstSize == 0) return;
  size_t n = src.length();
  if (n >= dstSize) n = dstSize - 1;
  memcpy(dst, src.c_str(), n);
  dst[n] = '\0';
}

// ===================== LED =====================
static void setRgb(bool r, bool g, bool b) {
  auto writePin = [](int pin, bool on) {
    if (RGB_COMMON_ANODE) digitalWrite(pin, on ? LOW : HIGH);
    else digitalWrite(pin, on ? HIGH : LOW);
  };
  writePin(PIN_LED_R, r);
  writePin(PIN_LED_G, g);
  writePin(PIN_LED_B, b);
}

static void setLedForState() {
  switch (g_state) {
    case State::DISARMED: setRgb(true, true, false); break;      // yellow
    case State::ARMING_WIFI: setRgb(false, false, true); break;  // blue
    case State::ARMED_WIFI: setRgb(false, true, false); break;   // green
    case State::MOTION: setRgb(true, false, false); break;       // red
    case State::ALARM: setRgb(true, false, false); break;        // red
  }
}

static void setState(State st, const char* reason) {
  g_state = st;
  setLedForState();
  Serial.print("STATE -> ");
  Serial.print((int)st);
  Serial.print(" (");
  Serial.print(reason);
  Serial.println(")");
}

// ===================== BUZZER =====================
static void buzzerOn(bool on) {
  digitalWrite(PIN_BUZZ, on ? HIGH : LOW);
}

static void stopBuzzer() {
  g_buzzOn = false;
  buzzerOn(false);
}

static void handleBuzzerPattern() {
  uint32_t now = millis();
  uint32_t period = g_buzzOn ? BUZZ_ON_MS : BUZZ_OFF_MS;
  if (now - g_buzzToggleMs >= period) {
    g_buzzToggleMs = now;
    g_buzzOn = !g_buzzOn;
    buzzerOn(g_buzzOn);
  }
}

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
    if (tok.length() == 0) {
      i = j + 1;
      continue;
    }
    if (tok.length() > 2) return false;

    char* endptr = nullptr;
    long v = strtol(tok.c_str(), &endptr, 16);
    if (endptr == tok.c_str() || v < 0 || v > 255) return false;

    if (outLen >= 10) return false;
    out[outLen++] = (uint8_t)v;

    i = j + 1;
  }
  return outLen > 0;
}

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
  if (cfg.uidCount == 0) return true;  // allow any if list empty

  for (uint8_t i = 0; i < cfg.uidCount; i++) {
    if (cfg.uidLen[i] != uid.size) continue;
    bool match = true;
    for (byte j = 0; j < uid.size; j++) {
      if (cfg.uidBytes[i][j] != uid.uidByte[j]) {
        match = false;
        break;
      }
    }
    if (match) return true;
  }
  return false;
}

// ===================== SD CONFIG LOAD =====================
static bool loadConfigFromSD(Config& c) {
  c = Config();

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_RFID_SS, OUTPUT);
  digitalWrite(PIN_RFID_SS, HIGH);

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
    digitalWrite(PIN_SD_CS, HIGH);
    return false;
  }

  Serial.println("Config: reading /config.txt ...");
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("#")) continue;

    if (line.startsWith("SSID:")) c.ssid = valueAfterColon(line);
    else if (line.startsWith("PASS:")) c.pass = valueAfterColon(line);
    else if (line.startsWith("SMTPHost:")) c.smtpHost = valueAfterColon(line);
    else if (line.startsWith("SMTPPort:")) c.smtpPort = (uint16_t)valueAfterColon(line).toInt();
    else if (line.startsWith("SMTPUser:")) c.smtpUser = valueAfterColon(line);
    else if (line.startsWith("SMTPPass:")) c.smtpPass = valueAfterColon(line);
    else if (line.startsWith("MPUThreshold:")) c.mpuThreshold = valueAfterColon(line).toInt();
    else if (line.startsWith("RFID:")) {
      if (c.uidCount >= MAX_UIDS) continue;
      String uidStr = valueAfterColon(line);
      uint8_t len = 0;
      uint8_t bytes[10] = { 0 };
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
  digitalWrite(PIN_SD_CS, HIGH);
  Serial.println("SD: unmounted (SD.end())");
  return true;
}

// ===================== WIFI (NON-BLOCKING) =====================
static void wifiStartConnect() {
  if (cfg.ssid.length() == 0) {
    Serial.println("WiFi: SSID missing in config");
    return;
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);

  Serial.print("WiFi: begin SSID=");
  Serial.println(cfg.ssid);

  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  g_wifiConnecting = true;
  g_wifiConnectStartMs = millis();
}

static bool wifiPollConnect() {
  if (!g_wifiConnecting) return (WiFi.status() == WL_CONNECTED);

  if (WiFi.status() == WL_CONNECTED) {
    g_wifiConnecting = false;
    Serial.print("WiFi: connected. IP=");
    Serial.println(WiFi.localIP());
    return true;
  }

  if (millis() - g_wifiConnectStartMs >= WIFI_CONNECT_TIMEOUT_MS) {
    Serial.println("WiFi: connect TIMEOUT -> retry later");
    WiFi.disconnect(true, true);
    g_wifiConnecting = false;
    g_wifiRetryAfterMs = millis() + 2000;
  }
  return false;
}

static void wifiStop() {
  WiFi.disconnect(true, true);
  g_wifiConnecting = false;
  g_wifiRetryAfterMs = 0;
}

static void superviseWifiWhileArmed() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("WiFi lost (armed) -> reconnecting");
  setState(State::ARMING_WIFI, "wifi_lost");
  g_wifiRetryAfterMs = 0;
}

// ===================== MPU (I2C) =====================
static bool mpuWriteByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

static bool mpuReadAccelRaw(int16_t& ax, int16_t& ay, int16_t& az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)MPU_ADDR, 6, true) != 6) return false;

  uint8_t b0 = Wire.read();
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();
  uint8_t b4 = Wire.read();
  uint8_t b5 = Wire.read();

  ax = (int16_t)((b0 << 8) | b1);
  ay = (int16_t)((b2 << 8) | b3);
  az = (int16_t)((b4 << 8) | b5);
  return true;
}

static void attemptMpuRecoverIfNeeded() {
  if (!g_mpuReady) return;
  if (g_mpuReadFailStreak < MPU_FAIL_STREAK_TO_RECOVER) return;

  const uint32_t now = millis();
  if (now - g_mpuLastRecoverMs < MPU_RECOVER_COOLDOWN_MS) return;

  g_mpuLastRecoverMs = now;
  g_mpuReadFailStreak = 0;

  Serial.println("MPU: attempting recover");

  Wire.end();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(5);

  g_mpuReady = mpuInit();

  if (g_state == State::ARMED_WIFI && g_mpuReady) {
    startCalibration();
    g_motionCandidate = false;
  }
}

static float mag3raw(int16_t ax, int16_t ay, int16_t az) {
  float x = (float)ax, y = (float)ay, z = (float)az;
  return sqrtf(x * x + y * y + z * z);
}

static bool mpuInit() {
  return mpuWriteByte(REG_PWR_MGMT_1, 0x00);
}

static void startCalibration() {
  g_calibrating = true;
  g_calibStartMs = millis();
  g_lastCalibSampleMs = 0;
  g_calibSum = 0;
  g_calibCount = 0;
  Serial.println("MPU: calibration started...");
}

static void handleCalibration() {
  if (!g_calibrating) return;

  uint32_t now = millis();

  if (g_lastCalibSampleMs == 0 || (now - g_lastCalibSampleMs >= CALIB_SAMPLE_MS)) {
    g_lastCalibSampleMs = now;

    int16_t ax, ay, az;
    if (mpuReadAccelRaw(ax, ay, az)) {
      g_mpuReadFailStreak = 0;
      g_calibSum += mag3raw(ax, ay, az);
      g_calibCount++;
    } else {
      if (g_mpuReadFailStreak < 0xFFFF) g_mpuReadFailStreak++;
      attemptMpuRecoverIfNeeded();
    }
  }

  if (now - g_calibStartMs >= CALIB_DURATION_MS) {
    if (g_calibCount > 0) g_baselineMag = g_calibSum / g_calibCount;
    g_filteredMag = g_baselineMag;
    Serial.print("MPU baseline raw magnitude: ");
    Serial.println(g_baselineMag, 1);
    Serial.print("MPU filtered magnitude init: ");
    Serial.println(g_filteredMag, 1);
    g_calibrating = false;

    // start stability timer (armed email will be delayed)
    g_armStableTimerRunning = true;
    g_armStableStartMs = millis();
  }
}

static bool mpuMotionDetected(float& outRawMag, float& outDelta) {
  int16_t ax, ay, az;
  if (!mpuReadAccelRaw(ax, ay, az)) {
    if (g_mpuReadFailStreak < 0xFFFF) g_mpuReadFailStreak++;
    attemptMpuRecoverIfNeeded();
    return false;
  }
  g_mpuReadFailStreak = 0;

  float raw = mag3raw(ax, ay, az);

  // EMA smoothing on magnitude
  g_filteredMag += MAG_EMA_ALPHA * (raw - g_filteredMag);

  float delta = fabsf(g_filteredMag - g_baselineMag);

  outRawMag = raw;
  outDelta = delta;

  uint32_t now = millis();
  if (delta > (float)cfg.mpuThreshold) {
    if (!g_motionCandidate) {
      g_motionCandidate = true;
      g_motionStartMs = now;
    } else if (now - g_motionStartMs >= MOTION_CONFIRM_MS) {
      g_motionCandidate = false;
      return true;
    }
  } else {
    g_motionCandidate = false;
  }
  return false;
}

// ===================== EMAIL SERVICE (TASK) =====================
static bool smtpSendBlocking(const EmailJob& job) {
  if (WiFi.status() != WL_CONNECTED) return false;

  // Require SMTP config
  if (cfg.smtpHost.isEmpty() || cfg.smtpPort == 0 || cfg.smtpUser.isEmpty() || cfg.smtpPass.isEmpty()) {
    Serial.println("MAIL: skipped (SMTP config missing)");
    return false;
  }

  SMTPSession smtp;
  ESP_Mail_Session session;
  session.server.host_name = cfg.smtpHost.c_str();
  session.server.port = cfg.smtpPort;
  session.login.email = cfg.smtpUser.c_str();
  session.login.password = cfg.smtpPass.c_str();
  session.login.user_domain = "";

  SMTP_Message message;
  message.sender.name = "Bike Alarm";
  message.sender.email = cfg.smtpUser.c_str();  // owner email = smtp user
  message.subject = job.subject;
  message.addRecipient("Owner", cfg.smtpUser.c_str());

  message.text.content = job.body;
  message.text.charSet = "utf-8";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  if (!smtp.connect(&session)) {
    Serial.print("MAIL: connect failed: ");
    Serial.println(smtp.errorReason());
    smtp.closeSession();
    return false;
  }

  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.print("MAIL: send failed: ");
    Serial.println(smtp.errorReason());
    smtp.closeSession();
    return false;
  }

  smtp.closeSession();
  Serial.println("MAIL: sent.");
  return true;
}

static void emailTask(void* parameter) {
  (void)parameter;
  EmailJob job;

  for (;;) {
    if (xQueueReceive(g_emailQueue, &job, portMAX_DELAY) == pdTRUE) {

      // If WiFi is down, wait (up to 60s total) for connectivity
      uint32_t waitedMs = 0;
      while (WiFi.status() != WL_CONNECTED && waitedMs < 60000) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        waitedMs += 500;
      }

      bool ok = smtpSendBlocking(job);

      if (!ok && job.retriesLeft > 0) {
        job.retriesLeft--;
        Serial.print("MAIL: retry scheduled, retriesLeft=");
        Serial.println(job.retriesLeft);

        vTaskDelay(5000 / portTICK_PERIOD_MS);  // backoff
        xQueueSend(g_emailQueue, &job, 0);
      }
    }
  }
}

static void queueEmail(EmailType type, const String& subject, const String& body) {
  if (!g_emailQueue) return;

  EmailJob job;
  job.type = type;
  job.retriesLeft = 2;  // total attempts = 1 + 2 retries

  safeCopy(job.subject, sizeof(job.subject), subject);
  safeCopy(job.body, sizeof(job.body), body);

  // best-effort enqueue
  if (xQueueSend(g_emailQueue, &job, 0) != pdTRUE) {
    Serial.println("MAIL: queue full, dropping email");
  } else {
    Serial.println("MAIL: queued.");
  }
}

// ===================== RFID TOGGLE =====================
static void resetArmCycleFlags() {
  g_armedEmailQueuedOrSent = false;
  g_motionEmailQueuedOrSent = false;
  g_motionHappenedThisArm = false;

  g_armStableTimerRunning = false;
  g_armStableStartMs = 0;

  g_wifiLostSinceMs = 0;
  g_motionCandidate = false;

  g_calibrating = false;
}

static void handleRfidToggle() {
  uint32_t now = millis();
  if (now - g_lastRfidMs < RFID_DEBOUNCE_MS) return;

  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  g_lastRfidMs = now;

  String uidStr = uidToString(mfrc522.uid);
  bool ok = uidAuthorized(mfrc522.uid);

  Serial.print("RFID UID: ");
  Serial.print(uidStr);
  Serial.println(ok ? " (OK)" : " (UNAUTHORIZED)");

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  if (!ok) return;

  // toggle
  if (g_state == State::DISARMED) {
    resetArmCycleFlags();
    setState(State::ARMING_WIFI, "rfid_arm");
    wifiStartConnect();
  } else {
    // DISARM from ANY state
    wifiStop();
    stopBuzzer();
    resetArmCycleFlags();
    setState(State::DISARMED, "rfid_disarm");
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // LED pins
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  // Buzzer
  pinMode(PIN_BUZZ, OUTPUT);
  buzzerOn(false);

  // CS deselects
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_RFID_SS, OUTPUT);
  digitalWrite(PIN_RFID_SS, HIGH);

  // SPI init
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RFID_SS);

  // Load config
  bool ok = loadConfigFromSD(cfg);
  Serial.println(ok ? "Config load: OK" : "Config load: FAIL");
  Serial.print("MPUThreshold(delta on filtered mag): ");
  Serial.println(cfg.mpuThreshold);

  // RFID init
  Serial.println("RFID: init...");
  mfrc522.PCD_Init();
  delay(50);
  mfrc522.PCD_DumpVersionToSerial();
  Serial.println("RFID: ready");

  // I2C + MPU init
  Serial.println("MPU: init...");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(5);
  g_mpuReady = mpuInit();
  Serial.println(g_mpuReady ? "MPU: init OK" : "MPU: init FAILED");

  // Email queue + task
  g_emailQueue = xQueueCreate(6, sizeof(EmailJob));  // up to 6 pending messages
  if (g_emailQueue) {
    xTaskCreatePinnedToCore(
      emailTask,
      "EmailTask",
      12000,  // stack
      nullptr,
      1,  // priority
      &g_emailTaskHandle,
      0  // core 0
    );
    Serial.println("MAIL: task started");
  } else {
    Serial.println("MAIL: queue create FAILED");
  }

  setState(State::DISARMED, "boot");
  Serial.println("Present authorized tag to arm/disarm.");
}

void loop() {
  handleRfidToggle();

  // ================= WiFi connect/reconnect state =================
  if (g_state == State::ARMING_WIFI) {
    if (!g_wifiConnecting && millis() >= g_wifiRetryAfterMs && WiFi.status() != WL_CONNECTED) {
      wifiStartConnect();
    }

    if (wifiPollConnect()) {
      setState(State::ARMED_WIFI, "wifi_ok");

      // start calibration on entry
      if (g_mpuReady) {
        startCalibration();
        g_motionCandidate = false;
      } else {
        // If no MPU, still start stability timer for email policy
        g_armStableTimerRunning = true;
        g_armStableStartMs = millis();
      }
    }
  }

  // While ARMED_WIFI (pre-motion), auto-reconnect if WiFi drops
  if (g_state == State::ARMED_WIFI) {
    superviseWifiWhileArmed();
  }

  // ================= Calibration (non-blocking) =================
  if (g_state == State::ARMED_WIFI && g_mpuReady) {
    handleCalibration();
  }

  // ================= Stable delay -> queue ARMED email =================
  // Only once per arm cycle, and only if:
  // - still ARMED_WIFI
  // - WiFi connected
  // - calibration finished (or no MPU)
  // - no motion has happened in this arm cycle
  if (g_state == State::ARMED_WIFI && !g_armedEmailQueuedOrSent && g_armStableTimerRunning && !g_calibrating && !g_motionHappenedThisArm && WiFi.status() == WL_CONNECTED) {

    if (millis() - g_armStableStartMs >= STABLE_DELAY_MS) {
      String body =
        "Bike alarm armed and stable.\n"
        "IP: "
        + WiFi.localIP().toString() + "\n"
                                      "RSSI: "
        + String(WiFi.RSSI()) + " dBm\n"
                                "MPUThreshold(delta on filtered mag): "
        + String(cfg.mpuThreshold) + "\n"
                                     "EMA alpha: "
        + String(MAG_EMA_ALPHA, 2) + "\n"
                                     "Baseline: "
        + String(g_baselineMag, 1) + "\n";

      queueEmail(EmailType::ARMED, "Bike alarm armed", body);
      g_armedEmailQueuedOrSent = true;
    }
  }

  // ================= Motion monitoring =================
  if (g_state == State::ARMED_WIFI && g_mpuReady && !g_calibrating) {
    float rawMag = 0, delta = 0;
    if (mpuMotionDetected(rawMag, delta)) {
      Serial.print("MOTION DETECTED! rawMag=");
      Serial.print(rawMag, 1);
      Serial.print(" filteredMag=");
      Serial.print(g_filteredMag, 1);
      Serial.print(" delta=");
      Serial.print(delta, 1);
      Serial.print(" threshold=");
      Serial.println(cfg.mpuThreshold);

      // latch motion
      setState(State::MOTION, "motion");
      g_motionHappenedThisArm = true;

      // cancel/avoid armed email if it wasn't sent yet
      // (we can't remove it from the queue, but we prevent queuing it in the first place)
      // At this point it might already be queued/sent; that's acceptable.

      // start watching WiFi loss-after-motion
      g_wifiLostSinceMs = 0;
      stopBuzzer();

      if (!g_motionEmailQueuedOrSent) {
        String body =
          "Motion detected by bike alarm.\n"
          "IP: "
          + WiFi.localIP().toString() + "\n"
                                        "RSSI: "
          + String(WiFi.RSSI()) + " dBm\n"
                                  "rawMag: "
          + String(rawMag, 1) + "\n"
                                "filteredMag: "
          + String(g_filteredMag, 1) + "\n"
                                       "delta: "
          + String(delta, 1) + "\n"
                               "baseline: "
          + String(g_baselineMag, 1) + "\n"
                                       "threshold(delta on filtered mag): "
          + String(cfg.mpuThreshold) + "\n"
                                       "EMA alpha: "
          + String(MAG_EMA_ALPHA, 2) + "\n";

        queueEmail(EmailType::MOTION, "Bike alarm: motion detected", body);
        g_motionEmailQueuedOrSent = true;
      }
    }
  }

  // ================= After motion: WiFi loss -> ALARM + buzzer =================
  if (g_state == State::MOTION) {
    if (WiFi.status() == WL_CONNECTED) {
      g_wifiLostSinceMs = 0;
    } else {
      if (g_wifiLostSinceMs == 0) {
        g_wifiLostSinceMs = millis();
        Serial.println("WiFi lost after motion -> grace timer started");
      } else if (millis() - g_wifiLostSinceMs >= WIFI_LOST_GRACE_MS) {
        Serial.println("WiFi lost after motion -> ALARM buzzer ON");
        setState(State::ALARM, "wifi_lost_after_motion");
        g_buzzOn = false;
        g_buzzToggleMs = millis();
        buzzerOn(false);
      }
    }
  }

  // ================= Alarm: beep while WiFi absent; stop if WiFi returns =================
  if (g_state == State::ALARM) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi back -> stopping buzzer (still in MOTION)");
      stopBuzzer();
      setState(State::MOTION, "wifi_back");
      g_wifiLostSinceMs = 0;
    } else {
      handleBuzzerPattern();
    }
  }

  yield();
}
