// =============================================================================
// eth_ntp_server.ino
// ESP32 GPS+PPS Stratum-1 NTP Server — Ethernet (W5500)
//
// Architecture (FreeRTOS, three tasks):
//   Core 1 — gpsTask    : NMEA parsing + PPS sync      (highest priority)
//   Core 0 — ntpTask    : NTP UDP responses             (high priority)
//   Core 0 — displayTask: OLED update 2×/s              (lowest priority)
//
// Timing approach:
//   The PPS ISR stores esp_timer_get_time() on pulse edge transition.
//   gpsTask combines the PPS timestamp + latest NMEA epoch into a
//   stable anchor point (lastSyncUnixSec, lastSyncMicrosAtPps) under timeMutex.
//   getPreciseTime() interpolates elapsed µs from this anchor point.
//
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =============================================================================
// PRECISE TIME — defined here so type is visible everywhere
// =============================================================================

struct PreciseTime {
  uint64_t seconds;       // Unix timestamp
  uint32_t microseconds;  // Sub-second component [0, 999999]
};

// =============================================================================
// CONFIGURATION
// =============================================================================

#define DEBUG_MODE true

// --- Ethernet (W5500) --------------------------------------------------------
#define PIN_ETH_CS   14
#define PIN_ETH_SCK  27
#define PIN_ETH_MISO 26
#define PIN_ETH_MOSI 25
#define ETH_SPI_FREQUENCY 8000000

byte ETH_MAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ETH_IP     (10, 0, 0, 13);
IPAddress ETH_GATEWAY(10, 0, 0,  1);
IPAddress ETH_SUBNET (255, 255, 0, 0);
IPAddress ETH_DNS    (10, 0, 0,  1);

// --- GPS (NEO-6M) ------------------------------------------------------------
#define PIN_GPS_RX        16
#define PIN_GPS_TX        17
#define GPS_BAUD_DEFAULT   9600
#define GPS_MIN_VALID_UNIX 1700000000UL   // 2023-11-14 plausibility limit

// --- PPS ---------------------------------------------------------------------
#define PIN_PPS                32
#define PPS_STALE_THRESHOLD_US 2100000ULL  // >2.1 s without pulse → stale

// --- OLED (SSD1306 128×64) ---------------------------------------------------
#define PIN_OLED_SDA   23
#define PIN_OLED_SCL   18
#define OLED_WIDTH     128
#define OLED_HEIGHT     64
#define OLED_RESET      -1
#define OLED_I2C_ADDR  0x3C
#define OLED_LINE_H     10

// --- NTP ---------------------------------------------------------------------
#define NTP_PORT        123
#define NTP_PACKET_SIZE  48
static const uint32_t NTP_EPOCH_OFFSET = 2208988800UL;

#define NTP_PRECISION_GPS    0xEC   // 2^-20 ≈ 1 µs   (GPS + PPS)
#define NTP_PRECISION_NMEA   0xF0   // 2^-16 ≈ 15 µs  (GPS, no PPS)
#define NTP_PRECISION_NTP    0xF6   // 2^-10 ≈ 1 ms   (NTP fallback)

#define NTP_DISPERSION_GPS   0x08
#define NTP_DISPERSION_NMEA  0x30
#define NTP_DISPERSION_NTP   0x50

#define NTP_MUTEX_TIMEOUT_MS 2

// =============================================================================
// GPS DISPLAY SNAPSHOT  (gpsTask writes under gpsMutex, displayTask reads)
// =============================================================================

static SemaphoreHandle_t gpsMutex;

struct GpsSnapshot {
  bool    valid;
  uint8_t hour, minute, second;
  uint8_t satellites;
  float   hdop;
  bool    hasSats;
  bool    hasHdop;
};
static GpsSnapshot gpsSnap = {};

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================

HardwareSerial   gpsSerial(1);
TinyGPSPlus      gps;
EthernetUDP      ntpServerUDP;
EthernetUDP      ntpFallbackUDP;
NTPClient        ntpFallbackClient(ntpFallbackUDP, "pool.ntp.org", 0, 30000);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// =============================================================================
// SHARED TIMING STATE
// =============================================================================

static SemaphoreHandle_t timeMutex;
static volatile uint64_t lastSyncUnixSec     = 0;
static volatile uint64_t lastSyncMicrosAtPps = 0;

static String currentTimeSource = "NONE";

// =============================================================================
// PPS STATE  (ISR → gpsTask, unidirectional)
// =============================================================================

static volatile bool     ppsFired         = false;
static volatile uint64_t lastPpsMicrosIsr = 0;
static volatile bool     ppsAvailable     = false;
static bool isPpsValid() {
  if (!ppsAvailable) return false;
  return (esp_timer_get_time() - lastPpsMicrosIsr) < PPS_STALE_THRESHOLD_US;
}

// =============================================================================
// GPS STATE
// =============================================================================

static volatile uint32_t latestGpsUnixSec  = 0;
static volatile bool     latestGpsSecValid = false;

// =============================================================================
// MISC
// =============================================================================

static uint8_t ntpPacketBuffer[NTP_PACKET_SIZE];

// =============================================================================
// PRECISE TIME — Functions (Struct definition above, after includes)
// =============================================================================

static PreciseTime getPreciseTime() {
  uint64_t now     = esp_timer_get_time();
  uint64_t elapsed = now - lastSyncMicrosAtPps;  // always ≥ 0

  PreciseTime pt;
  pt.seconds      = lastSyncUnixSec + elapsed / 1000000ULL;
  pt.microseconds = (uint32_t)(elapsed % 1000000ULL);
  return pt;
}

static PreciseTime getPreciseTimeSafe() {
  PreciseTime pt = { lastSyncUnixSec, 0 };

  if (xSemaphoreTake(timeMutex, pdMS_TO_TICKS(NTP_MUTEX_TIMEOUT_MS)) == pdTRUE) {
    pt = getPreciseTime();
    xSemaphoreGive(timeMutex);
  } else {
    if (DEBUG_MODE) Serial.println("[TIME] Mutex timeout in getPreciseTimeSafe");
  }
  return pt;
}

// =============================================================================
// PPS INTERRUPT
// =============================================================================

void IRAM_ATTR handlePpsInterrupt() {
  lastPpsMicrosIsr = esp_timer_get_time();
  ppsFired         = true;
  ppsAvailable     = true;
}

// =============================================================================
// GPS HELPERS
// =============================================================================

static bool isGpsTimeValid() {
  return gps.date.isValid()
      && gps.time.isValid()
      && gps.date.year()   >= 2024
      && gps.date.month()  >= 1  && gps.date.month()  <= 12
      && gps.date.day()    >= 1  && gps.date.day()    <= 31
      && gps.time.hour()   <= 23
      && gps.time.minute() <= 59
      && gps.time.second() <= 60;
}

static uint32_t gpsToEpoch() {
  uint8_t  d = gps.date.day();
  uint8_t  m = gps.date.month();
  uint16_t y = gps.date.year();

  int Y = y, M = m;
  if (M < 3) { Y--; M += 12; }
  uint32_t era  = (uint32_t)Y / 400;
  uint32_t yoe  = (uint32_t)Y - era * 400;
  uint32_t doy  = (153u * (uint32_t)(M - 3) + 2u) / 5u + d - 1u;
  uint32_t doe  = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  uint32_t days = era * 146097u + doe - 719468u;

  uint32_t epoch = days * 86400UL
                 + gps.time.hour()   * 3600UL
                 + gps.time.minute() * 60UL
                 + gps.time.second();

  return (epoch > GPS_MIN_VALID_UNIX) ? epoch : 0;
}

// =============================================================================
// SYNC TIME WITH GPS  (called by gpsTask under timeMutex)
// =============================================================================

// Priority:
//   1. GPS+PPS  → anchor point = (NMEA epoch+1, ISR timestamp)
//   2. PPS only → anchor point = (last epoch+1, ISR timestamp)
//   3. NMEA only → anchor point = (NMEA epoch, esp_timer_get_time() now)
//   4. NTP fallback → set in ntpTask
static void syncTimeWithGPS() {
  const bool ppsLive = isPpsValid();

  if (ppsFired && ppsLive) {
    ppsFired = false;

    if (latestGpsSecValid) {
      // Best case: GPS+PPS. +1 because the pulse fires at start of second N+1,
      // but NMEA sentence still carries second N.
      lastSyncUnixSec     = (uint64_t)latestGpsUnixSec + 1;
      lastSyncMicrosAtPps = lastPpsMicrosIsr;
      currentTimeSource   = "GPS+PPS";
      return;
    }

    if (lastSyncUnixSec != 0) {
      lastSyncUnixSec++;
      lastSyncMicrosAtPps = lastPpsMicrosIsr;
      currentTimeSource   = "PPS";
      return;
    }
  }

  if (latestGpsSecValid) {
    lastSyncUnixSec     = latestGpsUnixSec;
    lastSyncMicrosAtPps = esp_timer_get_time();
    currentTimeSource   = ppsLive ? "GPS+PPS" : "GPS";
  }
}

// =============================================================================
// NTP TIMESTAMP BUILDER
// =============================================================================

static uint64_t preciseTimeToNtp(const PreciseTime& pt) {
  uint64_t sec1900 = pt.seconds + (uint64_t)NTP_EPOCH_OFFSET;
  uint32_t frac    = (uint32_t)(((uint64_t)pt.microseconds * 4294967296ULL) / 1000000ULL);
  return (sec1900 << 32) | frac;
}

// =============================================================================
// BINARY WRITE HELPERS
// =============================================================================

static void writeU32BE(uint8_t* buf, int offset, uint32_t v) {
  buf[offset + 0] = (v >> 24) & 0xFF;
  buf[offset + 1] = (v >> 16) & 0xFF;
  buf[offset + 2] = (v >>  8) & 0xFF;
  buf[offset + 3] =  v        & 0xFF;
}

static void writeU64BE(uint8_t* buf, int offset, uint64_t v) {
  buf[offset + 0] = (v >> 56) & 0xFF;
  buf[offset + 1] = (v >> 48) & 0xFF;
  buf[offset + 2] = (v >> 40) & 0xFF;
  buf[offset + 3] = (v >> 32) & 0xFF;
  buf[offset + 4] = (v >> 24) & 0xFF;
  buf[offset + 5] = (v >> 16) & 0xFF;
  buf[offset + 6] = (v >>  8) & 0xFF;
  buf[offset + 7] =  v        & 0xFF;
}

// =============================================================================
// NTP RESPONSE
// =============================================================================

static void sendNtpResponse(IPAddress remoteIp,
                            uint16_t  remotePort,
                            uint8_t*  request) {

  const PreciseTime receiveTime = getPreciseTimeSafe();

  const bool ppsLive = isPpsValid();
  const bool usingGps = (currentTimeSource == "GPS+PPS" ||
                         currentTimeSource == "GPS"      ||
                         currentTimeSource == "PPS");
  const bool usingPps = ppsLive && (currentTimeSource == "GPS+PPS" ||
                                    currentTimeSource == "PPS");

  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);

  ntpPacketBuffer[0] = 0x24;
  ntpPacketBuffer[1] = usingGps ? 1 : 2;   // Stratum
  ntpPacketBuffer[2] = 4;                   // Poll exponent (informational)
  ntpPacketBuffer[3] = usingPps  ? NTP_PRECISION_GPS
                     : usingGps  ? NTP_PRECISION_NMEA
                                 : NTP_PRECISION_NTP;

  writeU32BE(ntpPacketBuffer,  4, 0);       // Root delay
  writeU32BE(ntpPacketBuffer,  8, usingPps  ? NTP_DISPERSION_GPS
                                : usingGps  ? NTP_DISPERSION_NMEA
                                            : NTP_DISPERSION_NTP);

  ntpPacketBuffer[12] = usingGps ? 'G' : 'N';
  ntpPacketBuffer[13] = usingGps ? 'P' : 'T';
  ntpPacketBuffer[14] = usingGps ? 'S' : 'P';
  ntpPacketBuffer[15] = 0;

  {
    uint64_t refNtp = ((uint64_t)(lastSyncUnixSec + NTP_EPOCH_OFFSET)) << 32;
    writeU64BE(ntpPacketBuffer, 16, refNtp);
  }

  memcpy(&ntpPacketBuffer[24], &request[40], 8);                  // Origin = Client TX
  writeU64BE(ntpPacketBuffer, 32, preciseTimeToNtp(receiveTime));  // Receive

  const PreciseTime transmitTime = getPreciseTimeSafe();
  writeU64BE(ntpPacketBuffer, 40, preciseTimeToNtp(transmitTime)); // Transmit

  ntpServerUDP.beginPacket(remoteIp, remotePort);
  ntpServerUDP.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  ntpServerUDP.endPacket();

  if (DEBUG_MODE) {
    Serial.printf("[NTP] → %s:%u  src=%s  pps=%s\n",
                  remoteIp.toString().c_str(), remotePort,
                  currentTimeSource.c_str(), usingPps ? "yes" : "no");
  }
}

// =============================================================================
// FREERTOS TASK — gpsTask  (Core 1, highest priority)
// =============================================================================

void gpsTask(void* param) {
  if (DEBUG_MODE) Serial.println("[gpsTask] Started");

  for (;;) {
    // Feed all available UART bytes into TinyGPS (sole writer).
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    // Update GPS snapshot and latestGpsUnixSec under gpsMutex.
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      gpsSnap.valid      = isGpsTimeValid();
      gpsSnap.hour       = gps.time.hour();
      gpsSnap.minute     = gps.time.minute();
      gpsSnap.second     = gps.time.second();
      gpsSnap.hasSats    = gps.satellites.isValid();
      gpsSnap.satellites = gpsSnap.hasSats ? gps.satellites.value() : 0;
      gpsSnap.hasHdop    = gps.hdop.isValid();
      gpsSnap.hdop       = gpsSnap.hasHdop ? gps.hdop.hdop() : 0.0f;

      if (gpsSnap.valid) {
        uint32_t t = gpsToEpoch();
        if (t > 0) {
          latestGpsUnixSec  = t;
          latestGpsSecValid = true;
        }
      } else {
        latestGpsSecValid = false;
      }
      xSemaphoreGive(gpsMutex);
    }

    // Combine PPS+NMEA into stable sync anchor point.
    if (xSemaphoreTake(timeMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      syncTimeWithGPS();
      xSemaphoreGive(timeMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// FREERTOS TASK — ntpTask  (Core 0, high priority)
// =============================================================================

void ntpTask(void* param) {
  if (DEBUG_MODE) Serial.println("[ntpTask] Started");

  ntpFallbackClient.begin();
  ntpFallbackClient.setTimeOffset(0);
  ntpFallbackClient.forceUpdate();

  ntpServerUDP.begin(NTP_PORT);
  if (DEBUG_MODE) Serial.println("[NTP] Listening on UDP 123");

  if (ntpFallbackClient.isTimeSet()) {
    if (xSemaphoreTake(timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      lastSyncUnixSec     = ntpFallbackClient.getEpochTime();
      lastSyncMicrosAtPps = esp_timer_get_time();
      currentTimeSource   = "NTP";
      xSemaphoreGive(timeMutex);
      if (DEBUG_MODE) Serial.printf("[NTP] Initial seed: %llu\n", lastSyncUnixSec);
    }
  }

  unsigned long lastFallbackUpdate = ntpFallbackClient.isTimeSet() ? millis() : 0;

  for (;;) {
    int packetSize = ntpServerUDP.parsePacket();
    if (packetSize == NTP_PACKET_SIZE) {
      IPAddress remoteIp   = ntpServerUDP.remoteIP();
      uint16_t  remotePort = ntpServerUDP.remotePort();
      uint8_t   req[NTP_PACKET_SIZE];
      ntpServerUDP.read(req, NTP_PACKET_SIZE);
      sendNtpResponse(remoteIp, remotePort, req);
    } else if (packetSize > 0) {
      ntpServerUDP.flush();
      if (DEBUG_MODE) Serial.printf("[NTP] Malformed packet (%d B) discarded\n", packetSize);
    }

    bool needFallback = (currentTimeSource == "NONE" || currentTimeSource == "NTP");
    if (needFallback && millis() - lastFallbackUpdate > 30000) {
      lastFallbackUpdate = millis();
      ntpFallbackClient.update();
      if (ntpFallbackClient.isTimeSet()) {
        if (xSemaphoreTake(timeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          lastSyncUnixSec     = ntpFallbackClient.getEpochTime();
          lastSyncMicrosAtPps = esp_timer_get_time();
          currentTimeSource   = "NTP";
          xSemaphoreGive(timeMutex);
          if (DEBUG_MODE) Serial.printf("[NTP] Fallback sync: %llu\n", lastSyncUnixSec);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// FREERTOS TASK — displayTask  (Core 0, lowest priority)
// =============================================================================

static void oledLine(int line, const String& text) {
  display.setCursor(0, line * OLED_LINE_H);
  display.println(text);
}

void displayTask(void* param) {
  if (DEBUG_MODE) Serial.println("[displayTask] Started");

  for (;;) {
    PreciseTime pt = getPreciseTimeSafe();

    GpsSnapshot snap;
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      snap = gpsSnap;
      xSemaphoreGive(gpsMutex);
    }

    char timeBuf[22];
    time_t     t     = (time_t)pt.seconds;
    struct tm* tmPtr = gmtime(&t);

    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d.%06lu%s",
             tmPtr->tm_hour, tmPtr->tm_min, tmPtr->tm_sec,
             (unsigned long)pt.microseconds,
             isPpsValid() ? "" : "~");

    String gpsStatus = snap.valid ? "GPS OK" : "GPS WAIT";
    String sats      = snap.hasSats ? String(snap.satellites) : "-";
    String hdop      = snap.hasHdop ? String(snap.hdop, 2)    : "-";

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    oledLine(0, "ETH GPS NTP");
    oledLine(1, Ethernet.localIP().toString());
    oledLine(2, gpsStatus + "  SAT:" + sats);
    oledLine(3, "HDOP:" + hdop);
    oledLine(4, "SRC: " + currentTimeSource);
    oledLine(5, timeBuf);
    display.display();

    if (DEBUG_MODE) {
      Serial.printf("[DISP] %s | %s | SAT:%s | HDOP:%s | %s\n",
                    Ethernet.localIP().toString().c_str(),
                    gpsStatus.c_str(), sats.c_str(), hdop.c_str(), timeBuf);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// =============================================================================
// SETUP HELPERS
// =============================================================================

void disableWiFi() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

static void setupOled() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    if (DEBUG_MODE) Serial.println("[OLED] Init failed");
    return;
  }
  display.clearDisplay();
  display.display();
  display.setRotation(2);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ETH GPS NTP");
  display.println("Booting...");
  display.display();
  if (DEBUG_MODE) Serial.println("[OLED] Ready");
}

static void setupEthernet() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ETH connecting...");
  display.display();

  disableWiFi();

  pinMode(PIN_ETH_CS, OUTPUT);
  digitalWrite(PIN_ETH_CS, HIGH);
  SPI.begin(PIN_ETH_SCK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
  SPI.setFrequency(ETH_SPI_FREQUENCY);
  Ethernet.init(PIN_ETH_CS);
  Ethernet.begin(ETH_MAC, ETH_IP, ETH_DNS, ETH_GATEWAY, ETH_SUBNET);
  delay(1000);

  if (DEBUG_MODE) {
    Serial.print("[ETH] IP: ");
    Serial.println(Ethernet.localIP());
  }
}

static void setupGps() {
  if (DEBUG_MODE) Serial.printf("[GPS] Probing %d baud...\n", GPS_BAUD_DEFAULT);
  gpsSerial.begin(GPS_BAUD_DEFAULT, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(300);
  while (gpsSerial.available()) gpsSerial.read();

  unsigned long t = millis();
  bool found = false;
  while (millis() - t < 2000) {
    if (gpsSerial.available() && gpsSerial.peek() == '$') { found = true; break; }
    if (gpsSerial.available()) gpsSerial.read();
  }
  if (!found && DEBUG_MODE) Serial.println("[GPS] WARNING: no NMEA detected");

  delay(200);
  while (gpsSerial.available()) gpsSerial.read();

  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), handlePpsInterrupt, RISING);
  gpio_set_intr_type((gpio_num_t)PIN_PPS, GPIO_INTR_POSEDGE);
  gpio_intr_enable((gpio_num_t)PIN_PPS);
  if (DEBUG_MODE) Serial.printf("[GPS] PPS interrupt registered on GPIO %d\n", PIN_PPS);
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================

void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] ESP32 ETH GPS NTP Server");
  }

  timeMutex = xSemaphoreCreateMutex();
  if (!timeMutex) { Serial.println("[BOOT] FATAL: timeMutex"); while (true); }

  gpsMutex = xSemaphoreCreateMutex();
  if (!gpsMutex) { Serial.println("[BOOT] FATAL: gpsMutex");  while (true); }

  setupOled();
  setupGps();
  setupEthernet();

  xTaskCreatePinnedToCore(gpsTask,     "gpsTask",     4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);

  xTaskCreatePinnedToCore(ntpTask,     "ntpTask",     4096, NULL,
                          configMAX_PRIORITIES - 2, NULL, 0);

  xTaskCreatePinnedToCore(displayTask, "displayTask", 8192, NULL,
                          1,                        NULL, 0);

  if (DEBUG_MODE) Serial.println("[BOOT] All tasks started");
}

void loop() {
  // Entire logic runs in FreeRTOS tasks.
  // Delete standard Arduino loop task to free stack.
  vTaskDelete(NULL);
}
