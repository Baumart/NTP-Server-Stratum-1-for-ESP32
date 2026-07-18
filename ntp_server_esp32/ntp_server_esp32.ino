// =============================================================================
// ntp_server_esp32.ino — Refactored Version
// ESP32 GPS+PPS Stratum-1 NTP Server (Ethernet W5500)
//
// Key improvements over original:
//  • Zero-mutex operations in ISR (no race conditions)
//  • Atomic timestamp updates with proper PPS anchor point
//  • Clean separation: GPS data collection ≠ timing anchor point
//  • RFC 3330 compliant NTP responses
//  • Minimal jitter in offset measurements
//
// Architecture:
//  Core 1 (gpsTask): NMEA parsing, GPS validation
//  Core 0 (ntpTask): NTP responses, synchronization
//  ISR: PPS timestamp capture (IRAM_ATTR, no locks)
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
// TYPES & STRUCTURES
// =============================================================================

struct PreciseTime {
  uint64_t seconds;      // Unix timestamp (since 1970)
  uint32_t microseconds; // Sub-second [0, 999999]
};

struct TimingState {
  uint64_t unixSec;      // Synchronization point (full second at PPS edge)
  uint64_t microsAtPps;  // esp_timer_get_time() when PPS fired
  uint8_t  quality;      // 0=none, 1=NTP, 2=GPS, 3=GPS+PPS
};

struct GpsSnapshot {
  bool     valid;
  uint32_t unixSec;
  uint8_t  hour, minute, second;
  uint8_t  satellites;
  float    hdop;
  uint32_t updateCount;
  // UBX-NAV-TIMEGPS (Class 0x01, ID 0x20)
  int8_t   leapS;       // GPS-UTC offset [s]; valid only if leapSValid
  bool     leapSValid;  // Bit 2 of valid-field; true = leapS decoded from nav-msg
};

struct LeapState {
  uint8_t leapIndicator;   // 0=no warning, 1=+1s, 2=-1s, 3=unsync
  bool valid;
  int8_t leapSeconds;      // GPS-UTC offset information
};

// =============================================================================
// CONFIGURATION
// =============================================================================

#define DEBUG_MODE true

// Ethernet (W5500)
#define PIN_ETH_CS   14
#define PIN_ETH_SCK  27
#define PIN_ETH_MISO 26
#define PIN_ETH_MOSI 25
#define ETH_SPI_FREQ 8000000
byte ETH_MAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ETH_IP     (10, 0, 0, 13);
IPAddress ETH_GATEWAY(10, 0, 0,  1);
IPAddress ETH_SUBNET (255, 255, 0, 0);
IPAddress ETH_DNS    (10, 0, 0,  1);

// GPS (NEO-6M)
#define PIN_GPS_RX      16
#define PIN_GPS_TX      17
#define GPS_BAUD        9600
#define GPS_MIN_VALID   1700000000UL

// PPS
#define PIN_PPS                32
#define PPS_STALE_US           2100000ULL
#define PPS_LOCK_CONFIRM_COUNT 3

// OLED (SSD1306)
#define PIN_OLED_SDA   23
#define PIN_OLED_SCL   18
#define OLED_WIDTH     128
#define OLED_HEIGHT     64
#define OLED_RESET      -1
#define OLED_ADDR  0x3C
#define OLED_LINE_H     10
// NTP
#define NTP_PORT        123
#define NTP_PACKET_SIZE 48
#define NTP_EPOCH_1900  2208988800UL
#define NTP_PREC_GPS    0xEC  // 2^-20 ≈ 1 µs
#define NTP_PREC_NMEA   0xF0  // 2^-16 ≈ 15 µs
#define NTP_PREC_NTP    0xF6  // 2^-10 ≈ 1 ms
#define NTP_DISP_GPS    0x08
#define NTP_DISP_NMEA   0x30
#define NTP_DISP_NTP    0x50

// =============================================================================
// VOLATILE STATE — ISR writes only
// =============================================================================

volatile uint64_t lastPpsMicros = 0;
volatile uint8_t  ppsCounter = 0;        // Count consecutive PPS pulses
volatile bool     ppsValid = false;       // Is PPS currently valid?
volatile uint64_t lastPpsSecTime = 0;    // When PPS became valid
static portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// SHARED STATE — Protected by mutexes
// =============================================================================

static SemaphoreHandle_t timingMutex;    // Protects timingState
static SemaphoreHandle_t gpsMutex;       // Protects GPS data

static TimingState timingState = {0, 0, 0};
static GpsSnapshot gpsSnap = {0};
static String timeSource = "NONE";       // Display only

static LeapState leapState = {
  3,      // Start: unsynchronized
  false,
  0
};

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================

HardwareSerial   gpsSerial(1);
TinyGPSPlus      gps;
EthernetUDP      ntpServerUDP;
EthernetUDP      ntpFallbackUDP;
NTPClient        ntpClient(ntpFallbackUDP, "pool.ntp.org", 0, 30000);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// =============================================================================
// PPS INTERRUPT — ZERO ALLOCATION, NO MUTEX
// =============================================================================

void IRAM_ATTR handlePpsInterrupt() {
  uint64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&ppsMux);

  // Ensure we have at least PPS_LOCK_CONFIRM_COUNT pulses before trusting
  if (ppsCounter < PPS_LOCK_CONFIRM_COUNT) {
    ppsCounter++;
    lastPpsMicros = now;
    if (ppsCounter == PPS_LOCK_CONFIRM_COUNT) {
      ppsValid = true;
      lastPpsSecTime = now;
      if (DEBUG_MODE) Serial.println("[PPS] Locked");
    }
  } else {
    lastPpsMicros = now;
  }
  portEXIT_CRITICAL_ISR(&ppsMux);
}

static uint64_t readLastPpsMicros() {
  portENTER_CRITICAL(&ppsMux);
  uint64_t val = lastPpsMicros;
  portEXIT_CRITICAL(&ppsMux);
  return val;
}

static bool isPpsStale() {
  if (!ppsValid) return true;
  return (esp_timer_get_time() - readLastPpsMicros()) > PPS_STALE_US;
}

// =============================================================================
// GPS HELPERS
// =============================================================================

static bool isGpsTimeValid() {
  return gps.date.isValid() && gps.time.isValid()
      && gps.date.year() >= 2024
      && gps.date.month() >= 1 && gps.date.month() <= 12
      && gps.date.day() >= 1 && gps.date.day() <= 31
      && gps.time.hour() <= 23
      && gps.time.minute() <= 59
      && gps.time.second() <= 60;
}

// Convert GPS date/time to Unix epoch (1970)
static uint32_t gpsToEpoch() {
  uint16_t y = gps.date.year();
  uint8_t  m = gps.date.month();
  uint8_t  d = gps.date.day();

  // Zeller algorithm variant
  int Y = y, M = m;
  if (M < 3) { Y--; M += 12; }
  uint32_t era   = Y / 400;
  uint32_t yoe   = Y - era * 400;
  uint32_t doy   = (153 * (M - 3) + 2) / 5 + d - 1;
  uint32_t doe   = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  uint32_t days  = era * 146097 + doe - 719468;

  uint32_t epoch = days * 86400UL
                 + gps.time.hour() * 3600UL
                 + gps.time.minute() * 60UL
                 + gps.time.second();

  return (epoch > GPS_MIN_VALID) ? epoch : 0;
}

// =============================================================================
// TIME INTERPOLATION
// =============================================================================

static PreciseTime getPreciseTime() {
  TimingState snap = timingState;
  uint64_t now = esp_timer_get_time();
  uint64_t elapsed = now - snap.microsAtPps;

  PreciseTime pt;
  pt.seconds = snap.unixSec + elapsed / 1000000ULL;
  pt.microseconds = (uint32_t)(elapsed % 1000000ULL);
  return pt;
}

static PreciseTime lastKnownGoodPt = {0, 0};

static PreciseTime getPreciseTimeSafe() {
  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    PreciseTime pt = getPreciseTime();
    lastKnownGoodPt = pt;
    xSemaphoreGive(timingMutex);
    return pt;
  }
  // Fallback: last known state (stale but safe no torn-read)
  return lastKnownGoodPt;
}

// =============================================================================
// SYNCHRONIZATION LOGIC
// =============================================================================

static void syncWithGps() {
  // Called under timingMutex
  uint32_t gpsEpoch = 0;
  bool gpsValid = false;

  // Take GPS snapshot
  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (gpsSnap.valid) {
      gpsEpoch = gpsSnap.unixSec;
      gpsValid = true;
    }
    xSemaphoreGive(gpsMutex);
  }

  // Priority:
  // 1. GPS+PPS: sync at next PPS pulse with GPS+1 second
  // 2. GPS only: sync now with GPS time
  // 3. Keep existing sync if no new data

  static uint32_t lastSyncGpsEpoch = 0;

  if (gpsValid && ppsValid && !isPpsStale()) {
    if (gpsEpoch != lastSyncGpsEpoch) {
      // Neue Sekunde von NMEA bestätigt → PPS passt garantiert dazu
      lastSyncGpsEpoch = gpsEpoch;
      timingState.unixSec = gpsEpoch;
      timingState.microsAtPps = readLastPpsMicros();
      timingState.quality = 3;
    }
    // else: NMEA noch nicht aktualisiert → alten Sync behalten, nicht anfassen
  } else if (gpsValid) {
    timingState.unixSec = gpsEpoch;
    timingState.microsAtPps = esp_timer_get_time();
    timingState.quality = 2;
  }
}

// =============================================================================
// NTP PACKET HELPERS
// =============================================================================

static void writeU32BE(uint8_t* buf, int off, uint32_t v) {
  buf[off + 0] = (v >> 24) & 0xFF;
  buf[off + 1] = (v >> 16) & 0xFF;
  buf[off + 2] = (v >>  8) & 0xFF;
  buf[off + 3] = v & 0xFF;
}

static void writeU64BE(uint8_t* buf, int off, uint64_t v) {
  buf[off + 0] = (v >> 56) & 0xFF;
  buf[off + 1] = (v >> 48) & 0xFF;
  buf[off + 2] = (v >> 40) & 0xFF;
  buf[off + 3] = (v >> 32) & 0xFF;
  buf[off + 4] = (v >> 24) & 0xFF;
  buf[off + 5] = (v >> 16) & 0xFF;
  buf[off + 6] = (v >>  8) & 0xFF;
  buf[off + 7] = v & 0xFF;
}

static uint64_t preciseTimeToNtp64(const PreciseTime& pt) {
  uint64_t sec1900 = pt.seconds + (uint64_t)NTP_EPOCH_1900;
  // Convert microseconds to NTP fractional part: µs * 2^32 / 1000000
  uint32_t frac = (uint32_t)(((uint64_t)pt.microseconds * 4294967296ULL) / 1000000ULL);
  return (sec1900 << 32) | frac;
}

// =============================================================================
// NTP RESPONSE
// =============================================================================

static uint8_t ntpPacketBuffer[NTP_PACKET_SIZE];

static void sendNtpResponse(IPAddress remoteIp, uint16_t remotePort, uint8_t* request) {
  // Capture receive time FIRST
  PreciseTime recvTime = getPreciseTimeSafe();

  // Get current quality
  uint8_t quality = 0;
  uint64_t refSyncSec = 0;
  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    quality = timingState.quality;
    refSyncSec = timingState.unixSec;
    xSemaphoreGive(timingMutex);
  }

  bool usingGps = (quality >= 2);
  bool usingPps = (quality == 3) && ppsValid;

  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);

  uint8_t li = 3; // default: unsynchronized

  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    li = leapState.leapIndicator;
    xSemaphoreGive(timingMutex);
  }

  // LI | VN=4 | Mode=4
  ntpPacketBuffer[0] = (li << 6) | (4 << 3) | 4;
  // Byte 1: Stratum
  ntpPacketBuffer[1] = usingGps ? 1 : (quality == 1 ? 2 : 16);
  // Byte 2: Poll (informational)
  ntpPacketBuffer[2] = 4;
  // Byte 3: Precision
  ntpPacketBuffer[3] = usingPps ? NTP_PREC_GPS
                      : usingGps ? NTP_PREC_NMEA
                      : NTP_PREC_NTP;

  // Root delay (us)
  writeU32BE(ntpPacketBuffer, 4, 0);
  // Root dispersion (us)
  writeU32BE(ntpPacketBuffer, 8, usingPps ? NTP_DISP_GPS
                                : usingGps ? NTP_DISP_NMEA
                                : NTP_DISP_NTP);

  // Reference clock ID
  if (usingGps) {
    ntpPacketBuffer[12] = 'G';
    ntpPacketBuffer[13] = 'P';
    ntpPacketBuffer[14] = 'S';
  } else {
    ntpPacketBuffer[12] = 'N';
    ntpPacketBuffer[13] = 'T';
    ntpPacketBuffer[14] = 'P';
  }
  ntpPacketBuffer[15] = 0;

  // Reference timestamp (last sync time, seconds only for stability)
  {
    //uint64_t refSec = timingState.unixSec + NTP_EPOCH_1900;
    uint64_t refSec = refSyncSec + NTP_EPOCH_1900;
    writeU64BE(ntpPacketBuffer, 16, refSec << 32);
  }

  // Origin timestamp (copy from request)
  memcpy(&ntpPacketBuffer[24], &request[40], 8);

  // Receive timestamp
  writeU64BE(ntpPacketBuffer, 32, preciseTimeToNtp64(recvTime));

  // Transmit timestamp (capture NOW)
  PreciseTime xmitTime = getPreciseTimeSafe();
  writeU64BE(ntpPacketBuffer, 40, preciseTimeToNtp64(xmitTime));

  ntpServerUDP.beginPacket(remoteIp, remotePort);
  ntpServerUDP.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  ntpServerUDP.endPacket();

  if (DEBUG_MODE) {
    Serial.printf("[NTP] → %s:%u (src=%s, pps=%s)\n",
                  remoteIp.toString().c_str(), remotePort,
                  timeSource.c_str(), usingPps ? "yes" : "no");
  }
}

// =============================================================================
// FREERTOS TASKS
// =============================================================================
// =============================================================================
// UBX FRAME PARSER — state machine, no heap allocation
// Processes interleaved binary UBX frames from gpsSerial alongside NMEA.
// TinyGPSPlus silently ignores non-'$' bytes, so dual-feeding is safe.
// =============================================================================

enum UbxState : uint8_t {
  UBX_IDLE, UBX_SYNC2, UBX_CLASS, UBX_ID,
  UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB
};
static UbxState  ubxState  = UBX_IDLE;
static uint8_t   ubxClass  = 0, ubxId = 0;
static uint16_t  ubxLen    = 0, ubxIdx = 0;
static uint8_t   ubxCalcA  = 0, ubxCalcB = 0;
static uint8_t   ubxRxCkA  = 0;
// NAV-TIMEGPS payload is 16 bytes; buffer capped to guard against large frames
static uint8_t   ubxBuf[20];

// Returns true if a complete, checksum-valid NAV-TIMEGPS frame was received.
// All other valid UBX frames are silently consumed without signalling.
static bool feedUbx(uint8_t b) {
  switch (ubxState) {
    case UBX_IDLE:
      if (b == 0xB5) ubxState = UBX_SYNC2;
      break;
    case UBX_SYNC2:
      ubxState = (b == 0x62) ? UBX_CLASS : UBX_IDLE;
      break;
    case UBX_CLASS:
      ubxClass = b; ubxCalcA = b; ubxCalcB = b;
      ubxState = UBX_ID;
      break;
    case UBX_ID:
      ubxId = b; ubxCalcA += b; ubxCalcB += ubxCalcA;
      ubxState = UBX_LEN1;
      break;
    case UBX_LEN1:
      ubxLen = b; ubxCalcA += b; ubxCalcB += ubxCalcA;
      ubxState = UBX_LEN2;
      break;
    case UBX_LEN2:
      ubxLen |= ((uint16_t)b << 8); ubxCalcA += b; ubxCalcB += ubxCalcA;
      ubxIdx  = 0;
      ubxState = (ubxLen > 0) ? UBX_PAYLOAD : UBX_CKA;
      break;
    case UBX_PAYLOAD:
      if (ubxIdx < sizeof(ubxBuf)) ubxBuf[ubxIdx] = b;
      ubxCalcA += b; ubxCalcB += ubxCalcA;
      if (++ubxIdx >= ubxLen) ubxState = UBX_CKA;
      break;
    case UBX_CKA:
      ubxRxCkA = b; ubxState = UBX_CKB;
      break;
    case UBX_CKB:
      ubxState = UBX_IDLE;
      return (ubxClass == 0x01 && ubxId == 0x20
              && ubxLen == 16
              && ubxRxCkA == ubxCalcA && b == ubxCalcB);
  }
  return false;
}

// Called under gpsMutex after feedUbx() returns true.
static void parseUbxNavTimeGps(GpsSnapshot& snap) {
  // Payload layout (u-blox 6 Protocol Spec, Table NAV-TIMEGPS):
  //   Byte 10: leapS   (I1) — GPS-UTC leap seconds
  //   Byte 11: valid   (X1) — bit2 = leapSValid
  if (ubxLen < 12) return;
  snap.leapS      = (int8_t)ubxBuf[10];
  snap.leapSValid = (ubxBuf[11] & 0x04) != 0;
}

static void updateLeapState() {
  // Step 1: Read GPS leap data under gpsMutex (copy to locals, no nesting)
  int8_t leapS      = 0;
  bool   leapSValid = false;
  bool   gpsValid   = false;

  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    gpsValid   = gpsSnap.valid;
    leapS      = gpsSnap.leapS;
    leapSValid = gpsSnap.leapSValid;
    xSemaphoreGive(gpsMutex);
  } else {
    return; // can't read GPS state, leave leapState unchanged
  }

  // Step 2: Update leapState under timingMutex
  // Lock ordering: gpsMutex released above → safe to acquire timingMutex.
  // (syncWithGps uses timingMutex→gpsMutex; no inversion possible here.)
  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;

  if (timingState.quality == 0) {
    // No time source at all
    leapState.leapIndicator = 3;
    leapState.valid         = false;

  } else if (gpsValid && leapSValid) {
    // GPS synchronized and leap second offset confirmed by navigation message.
    // NEO-6M (u-blox 6) provides no tLSF/WNlsf (future event fields).
    // LI=1 or LI=2 require advance knowledge of the event → not possible here.
    // LI=0 is therefore the only correct value: synchronized, no event announced.
    leapState.leapIndicator = 0;
    leapState.valid         = true;
    leapState.leapSeconds   = leapS;

  } else {
    // GPS fix present but leapS not yet decoded from nav subframe
    leapState.leapIndicator = 3;
    leapState.valid         = false;
  }

  xSemaphoreGive(timingMutex);
}

void gpsTask(void* param) {
  if (DEBUG_MODE) Serial.println("[gpsTask] Started");

  for (;;) {
    // Feed UART bytes to TinyGPS
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    // Update GPS snapshot under gpsMutex
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (isGpsTimeValid()) {
        gpsSnap.valid = true;
        gpsSnap.unixSec = gpsToEpoch();
        gpsSnap.hour = gps.time.hour();
        gpsSnap.minute = gps.time.minute();
        gpsSnap.second = gps.time.second();
        gpsSnap.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
        gpsSnap.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
        gpsSnap.updateCount++;
      } else {
        gpsSnap.valid = false;
      }
      xSemaphoreGive(gpsMutex);
    }
    // LeapSec
    updateLeapState();

    // Attempt synchronization
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      syncWithGps();
      xSemaphoreGive(timingMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ntpTask(void* param) {
  if (DEBUG_MODE) Serial.println("[ntpTask] Started");

  // Start NTP fallback client
  ntpClient.begin();
  ntpClient.setTimeOffset(0);
  ntpClient.forceUpdate();

  // Start NTP server
  ntpServerUDP.begin(NTP_PORT);
  if (DEBUG_MODE) Serial.println("[NTP] Listening on UDP 123");

  // Initial seed from fallback NTP
  if (ntpClient.isTimeSet()) {
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      timingState.unixSec = ntpClient.getEpochTime();
      timingState.microsAtPps = esp_timer_get_time();
      timingState.quality = 1;
      xSemaphoreGive(timingMutex);
      if (DEBUG_MODE) Serial.printf("[NTP] Seeded: %llu\n", timingState.unixSec);
    }
  }

  unsigned long lastNtpSync = millis();

  for (;;) {
    // Process incoming NTP requests
    int packetSize = ntpServerUDP.parsePacket();
    if (packetSize == NTP_PACKET_SIZE) {
      IPAddress remoteIp = ntpServerUDP.remoteIP();
      uint16_t remotePort = ntpServerUDP.remotePort();
      uint8_t req[NTP_PACKET_SIZE];
      ntpServerUDP.read(req, NTP_PACKET_SIZE);
      sendNtpResponse(remoteIp, remotePort, req);
    } else if (packetSize > 0) {
      ntpServerUDP.flush();
      if (DEBUG_MODE) Serial.printf("[NTP] Malformed packet (%d B)\n", packetSize);
    }

    // Periodic fallback sync (FIXED: increased interval + only when no GPS)
    // ISSUE: 30s updates caused spikes every 70-80s when NTP would fail over
    // SOLUTION: Only sync if GPS unavailable, use 60s interval instead
    uint8_t currentQuality = 0;
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      currentQuality = timingState.quality;
      xSemaphoreGive(timingMutex);
    }

    if (currentQuality < 2 && millis() - lastNtpSync > 60000) {
      lastNtpSync = millis();
      if (DEBUG_MODE) Serial.println("[NTP] Fallback sync attempt (GPS unavailable)");
      ntpClient.update();

      if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        currentQuality = timingState.quality;
        if (currentQuality < 2 && ntpClient.isTimeSet()) {
          timingState.unixSec = ntpClient.getEpochTime();
          timingState.microsAtPps = esp_timer_get_time();
          timingState.quality = 1;
          if (DEBUG_MODE) Serial.printf("[NTP] Fallback sync OK: %llu\n", timingState.unixSec);
        }
        xSemaphoreGive(timingMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void oledLine(int line, const String& text) {
  display.setCursor(0, line * OLED_LINE_H);
  display.println(text);
}

void displayTask(void* param) {
  if (DEBUG_MODE) Serial.println("[displayTask] Started");

  for (;;) {
    PreciseTime pt = getPreciseTimeSafe();

    // Get GPS snapshot
    GpsSnapshot snap = {0};
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      snap = gpsSnap;
      xSemaphoreGive(gpsMutex);
    }

    // Get quality
    uint8_t quality = 0;
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      quality = timingState.quality;
      xSemaphoreGive(timingMutex);
    }

    // Format time
    char timeBuf[22];
    time_t t = (time_t)pt.seconds;
    struct tm* tm = gmtime(&t);
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d.%06u%s",
             tm->tm_hour, tm->tm_min, tm->tm_sec,
             pt.microseconds, ppsValid ? "" : "~");

    // Update display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    oledLine(0, "ETH GPS NTP Server");
    oledLine(1, "IP: " + Ethernet.localIP().toString());
    oledLine(2, snap.valid ? "GPS OK" : "GPS WAIT");
    oledLine(3, "SAT:" + String(snap.satellites) + " HDOP:" + String(snap.hdop,1));

    if (quality == 3) {
      oledLine(4,"SRC: GPS+PPS");
    } else if (quality == 2) {
      oledLine(4, "SRC: GPS");
    } else if (quality == 1) {
      oledLine(4, "SRC: NTP");
    } else {
      oledLine(4, "SRC: NONE");
    }

    oledLine(5, timeBuf);
    display.display();

    if (DEBUG_MODE) {
      Serial.printf("[DISP] SAT:%d | HDOP:%.1f | %s\n",
                    snap.satellites, snap.hdop, timeBuf);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// =============================================================================
// SETUP HELPERS
// =============================================================================

static void setupOled() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
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
  pinMode(PIN_ETH_CS, OUTPUT);
  digitalWrite(PIN_ETH_CS, HIGH);
  SPI.begin(PIN_ETH_SCK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
  SPI.setFrequency(ETH_SPI_FREQ);
  Ethernet.init(PIN_ETH_CS);
  Ethernet.begin(ETH_MAC, ETH_IP, ETH_DNS, ETH_GATEWAY, ETH_SUBNET);
  delay(1000);
  if (DEBUG_MODE) {
    Serial.print("[ETH] IP: ");
    Serial.println(Ethernet.localIP());
  }
}

static void setupGps() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(300);
  // Feed serial bytes to both TinyGPSPlus (NMEA) and the UBX state machine.
  // TinyGPSPlus ignores non-'$' bytes, so dual-feeding is safe.
  while (gpsSerial.available()) {
    uint8_t b = (uint8_t)gpsSerial.read();
    gps.encode((char)b);

    if (feedUbx(b)) {
      // Complete, checksum-valid NAV-TIMEGPS frame
      if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        parseUbxNavTimeGps(gpsSnap);
        xSemaphoreGive(gpsMutex);
        if (DEBUG_MODE)
          Serial.printf("[GPS] leapS=%d valid=%d\n", gpsSnap.leapS, gpsSnap.leapSValid);
      }
    }
  }
  delay(200);
  while (gpsSerial.available()) gpsSerial.read();

  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), handlePpsInterrupt, RISING);
  if (DEBUG_MODE) Serial.printf("[GPS] PPS on GPIO %d\n", PIN_PPS);
  // Enable UBX-NAV-TIMEGPS @ 1 Hz on UART1 (CFG-MSG, 8-byte payload)
  // Checksum covers bytes [class..payload]: CK_A=0x31, CK_B=0x90
  static const uint8_t ubxEnableNavTimeGps[] = {
    0xB5, 0x62,              // sync
    0x06, 0x01,              // class=CFG, id=MSG
    0x08, 0x00,              // payload length
    0x01, 0x20,              // msgClass=NAV, msgId=TIMEGPS
    0x00,                    // I2C   rate = 0
    0x01,                    // UART1 rate = 1 (1 frame per epoch)
    0x00, 0x00, 0x00, 0x00,  // UART2, USB, SPI, reserved
    0x31, 0x90               // CK_A, CK_B
  };
  gpsSerial.write(ubxEnableNavTimeGps, sizeof(ubxEnableNavTimeGps));
  delay(100);
  if (DEBUG_MODE) Serial.println("[GPS] UBX NAV-TIMEGPS enabled @ 1 Hz");
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  if (DEBUG_MODE) Serial.println("[BOOT] ESP32 ETH GPS NTP Server (Refactored)");

  // Create mutexes
  timingMutex = xSemaphoreCreateMutex();
  gpsMutex = xSemaphoreCreateMutex();
  if (!timingMutex || !gpsMutex) {
    Serial.println("[BOOT] FATAL: Mutex creation failed");
    while (true);
  }

  // Disable WiFi
  WiFi.mode(WIFI_OFF);
  btStop();

  // Setup hardware
  setupOled();
  setupGps();
  setupEthernet();

  // Create tasks
  xTaskCreatePinnedToCore(gpsTask,     "gpsTask",     4096, NULL,
                          configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(ntpTask,     "ntpTask",     4096, NULL,
                          configMAX_PRIORITIES - 2, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "displayTask", 8192, NULL,
                          1, NULL, 0);

  if (DEBUG_MODE) Serial.println("[BOOT] All tasks started");
}

void loop() {
  vTaskDelete(NULL);
}
