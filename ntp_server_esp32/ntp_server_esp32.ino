// =============================================================================
// ntp_server_esp32.ino
// ESP32 GPS+PPS Stratum-1 NTP Server (Ethernet W5500, u-blox M9N)
//
// UBX messages used (all @ 1 Hz, enabled after 5 s startup delay):
//   NAV-TIMEGPS (0x01/0x20) — GPS-UTC leap seconds, leapSValid
//   NAV-CLOCK   (0x01/0x22) — clock bias, drift, tAcc → dynamic root dispersion
//   NAV-STATUS  (0x01/0x03) — fix type (noFix/2D/3D/timeOnly), gpsFixOk
//   NAV-TIMELS  (0x01/0x26) — leap second advance notice (future event dates/times)
//   MON-RF      (0x0A/0x38) — jamming state, antenna status
//
// RFC 5905 compliance:
//   LI=0 when leapSValid and no future event; LI=3 otherwise or on critical jamming.
//   LI=1 when future positive leap second (lsChange=+1, timeToLsEvent>0).
//   LI=2 when future negative leap second (lsChange=-1, timeToLsEvent>0).
//   Root dispersion computed from tAcc [ns] → NTP short format (16.16 s).
//   Stratum=16 on critical jamming.
//
// Mutex ordering (never inverted):
//   syncWithGps:    timingMutex (held) → gpsMutex (inner, brief)
//   updateLeapState: gpsMutex (release) → timingMutex (sequential, not nested)
//
// Core assignment:
//   Core 1: gpsTask  (highest priority — latency-sensitive UBX/NMEA parsing)
//   Core 0: ntpTask, displayTask
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
// TYPES
// =============================================================================

struct PreciseTime {
  uint64_t seconds;       // Unix epoch
  uint32_t microseconds;  // Sub-second [0, 999999]
};

struct TimingState {
  uint64_t unixSec;        // Sync anchor (full second at PPS edge)
  uint64_t microsAtPps;    // esp_timer_get_time() at PPS
  uint8_t  quality;        // 0=none 1=NTP 2=GPS 3=GPS+PPS
  uint32_t tAccNs;         // Time accuracy from NAV-CLOCK [ns]; 0=unknown
  uint8_t  jammingState;   // From MON-RF: 0=unk 1=ok 2=warn 3=crit
};

struct GpsSnapshot {
  // NMEA (TinyGPSPlus)
  bool     valid;
  uint32_t unixSec;
  uint8_t  hour, minute, second;
  uint8_t  satellites;
  float    hdop;
  uint32_t updateCount;
  // NAV-TIMEGPS (0x01/0x20)
  int8_t   leapS;          // GPS-UTC offset [s]
  bool     leapSValid;     // valid-field bit 2
  // NAV-CLOCK (0x01/0x22)
  int32_t  clkB;           // Clock bias  [ns]
  int32_t  clkD;           // Clock drift [ns/s]
  uint32_t tAcc;           // Time accuracy estimate [ns]
  uint32_t fAcc;           // Frequency accuracy [ps/s]
  bool     clkValid;
  // NAV-STATUS (0x01/0x03)
  uint8_t  gpsFix;         // 0=noFix 2=2D 3=3D 5=timeOnly
  bool     gpsFixOk;       // flags bit0: fix within DOP/ACC masks
  // MON-RF (0x0A/0x38) block 0
  uint8_t  jammingState;   // 0=unk 1=ok 2=warn 3=crit
  uint8_t  antStatus;      // 0=init 1=dontKnow 2=ok 3=short 4=open
  // NAV-TIMELS (0x01/0x26) — leap second advance notice
  int8_t   currLs;         // Current leap seconds (GPS-UTC) [s]
  bool     currLsValid;    // validCurrLs flag from valid byte
  int8_t   lsChange;       // Future leap second change: +1, -1, or 0 [s]
  bool     lsChangeValid;  // validTimeToLsEvent flag from valid byte
  int32_t  timeToLsEvent;  // Seconds until next LS event (can be negative) [s]
  uint16_t dateOfLsGpsWn;  // GPS week number of LS event
  uint16_t dateOfLsGpsDn;  // GPS day-of-week number of LS event
};

struct LeapState {
  uint8_t leapIndicator;   // NTP LI field: 0=ok 1=+1s 2=-1s 3=unsync
  bool    valid;
  int8_t  leapSeconds;     // GPS-UTC offset [s]
};

// =============================================================================
// CONFIGURATION
// =============================================================================

#define DEBUG_MODE true

// Ethernet W5500
#define PIN_ETH_CS    14
#define PIN_ETH_SCK   27
#define PIN_ETH_MISO  26
#define PIN_ETH_MOSI  25
#define ETH_SPI_FREQ  8000000
byte      ETH_MAC[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ETH_IP     (10, 0, 0, 13);
IPAddress ETH_GATEWAY(10, 0, 0,  1);
IPAddress ETH_SUBNET (255, 255, 0, 0);
IPAddress ETH_DNS    (10, 0, 0,  1);

// GPS u-blox M9N
#define PIN_GPS_RX     16
#define PIN_GPS_TX     17
#define GPS_BAUD       38400
#define GPS_MIN_VALID  1700000000UL

// PPS
#define PIN_PPS                 32
#define PPS_STALE_US            2100000ULL
#define PPS_LOCK_CONFIRM_COUNT  3

// OLED SSD1306
#define PIN_OLED_SDA  23
#define PIN_OLED_SCL  18
#define OLED_WIDTH    128
#define OLED_HEIGHT    64
#define OLED_RESET     -1
#define OLED_ADDR  0x3C
#define OLED_LINE_H    10

// NTP
#define NTP_PORT        123
#define NTP_PACKET_SIZE  48
#define NTP_EPOCH_1900  2208988800UL
#define NTP_PREC_GPS    0xEC   // 2^-20 ≈ 1 µs  (GPS+PPS)
#define NTP_PREC_NMEA   0xF0   // 2^-16 ≈ 15 µs (GPS only)
#define NTP_PREC_NTP    0xF6   // 2^-10 ≈ 1 ms  (NTP fallback)
// Static root dispersion fallbacks (NTP short 16.16 format):
#define NTP_DISP_GPS    0x0008  // ≈ 122 µs — used when tAcc=0
#define NTP_DISP_NMEA   0x0030  // ≈ 732 µs
#define NTP_DISP_NTP    0x0050  // ≈ 1.2 ms

// =============================================================================
// VOLATILE STATE — ISR writes only
// =============================================================================

volatile uint64_t    lastPpsMicros  = 0;
volatile uint8_t     ppsCounter     = 0;
volatile bool        ppsValid       = false;
volatile uint64_t    lastPpsSecTime = 0;
static portMUX_TYPE  ppsMux         = portMUX_INITIALIZER_UNLOCKED;

// UBX message watchdog — gpsTask detects silent CFG-MSG failures
volatile uint32_t ubxMsgTimestamps[4] = { 0, 0, 0, 0 };  // [0]=TIMEGPS, [1]=CLOCK, [2]=STATUS, [3]=TIMELS
volatile bool ubxMsgReceived = false;

// =============================================================================
// SHARED STATE — mutex-protected
// =============================================================================

static SemaphoreHandle_t timingMutex;   // guards timingState, leapState
static SemaphoreHandle_t gpsMutex;      // guards gpsSnap

static TimingState timingState = { 0, 0, 0, 0, 0 };
static GpsSnapshot gpsSnap     = {};
static LeapState   leapState   = { 3, false, 0 };

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
// PPS ISR — IRAM_ATTR, portMUX only (no FreeRTOS primitives)
// =============================================================================

void IRAM_ATTR handlePpsInterrupt() {
  uint64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&ppsMux);
  if (ppsCounter < PPS_LOCK_CONFIRM_COUNT) {
    ppsCounter++;
    lastPpsMicros = now;
    if (ppsCounter == PPS_LOCK_CONFIRM_COUNT) {
      ppsValid       = true;
      lastPpsSecTime = now;
    }
  } else {
    lastPpsMicros = now;
  }
  portEXIT_CRITICAL_ISR(&ppsMux);
}

static uint64_t readLastPpsMicros() {
  portENTER_CRITICAL(&ppsMux);
  uint64_t v = lastPpsMicros;
  portEXIT_CRITICAL(&ppsMux);
  return v;
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
      && gps.date.year()   >= 2024
      && gps.date.month()  >= 1  && gps.date.month()  <= 12
      && gps.date.day()    >= 1  && gps.date.day()    <= 31
      && gps.time.hour()   <= 23
      && gps.time.minute() <= 59
      && gps.time.second() <= 60;
}

static uint32_t gpsToEpoch() {
  int Y = gps.date.year();
  int M = gps.date.month();
  int d = gps.date.day();
  if (M < 3) { Y--; M += 12; }
  uint32_t era  = (uint32_t)Y / 400;
  uint32_t yoe  = (uint32_t)Y - era * 400;
  uint32_t doy  = (153 * (M - 3) + 2) / 5 + d - 1;
  uint32_t doe  = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  uint32_t days = era * 146097 + doe - 719468;
  uint32_t epoch = days * 86400UL
                 + gps.time.hour()   * 3600UL
                 + gps.time.minute() * 60UL
                 + gps.time.second();
  return (epoch > GPS_MIN_VALID) ? epoch : 0;
}

// =============================================================================
// TIME INTERPOLATION
// =============================================================================

static PreciseTime getPreciseTime() {
  // Called under timingMutex
  TimingState s   = timingState;
  uint64_t elapsed = esp_timer_get_time() - s.microsAtPps;
  PreciseTime pt;
  pt.seconds      = s.unixSec + elapsed / 1000000ULL;
  pt.microseconds = (uint32_t)(elapsed % 1000000ULL);
  return pt;
}

static PreciseTime lastKnownGoodPt = { 0, 0 };

static PreciseTime getPreciseTimeSafe() {
  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    PreciseTime pt  = getPreciseTime();
    lastKnownGoodPt = pt;
    xSemaphoreGive(timingMutex);
    return pt;
  }
  return lastKnownGoodPt;  // stale but no torn read
}

// =============================================================================
// SYNCHRONIZATION
// =============================================================================

static void syncWithGps() {
  // Must be called under timingMutex.
  // Takes gpsMutex briefly (inner lock — no inversion risk).
  uint32_t gpsEpoch    = 0;
  bool     gpsValid    = false;
  uint32_t tAccNs      = 0;
  uint8_t  jammingState = 0;

  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    gpsValid      = gpsSnap.valid;
    gpsEpoch      = gpsSnap.unixSec;
    tAccNs        = gpsSnap.tAcc;
    jammingState  = gpsSnap.jammingState;
    xSemaphoreGive(gpsMutex);
  }

  // Always propagate telemetry fields — independent of sync decision
  timingState.tAccNs       = tAccNs;
  timingState.jammingState = jammingState;

  static uint32_t lastSyncGpsEpoch = 0;

  if (gpsValid && ppsValid && !isPpsStale()) {
    // Gate on NMEA epoch advancing: ensures PPS and GPS epoch are aligned.
    // Without this gate a new PPS paired with the previous NMEA second causes
    // a ±1 s spike (the original bug).
    if (gpsEpoch != lastSyncGpsEpoch) {
      lastSyncGpsEpoch        = gpsEpoch;
      timingState.unixSec     = gpsEpoch;
      timingState.microsAtPps = readLastPpsMicros();
      timingState.quality     = 3;
    }
    // else: NMEA not yet updated for this epoch — keep existing sync
  } else if (gpsValid) {
    timingState.unixSec     = gpsEpoch;
    timingState.microsAtPps = esp_timer_get_time();
    timingState.quality     = 2;
  }
}

// =============================================================================
// NTP PACKET HELPERS
// =============================================================================

static void writeU32BE(uint8_t* buf, int off, uint32_t v) {
  buf[off + 0] = (v >> 24) & 0xFF;
  buf[off + 1] = (v >> 16) & 0xFF;
  buf[off + 2] = (v >>  8) & 0xFF;
  buf[off + 3] =  v        & 0xFF;
}

static void writeU64BE(uint8_t* buf, int off, uint64_t v) {
  for (int i = 0; i < 8; i++)
    buf[off + i] = (v >> (56 - 8 * i)) & 0xFF;
}

static uint64_t preciseTimeToNtp64(const PreciseTime& pt) {
  uint64_t sec1900 = pt.seconds + (uint64_t)NTP_EPOCH_1900;
  uint32_t frac    = (uint32_t)(((uint64_t)pt.microseconds * 4294967296ULL) / 1000000ULL);
  return (sec1900 << 32) | frac;
}

// =============================================================================
// NTP RESPONSE — RFC 5905
// =============================================================================

static uint8_t ntpPacketBuffer[NTP_PACKET_SIZE];

static void sendNtpResponse(IPAddress remoteIp, uint16_t remotePort, uint8_t* request) {
  // Capture receive timestamp first — minimises T2 measurement error
  PreciseTime recvTime = getPreciseTimeSafe();

  // Read all timingMutex-protected state in a single critical section
  uint8_t  quality      = 0;
  uint64_t refSyncSec   = 0;
  uint32_t tAccNs       = 0;
  uint8_t  jammingState = 0;
  uint8_t  li           = 3;

  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    quality      = timingState.quality;
    refSyncSec   = timingState.unixSec;
    tAccNs       = timingState.tAccNs;
    jammingState = timingState.jammingState;
    li           = leapState.leapIndicator;   // also under timingMutex
    xSemaphoreGive(timingMutex);
  }

  const bool jamming  = (jammingState == 3);
  const bool usingGps = (quality >= 2) && !jamming;
  const bool usingPps = (quality == 3) && ppsValid && !jamming;

  // Critical jamming overrides LI regardless of leapState
  if (jamming) li = 3;

  // Dynamic root dispersion from NAV-CLOCK tAcc [ns] → NTP short 16.16 [s]
  // Formula: d = tAccNs * 65536 / 1e9   (floor at 1 ≈ 15 µs, cap at 0xFFFF ≈ 1 s)
  uint32_t rootDisp;
  if (usingPps && tAccNs > 0) {
    uint64_t d = ((uint64_t)tAccNs * 65536ULL + 500000000ULL) / 1000000000ULL;
    if (d < 1)      d = 1;
    if (d > 0xFFFF) d = 0xFFFF;
    rootDisp = (uint32_t)d;
  } else {
    rootDisp = usingPps ? NTP_DISP_GPS : usingGps ? NTP_DISP_NMEA : NTP_DISP_NTP;
  }

  const uint8_t stratum = jamming   ? 16
                        : usingGps  ?  1
                        : quality == 1 ? 2 : 16;

  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);

  ntpPacketBuffer[0] = (li << 6) | (4 << 3) | 4;  // LI | VN=4 | Mode=4 (server)
  ntpPacketBuffer[1] = stratum;
  ntpPacketBuffer[2] = 4;                            // Poll exponent (informational)
  ntpPacketBuffer[3] = usingPps ? NTP_PREC_GPS
                     : usingGps ? NTP_PREC_NMEA
                     : NTP_PREC_NTP;

  writeU32BE(ntpPacketBuffer,  4, 0);         // Root delay (0: direct GPS)
  writeU32BE(ntpPacketBuffer,  8, rootDisp);  // Root dispersion

  // Reference identifier
  if (usingGps) {
    ntpPacketBuffer[12] = 'G';
    ntpPacketBuffer[13] = 'P';
    ntpPacketBuffer[14] = 'S';
    ntpPacketBuffer[15] = 0;
  } else if (quality == 1) {
    ntpPacketBuffer[12] = 'N';
    ntpPacketBuffer[13] = 'T';
    ntpPacketBuffer[14] = 'P';
    ntpPacketBuffer[15] = 0;
  }
  // stratum=16: ref-id left 0x00000000 (undefined per RFC 5905 §7.3)

  // Reference timestamp (integer seconds only for stability)
  writeU64BE(ntpPacketBuffer, 16, (refSyncSec + NTP_EPOCH_1900) << 32);

  // Origin timestamp = client transmit (copy verbatim from request[40..47])
  memcpy(&ntpPacketBuffer[24], &request[40], 8);

  // Receive timestamp (captured above)
  writeU64BE(ntpPacketBuffer, 32, preciseTimeToNtp64(recvTime));

  // Transmit timestamp (as late as possible)
  PreciseTime xmitTime = getPreciseTimeSafe();
  writeU64BE(ntpPacketBuffer, 40, preciseTimeToNtp64(xmitTime));

  ntpServerUDP.beginPacket(remoteIp, remotePort);
  ntpServerUDP.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  ntpServerUDP.endPacket();

  if (DEBUG_MODE)
    Serial.printf("[NTP] → %s:%u  st=%u li=%u disp=0x%04X tAcc=%uns\n",
                  remoteIp.toString().c_str(), remotePort,
                  stratum, li, rootDisp, tAccNs);
}

// =============================================================================
// UBX FRAME PARSER
//
// feedUbx() implements a byte-level state machine.
// Returns true on ANY complete, checksum-valid frame.
// Caller dispatches on (ubxClass, ubxId).
//
// TinyGPSPlus ignores non-'$' bytes → dual-feeding gpsSerial is safe.
//
// Buffer: 64 B handles all enabled messages:
//   NAV-TIMEGPS 16 B | NAV-CLOCK 20 B | NAV-STATUS 16 B | MON-RF 28 B (1 block)
// Bytes beyond sizeof(ubxBuf) are discarded but still checksummed correctly.
// =============================================================================

enum UbxState : uint8_t {
  UBX_IDLE, UBX_SYNC2, UBX_CLASS, UBX_ID,
  UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CKA, UBX_CKB
};
static UbxState ubxState = UBX_IDLE;
static uint8_t  ubxClass = 0, ubxId = 0;
static uint16_t ubxLen   = 0, ubxIdx = 0;
static uint8_t  ubxCalcA = 0, ubxCalcB = 0, ubxRxCkA = 0;
static uint8_t  ubxBuf[64];

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
      return (ubxRxCkA == ubxCalcA && b == ubxCalcB);
  }
  return false;
}

// ---------------------------------------------------------------------------
// Parse helpers — called under gpsMutex, directly after feedUbx() == true
// ---------------------------------------------------------------------------

// NAV-TIMEGPS (0x01/0x20, 16 B)
// Byte 10: leapS (I1) — GPS-UTC leap seconds
// Byte 11: valid (X1) — bit 2 = leapSValid (decoded from subframe nav-msg)
static void parseUbxNavTimeGps(GpsSnapshot& s) {
  if (ubxLen < 12) return;
  s.leapS      = (int8_t)ubxBuf[10];
  s.leapSValid = (ubxBuf[11] & 0x04) != 0;
}

// NAV-CLOCK (0x01/0x22, 20 B)
// Bytes  4- 7: clkB (I4) [ns]   — receiver clock bias
// Bytes  8-11: clkD (I4) [ns/s] — receiver clock drift
// Bytes 12-15: tAcc (U4) [ns]   — time accuracy estimate (1σ)
// Bytes 16-19: fAcc (U4) [ps/s] — frequency accuracy estimate
static void parseUbxNavClock(GpsSnapshot& s) {
  if (ubxLen < 20) return;
  auto le32s = [](const uint8_t* p) -> int32_t {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
  };
  auto le32u = [](const uint8_t* p) -> uint32_t {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  };
  s.clkB     = le32s(&ubxBuf[4]);
  s.clkD     = le32s(&ubxBuf[8]);
  s.tAcc     = le32u(&ubxBuf[12]);
  s.fAcc     = le32u(&ubxBuf[16]);
  s.clkValid = true;
}

// NAV-STATUS (0x01/0x03, 16 B)
// Byte 4: gpsFix (U1) — 0=noFix 2=2D 3=3D 5=timeOnly
// Byte 5: flags  (X1) — bit 0 = gpsFixOk
static void parseUbxNavStatus(GpsSnapshot& s) {
  if (ubxLen < 6) return;
  s.gpsFix   = ubxBuf[4];
  s.gpsFixOk = (ubxBuf[5] & 0x01) != 0;
}

// MON-RF (0x0A/0x38, ≥28 B for 1 block)
// Header (4 B): [version, nBlocks, reserved×2]
// Block 0 at payload offset 4 (24 B/block):
//   +2: antStatus    (U1) — 0=init 1=dontKnow 2=ok 3=short 4=open
//   +20: jammingState (U1) — 0=unk 1=ok 2=warn 3=crit
static void parseUbxMonRf(GpsSnapshot& s) {
  if (ubxLen < 8)    return;    // header(4) + blockId + flags + antStatus
  if (ubxBuf[1] < 1) return;
  s.jammingState = ubxBuf[5] & 0x03;  // block flags bits[1:0]: 0=unk 1=ok 2=warn 3=crit
  s.antStatus    = ubxBuf[6];          // block antStatus (unverändert)
}

// NAV-TIMELS (0x01/0x26, 24 B)
// Leap second event information including future events.
// Bytes  0- 3: iTOW (U4) [ms] — GPS time of week
// Byte   4:    version (U1) — message version (0x00)
// Bytes  5- 7: reserved0 (U1[3])
// Byte   8:    srcOfCurrLs (U1) — info source for current leap seconds
// Byte   9:    currLs (I1) [s] — current GPS-UTC leap seconds
// Byte  10:    srcOfLsChange (U1) — info source for future leap second event
// Byte  11:    lsChange (I1) [s] — future LS change (+1, -1, or 0)
// Bytes 12-15: timeToLsEvent (I4) [s] — seconds to next event (>0=future, 0=now, <0=past)
// Bytes 16-17: dateOfLsGpsWn (U2) — GPS week number of event
// Bytes 18-19: dateOfLsGpsDn (U2) — GPS day-of-week number of event
// Bytes 20-22: reserved1 (U1[3])
// Byte  23:    valid (X1) — validity flags: bit0=validCurrLs, bit1=validTimeToLsEvent
static void parseUbxNavTimels(GpsSnapshot& s) {
  if (ubxLen < 24) return;
  auto le32s = [](const uint8_t* p) -> int32_t {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
  };
  auto le16u = [](const uint8_t* p) -> uint16_t {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  };
  
  uint8_t validFlags = ubxBuf[23];
  s.currLsValid       = (validFlags & 0x01) != 0;
  s.lsChangeValid     = (validFlags & 0x02) != 0;
  
  s.currLs            = (int8_t)ubxBuf[9];
  s.lsChange          = (int8_t)ubxBuf[11];
  s.timeToLsEvent     = le32s(&ubxBuf[12]);
  s.dateOfLsGpsWn     = le16u(&ubxBuf[16]);
  s.dateOfLsGpsDn     = le16u(&ubxBuf[18]);
}

// =============================================================================
// LEAP STATE — RFC 5905 §7.3
//
// LI=0 : synchronized, no leap event known
// LI=3 : unsynchronized (startup, no leapSValid, or critical jamming)
// LI=1/2: M9N capable but data pending — NAV-TIMELS supports future LS events
//         (srcOfLsChange, lsChange, timeToLsEvent fields present). However,
//         `srcOfLsChange=0` indicates satellite hasn't yet transmitted scheduled
//         event or no future event is known. Data becomes available as satellite
//         subframes are decoded (can take minutes after first GPS fix).
// =============================================================================

static void updateLeapState() {
  // Step 1: snapshot GPS fields under gpsMutex (release before taking timingMutex)
  int8_t  leapS        = 0;
  bool    leapSValid   = false;
  bool    gpsValid     = false;
  uint8_t jammingState = 0;
  int8_t  lsChange     = 0;
  bool    lsChangeValid = false;
  int32_t timeToLsEvent = 0;

  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    gpsValid      = gpsSnap.valid;
    leapS         = gpsSnap.leapS;
    leapSValid    = gpsSnap.leapSValid;
    jammingState  = gpsSnap.jammingState;
    lsChange      = gpsSnap.lsChange;
    lsChangeValid = gpsSnap.lsChangeValid;
    timeToLsEvent = gpsSnap.timeToLsEvent;
    xSemaphoreGive(gpsMutex);
  } else {
    return;
  }

  // Step 2: update leapState under timingMutex
  if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;

  if (jammingState == 3) {
    // Critical jamming: unsynchronized
    leapState.leapIndicator = 3;
    leapState.valid         = false;
  } else if (timingState.quality == 0) {
    // No GPS lock: unsynchronized
    leapState.leapIndicator = 3;
    leapState.valid         = false;
  } else if (gpsValid && leapSValid) {
    // GPS synchronized with known current leap seconds
    // Check if future leap second event is known (LI=1 or LI=2)
    if (lsChangeValid && lsChange != 0 && timeToLsEvent > 0) {
      // Future LS event pending: positive (LI=1) or negative (LI=2)
      leapState.leapIndicator = (lsChange > 0) ? 1 : 2;
      leapState.valid         = true;
      leapState.leapSeconds   = leapS + lsChange;  // Future value after event
    } else {
      // Current leap seconds known, no future event pending
      leapState.leapIndicator = 0;
      leapState.valid         = true;
      leapState.leapSeconds   = leapS;
    }
  } else {
    // GPS fix present but subframe not yet decoded (no leapSValid)
    leapState.leapIndicator = 3;
    leapState.valid         = false;
  }

  xSemaphoreGive(timingMutex);
}

// =============================================================================
// GPS TASK — Core 1, highest priority
// =============================================================================

void gpsTask(void* param) {
  if (DEBUG_MODE) Serial.println("[gpsTask] Started");

  // M9N needs ~3-5 s after UART power-on before it accepts CFG-MSG reliably.
  // Sending earlier causes silent discard (no ACK implemented here).
  vTaskDelay(pdMS_TO_TICKS(5000));

  // CFG-MSG payload layout (8 B): msgClass, msgID, rate[I2C UART1 UART2 USB SPI rsv]
  // All enabled at 1 Hz on UART1 only.
  // Checksums computed over [class(0x06) id(0x01) len_lo len_hi payload…]:

  // NAV-TIMEGPS 0x01/0x20 — CK=0x31/0x90
  static const uint8_t cfgNavTimeGps[] = {
    0xB5,0x62, 0x06,0x01, 0x08,0x00,
    0x01,0x20, 0x00,0x01, 0x00,0x00,0x00,0x00, 0x31,0x90
  };
  // NAV-CLOCK 0x01/0x22 — CK=0x33/0x9E
  static const uint8_t cfgNavClock[] = {
    0xB5,0x62, 0x06,0x01, 0x08,0x00,
    0x01,0x22, 0x00,0x01, 0x00,0x00,0x00,0x00, 0x33,0x9E
  };
  // NAV-STATUS 0x01/0x03 — CK=0x14/0xC5
  static const uint8_t cfgNavStatus[] = {
    0xB5,0x62, 0x06,0x01, 0x08,0x00,
    0x01,0x03, 0x00,0x01, 0x00,0x00,0x00,0x00, 0x14,0xC5
  };
  // NAV-TIMELS 0x01/0x26 — CK=0x37/0xBA
  static const uint8_t cfgNavTimels[] = {
    0xB5,0x62, 0x06,0x01, 0x08,0x00,
    0x01,0x26, 0x00,0x01, 0x00,0x00,0x00,0x00, 0x37,0xBA
  };
  // MON-RF 0x0A/0x38 — CK=0x52/0x80
  static const uint8_t cfgMonRf[] = {
    0xB5,0x62, 0x06,0x01, 0x08,0x00,
    0x0A,0x38, 0x00,0x01, 0x00,0x00,0x00,0x00, 0x52,0x80
  };

  gpsSerial.write(cfgNavTimeGps, sizeof(cfgNavTimeGps)); vTaskDelay(pdMS_TO_TICKS(100));
  gpsSerial.write(cfgNavClock,   sizeof(cfgNavClock));   vTaskDelay(pdMS_TO_TICKS(100));
  gpsSerial.write(cfgNavStatus,  sizeof(cfgNavStatus));  vTaskDelay(pdMS_TO_TICKS(100));
  gpsSerial.write(cfgNavTimels,  sizeof(cfgNavTimels));  vTaskDelay(pdMS_TO_TICKS(100));
  gpsSerial.write(cfgMonRf,      sizeof(cfgMonRf));      vTaskDelay(pdMS_TO_TICKS(100));

  if (DEBUG_MODE) Serial.println("[GPS] UBX enabled: NAV-TIMEGPS NAV-CLOCK NAV-STATUS NAV-TIMELS MON-RF @ 1Hz");

  for (;;) {
    // Dual-feed: every byte → TinyGPSPlus (NMEA) + UBX state machine
    while (gpsSerial.available()) {
      uint8_t b = (uint8_t)gpsSerial.read();
      gps.encode((char)b);

      if (feedUbx(b)) {
        // Complete, checksum-valid frame — dispatch to parser under gpsMutex
        if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          if      (ubxClass == 0x01 && ubxId == 0x20) { parseUbxNavTimeGps(gpsSnap); ubxMsgTimestamps[0] = millis(); ubxMsgReceived = true; }
          else if (ubxClass == 0x01 && ubxId == 0x22) { parseUbxNavClock(gpsSnap); ubxMsgTimestamps[1] = millis(); ubxMsgReceived = true; }
          else if (ubxClass == 0x01 && ubxId == 0x03) { parseUbxNavStatus(gpsSnap); ubxMsgTimestamps[2] = millis(); ubxMsgReceived = true; }
          else if (ubxClass == 0x01 && ubxId == 0x26) { parseUbxNavTimels(gpsSnap); ubxMsgTimestamps[3] = millis(); ubxMsgReceived = true; }
          else if (ubxClass == 0x0A && ubxId == 0x38) { parseUbxMonRf(gpsSnap); ubxMsgReceived = true; }
          xSemaphoreGive(gpsMutex);
        }

        // Debug prints outside mutex — all fields are ≤32-bit (atomic on Xtensa LX6)
        // and written only by gpsTask, so no torn-read risk for informational output
        if (DEBUG_MODE) {
          if      (ubxClass == 0x01 && ubxId == 0x20)
            Serial.printf("[GPS] TIMEGPS leapS=%d leapSValid=%d\n",
                          gpsSnap.leapS, (int)gpsSnap.leapSValid);
          else if (ubxClass == 0x01 && ubxId == 0x22)
            Serial.printf("[GPS] CLOCK   tAcc=%uns clkB=%dns clkD=%dns/s\n",
                          gpsSnap.tAcc, gpsSnap.clkB, gpsSnap.clkD);
          else if (ubxClass == 0x01 && ubxId == 0x03)
            Serial.printf("[GPS] STATUS  gpsFix=%u gpsFixOk=%d\n",
                          gpsSnap.gpsFix, (int)gpsSnap.gpsFixOk);
          else if (ubxClass == 0x01 && ubxId == 0x26)
            Serial.printf("[GPS] TIMELS  currLs=%d lsChange=%d timeToLsEvent=%lds WN=%u DN=%u validFlags=0x%02x\n",
                          gpsSnap.currLs, gpsSnap.lsChange, (long)gpsSnap.timeToLsEvent,
                          gpsSnap.dateOfLsGpsWn, gpsSnap.dateOfLsGpsDn, ubxBuf[23]);
          else if (ubxClass == 0x0A && ubxId == 0x38)
            Serial.printf("[GPS] MON-RF  antStatus=%u jammingState=%u\n",
                          gpsSnap.antStatus, gpsSnap.jammingState);
        }
      }
    }

    // NMEA snapshot under gpsMutex
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (isGpsTimeValid()) {
        gpsSnap.valid       = true;
        gpsSnap.unixSec     = gpsToEpoch();
        gpsSnap.hour        = gps.time.hour();
        gpsSnap.minute      = gps.time.minute();
        gpsSnap.second      = gps.time.second();
        gpsSnap.satellites  = gps.satellites.isValid() ? gps.satellites.value() : 0;
        gpsSnap.hdop        = gps.hdop.isValid()       ? gps.hdop.hdop()        : 0.0f;
        gpsSnap.updateCount++;
      } else {
        gpsSnap.valid = false;
      }
      xSemaphoreGive(gpsMutex);
    }

    updateLeapState();

    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      syncWithGps();
      xSemaphoreGive(timingMutex);
    }

    // CFG-MSG watchdog: detect silent failures (no UBX messages for >30 seconds after startup)
    // If CFG-MSG silent fails, no messages arrive despite 5s startup delay.
    // Recovery: restart ESP32 to re-initialize M9N UART.
    static uint32_t taskStartMillis = millis();
    static bool watchdogEnabled = false;
    static bool watchdogTriggered = false;
    
    if (!watchdogEnabled && !watchdogTriggered && (millis() - taskStartMillis) > 5000) {
      watchdogEnabled = true;
      if (DEBUG_MODE) Serial.println("[GPS] Watchdog armed: checking for UBX silence...");
    }
    
    if (watchdogEnabled && !watchdogTriggered && (millis() - taskStartMillis) > 35000) {
      watchdogTriggered = true;  // Prevent re-checking
      if (!ubxMsgReceived) {
        Serial.println("[GPS] FATAL: No UBX messages in 30s — CFG-MSG likely silent-failed. Restarting ESP32...");
        Serial.flush();
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
      } else {
        if (DEBUG_MODE) Serial.println("[GPS] Watchdog OK: UBX messages detected, monitoring disabled.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =============================================================================
// NTP TASK — Core 0
// =============================================================================

void ntpTask(void* param) {
  if (DEBUG_MODE) Serial.println("[ntpTask] Started");

  ntpClient.begin();
  ntpClient.setTimeOffset(0);
  ntpClient.forceUpdate();

  ntpServerUDP.begin(NTP_PORT);
  if (DEBUG_MODE) Serial.println("[NTP] Listening on UDP 123");

  // Seed from fallback NTP so clock is usable before GPS fix
  if (ntpClient.isTimeSet()) {
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      timingState.unixSec     = ntpClient.getEpochTime();
      timingState.microsAtPps = esp_timer_get_time();
      timingState.quality     = 1;
      xSemaphoreGive(timingMutex);
      if (DEBUG_MODE) Serial.printf("[NTP] Seeded: %llu\n", timingState.unixSec);
    }
  }

  unsigned long lastNtpSync = millis();

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
      if (DEBUG_MODE) Serial.printf("[NTP] Discarded malformed packet (%d B)\n", packetSize);
    }

    // Fallback NTP sync — only when GPS unavailable, 60 s interval
    uint8_t q = 0;
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      q = timingState.quality;
      xSemaphoreGive(timingMutex);
    }
    if (q < 2 && millis() - lastNtpSync > 60000) {
      lastNtpSync = millis();
      ntpClient.update();
      if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        if (timingState.quality < 2 && ntpClient.isTimeSet()) {
          timingState.unixSec     = ntpClient.getEpochTime();
          timingState.microsAtPps = esp_timer_get_time();
          timingState.quality     = 1;
          if (DEBUG_MODE) Serial.printf("[NTP] Fallback sync: %llu\n", timingState.unixSec);
        }
        xSemaphoreGive(timingMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// DISPLAY TASK — Core 0, low priority
// =============================================================================

static void oledLine(int line, const char* text) {
  display.setCursor(0, line * OLED_LINE_H);
  display.println(text);
}

void displayTask(void* param) {
  if (DEBUG_MODE) Serial.println("[displayTask] Started");

  for (;;) {
    PreciseTime pt = getPreciseTimeSafe();

    GpsSnapshot snap = {};
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      snap = gpsSnap;
      xSemaphoreGive(gpsMutex);
    }

    uint8_t quality = 0;
    uint8_t li      = 3;
    if (xSemaphoreTake(timingMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      quality = timingState.quality;
      li      = leapState.leapIndicator;
      xSemaphoreGive(timingMutex);
    }

    // Stratum (mirrors sendNtpResponse logic)
    uint8_t stratum = (snap.jammingState == 3) ? 16
                    : (quality >= 2)            ?  1
                    : (quality == 1)            ?  2 : 16;

    // Fix type string
    const char* fixStr = "noFix";
    if (snap.gpsFixOk) {
      switch (snap.gpsFix) {
        case 2: fixStr = "2D";   break;
        case 3: fixStr = "3D";   break;
        case 5: fixStr = "Time"; break;
        default: fixStr = "DR";  break;
      }
    }

    // Source string
    const char* srcStr = "NONE";
    if      (quality == 3) srcStr = "GPS+PPS";
    else if (quality == 2) srcStr = "GPS";
    else if (quality == 1) srcStr = "NTP";

    // tAcc — display in ns if < 10000, else µs
    char tAccBuf[14] = "tA:--";
    if (snap.clkValid && snap.tAcc > 0) {
      if (snap.tAcc < 10000)
        snprintf(tAccBuf, sizeof(tAccBuf), "tA:%uns",  snap.tAcc);
      else
        snprintf(tAccBuf, sizeof(tAccBuf), "tA:%uus",  snap.tAcc / 1000);
    }

    // Alert string (jamming takes priority over antenna)
    char alertBuf[14] = "";
    if (snap.jammingState == 3)
      snprintf(alertBuf, sizeof(alertBuf), " JAM:CRIT");
    else if (snap.jammingState == 2)
      snprintf(alertBuf, sizeof(alertBuf), " JAM:WARN");
    else if (snap.antStatus == 3)
      snprintf(alertBuf, sizeof(alertBuf), " ANT:SHORT");
    else if (snap.antStatus == 4)
      snprintf(alertBuf, sizeof(alertBuf), " ANT:OPEN");

    // Time string
    char timeBuf[24];
    time_t t = (time_t)pt.seconds;
    struct tm* tm_info = gmtime(&t);
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d.%06u%s",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             pt.microseconds, ppsValid ? "" : "~");

    // OLED layout — 6 lines × 10 px = 60 px (fits in 64 px)
    char l0[22], l1[22], l2[22], l3[22], l4[22];
    snprintf(l0, sizeof(l0), "GPS NTP  LI:%u S:%u",  li, stratum);
    snprintf(l1, sizeof(l1), "IP:%s",                 Ethernet.localIP().toString().c_str());
    snprintf(l2, sizeof(l2), "%s SAT:%d HDOP:%.1f",  fixStr, snap.satellites, snap.hdop);
    if (snap.clkValid)
      snprintf(l3, sizeof(l3), "%s B:%dns",           tAccBuf, snap.clkB);
    else
      snprintf(l3, sizeof(l3), "%s",                  tAccBuf);
    snprintf(l4, sizeof(l4), "%s%s",                  srcStr, alertBuf);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    oledLine(0, l0);
    oledLine(1, l1);
    oledLine(2, l2);
    oledLine(3, l3);
    oledLine(4, l4);
    oledLine(5, timeBuf);
    display.display();

    if (DEBUG_MODE)
      Serial.printf("[DISP] %s SAT:%d | %s | tAcc:%uns clkB:%dns | jam:%u ant:%u | LI:%u S:%u\n",
                    fixStr, snap.satellites, timeBuf,
                    snap.tAcc, snap.clkB,
                    snap.jammingState, snap.antStatus, li, stratum);

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
  display.setRotation(2);
  display.setTextSize(1);
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
  while (gpsSerial.available()) gpsSerial.read();  // flush startup noise

  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), handlePpsInterrupt, RISING);
  if (DEBUG_MODE) Serial.printf("[GPS] M9N @ %d baud, PPS GPIO %d\n", GPS_BAUD, PIN_PPS);
  // CFG-MSG commands are sent in gpsTask() after the 5 s startup delay
}

// =============================================================================
// ENTRY POINTS
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  if (DEBUG_MODE) Serial.println("[BOOT] ESP32 ETH GPS NTP Server (M9N)");

  timingMutex = xSemaphoreCreateMutex();
  gpsMutex    = xSemaphoreCreateMutex();
  if (!timingMutex || !gpsMutex) {
    Serial.println("[BOOT] FATAL: mutex creation failed");
    while (true) {}
  }

  WiFi.mode(WIFI_OFF);
  btStop();

  setupOled();
  setupGps();
  setupEthernet();

  xTaskCreatePinnedToCore(gpsTask,     "gpsTask",     4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(ntpTask,     "ntpTask",     4096, NULL, configMAX_PRIORITIES - 2, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "displayTask", 8192, NULL, 1,                        NULL, 0);

  if (DEBUG_MODE) Serial.println("[BOOT] Tasks started — waiting for M9N init (5 s)");
}

void loop() {
  vTaskDelete(NULL);
}
