#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <NTPClient.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Set to false to disable serial debug output
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
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17

// The NEO-6M boots at 9600. We send a UBX command to switch it to 115200.
#define GPS_BAUD_DEFAULT  9600
#define GPS_BAUD_TARGET   115200

// Navigation update rate sent to the module via UBX-CFG-RATE.
// 200 ms = 5 Hz. Higher rates do not improve timing accuracy when PPS is used.
#define GPS_UPDATE_RATE_MS 200

// Minimum plausible Unix timestamp (2023-11-14) used to reject bogus GPS dates
#define GPS_MIN_VALID_UNIX 1700000000UL

// --- PPS ---------------------------------------------------------------------
// Connect the GPS TIMEPULSE (PPS) pad to this GPIO pin.
#define PIN_PPS 32

// If no PPS pulse arrives within this window the PPS data is considered stale.
#define PPS_STALE_THRESHOLD_US 1100000UL

// --- OLED (SSD1306, 128x64) --------------------------------------------------
#define PIN_OLED_SDA 23
#define PIN_OLED_SCL 18

#define OLED_WIDTH       128
#define OLED_HEIGHT       64
#define OLED_RESET        -1
#define OLED_I2C_ADDR   0x3C
#define OLED_LINE_HEIGHT  10

// --- NTP ---------------------------------------------------------------------
#define NTP_PORT        123
#define NTP_PACKET_SIZE  48

// Seconds between the NTP epoch (1900-01-01) and the Unix epoch (1970-01-01)
static const uint32_t NTP_EPOCH_OFFSET = 2208988800UL;

// NTP precision field (signed 8-bit, power of two):
//   -20 = 2^-20 s ≈ 1 µs  (with PPS)
//   -10 = 2^-10 s ≈ 1 ms  (without PPS)
#define NTP_PRECISION_WITH_PPS    0xEC
#define NTP_PRECISION_WITHOUT_PPS 0xF6

// NTP root dispersion: smaller value advertises better clock accuracy
#define NTP_DISPERSION_WITH_PPS    0x08
#define NTP_DISPERSION_WITHOUT_PPS 0x50

// Display refresh interval in milliseconds (~10 Hz)
#define DISPLAY_REFRESH_MS 100

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================

HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;

EthernetUDP ntpServerUDP;
EthernetUDP ntpFallbackUDP;
NTPClient   ntpFallbackClient(ntpFallbackUDP, "pool.ntp.org", 0, 5000);

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// --- Shared runtime state ----------------------------------------------------
static uint8_t          ntpPacketBuffer[NTP_PACKET_SIZE];
static volatile uint32_t currentUnixTime   = 0;
static String            currentTimeSource = "NONE";
static unsigned long     lastDisplayUpdate = 0;

// --- PPS state (written in ISR, read in main loop) ---------------------------
static volatile unsigned long ppsCaptureMicros = 0;
static volatile uint32_t      ppsEpochAtPulse  = 0;
static volatile bool          ppsIsValid       = false;

// =============================================================================
// PPS INTERRUPT
// =============================================================================

// Called on every rising edge of the PPS signal.
// Captures the current time and GPS epoch so the main loop can interpolate
// accurate sub-second fractions without polling the GPS UART.
void IRAM_ATTR handlePpsInterrupt() {
  ppsCaptureMicros = micros();
  ppsEpochAtPulse  = currentUnixTime;
  ppsIsValid       = true;
}

// =============================================================================
// OLED
// =============================================================================

static void oledPrintLine(int line, const String& text) {
  display.setCursor(0, line * OLED_LINE_HEIGHT);
  display.println(text);
}

static void updateDisplay(const String& line1,
                          const String& line2,
                          const String& line3,
                          const String& line4,
                          const String& line5 = "",
                          const String& line6 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  oledPrintLine(0, line1);
  oledPrintLine(1, line2);
  oledPrintLine(2, line3);
  oledPrintLine(3, line4);
  oledPrintLine(4, line5);
  oledPrintLine(5, line6);
  display.display();
}

// =============================================================================
// GPS HELPERS
// =============================================================================

// Returns true only when the GPS provides a fully sanity-checked date/time.
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

// Converts the current GPS date/time to a Unix timestamp.
// Returns 0 if the result fails the plausibility check.
static uint32_t gpsToEpoch() {
  struct tm t = {};
  t.tm_year = gps.date.year()  - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();

  time_t result = mktime(&t);
  return (result > (time_t)GPS_MIN_VALID_UNIX) ? (uint32_t)result : 0;
}

// =============================================================================
// TIME SOURCE
// =============================================================================

// Updates currentUnixTime from the best available source (GPS preferred, NTP
// as fallback). Returns true when a valid time is available.
static bool refreshTime() {
  if (isGpsTimeValid() && gps.location.isValid()) {
    uint32_t t = gpsToEpoch();
    if (t > 0) {
      currentUnixTime   = t;
      currentTimeSource = "GPS";
      if (DEBUG_MODE) Serial.printf("[TIME] GPS epoch: %lu\n", currentUnixTime);
      return true;
    }
  }

  ntpFallbackClient.update();
  if (ntpFallbackClient.isTimeSet()) {
    currentUnixTime   = ntpFallbackClient.getEpochTime();
    currentTimeSource = "NTP";
    if (DEBUG_MODE) Serial.printf("[TIME] NTP fallback epoch: %lu\n", currentUnixTime);
    return true;
  }

  currentUnixTime   = 0;
  currentTimeSource = "NONE";
  if (DEBUG_MODE) Serial.println("[TIME] No valid time source");
  return false;
}

// Returns the current time as a 64-bit NTP timestamp (seconds since 1900 in
// the upper 32 bits, binary fraction of a second in the lower 32 bits).
//
// With PPS: interpolates microseconds since the last pulse  → ~1 µs accuracy
// Without:  falls back to micros() % 1 000 000             → rough estimate
static uint64_t buildNtpTimestamp() {
  uint32_t epochSeconds;
  uint32_t microsIntoSecond;

  if (ppsIsValid) {
    unsigned long elapsed = micros() - ppsCaptureMicros;
    if (elapsed < PPS_STALE_THRESHOLD_US) {
      epochSeconds     = ppsEpochAtPulse + (uint32_t)(elapsed / 1000000UL);
      microsIntoSecond = elapsed % 1000000UL;
    } else {
      // PPS pulse is overdue — degrade gracefully to the coarser fallback
      epochSeconds     = currentUnixTime;
      microsIntoSecond = micros() % 1000000UL;
    }
  } else {
    epochSeconds     = currentUnixTime;
    microsIntoSecond = micros() % 1000000UL;
  }

  uint64_t seconds1900 = (uint64_t)epochSeconds + NTP_EPOCH_OFFSET;
  uint32_t fraction    = (uint32_t)((double)microsIntoSecond * 4294.967296); // 2^32 / 1e6

  return (seconds1900 << 32) | fraction;
}

// =============================================================================
// BINARY WRITE HELPERS
// =============================================================================

static void writeU32BE(uint8_t* buf, int offset, uint32_t value) {
  buf[offset + 0] = (value >> 24) & 0xFF;
  buf[offset + 1] = (value >> 16) & 0xFF;
  buf[offset + 2] = (value >>  8) & 0xFF;
  buf[offset + 3] =  value        & 0xFF;
}

static void writeU64BE(uint8_t* buf, int offset, uint64_t value) {
  buf[offset + 0] = (value >> 56) & 0xFF;
  buf[offset + 1] = (value >> 48) & 0xFF;
  buf[offset + 2] = (value >> 40) & 0xFF;
  buf[offset + 3] = (value >> 32) & 0xFF;
  buf[offset + 4] = (value >> 24) & 0xFF;
  buf[offset + 5] = (value >> 16) & 0xFF;
  buf[offset + 6] = (value >>  8) & 0xFF;
  buf[offset + 7] =  value        & 0xFF;
}

// =============================================================================
// NTP SERVER
// =============================================================================

// Builds and sends an NTPv4 server response.
// requestBuffer must point to the full 48-byte client request.
static void sendNtpResponse(IPAddress remoteIp,
                            uint16_t  remotePort,
                            uint8_t*  requestBuffer) {
  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);

  // Sample timestamps back-to-back to minimise the gap between them
  const uint64_t receiveTime  = buildNtpTimestamp();
  const uint64_t transmitTime = buildNtpTimestamp();

  const bool usingGps = (currentTimeSource == "GPS");
  const bool usingPps = ppsIsValid;

  // Byte 0: LI=00 (no leap warning), VN=100 (NTPv4), Mode=100 (server)
  ntpPacketBuffer[0] = 0b00100100;

  // Stratum 1 when disciplined by GPS/PPS, stratum 2 for NTP fallback
  ntpPacketBuffer[1] = usingGps ? 1 : 2;

  ntpPacketBuffer[2] = 4;  // Poll exponent (informational only)
  ntpPacketBuffer[3] = usingPps ? NTP_PRECISION_WITH_PPS : NTP_PRECISION_WITHOUT_PPS;

  writeU32BE(ntpPacketBuffer, 4, 0);
  writeU32BE(ntpPacketBuffer, 8,
             usingPps ? NTP_DISPERSION_WITH_PPS : NTP_DISPERSION_WITHOUT_PPS);

  // Reference identifier: "GPS\0" or "NTP\0"
  ntpPacketBuffer[12] = usingGps ? 'G' : 'N';
  ntpPacketBuffer[13] = usingGps ? 'P' : 'T';
  ntpPacketBuffer[14] = usingGps ? 'S' : 'P';
  ntpPacketBuffer[15] = 0;

  writeU64BE(ntpPacketBuffer, 16, transmitTime);        // Reference timestamp
  memcpy(&ntpPacketBuffer[24], &requestBuffer[40], 8);  // Origin = client transmit
  writeU64BE(ntpPacketBuffer, 32, receiveTime);          // Receive  timestamp
  writeU64BE(ntpPacketBuffer, 40, transmitTime);         // Transmit timestamp

  ntpServerUDP.beginPacket(remoteIp, remotePort);
  ntpServerUDP.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  ntpServerUDP.endPacket();

  if (DEBUG_MODE) {
    Serial.printf("[NTP] Response → %s:%u  source=%s  pps=%s\n",
                  remoteIp.toString().c_str(), remotePort,
                  currentTimeSource.c_str(), usingPps ? "yes" : "no");
  }
}

// Reads any pending UDP packets and replies to valid NTP requests.
static void processNtpRequests() {
  int packetSize = ntpServerUDP.parsePacket();

  if (packetSize == NTP_PACKET_SIZE) {
    IPAddress remoteIp   = ntpServerUDP.remoteIP();
    uint16_t  remotePort = ntpServerUDP.remotePort();

    uint8_t requestBuffer[NTP_PACKET_SIZE];
    ntpServerUDP.read(requestBuffer, NTP_PACKET_SIZE);

    sendNtpResponse(remoteIp, remotePort, requestBuffer);

  } else if (packetSize > 0) {
    ntpServerUDP.flush();  // discard malformed packets to keep the buffer clear
    if (DEBUG_MODE) {
      Serial.printf("[NTP] Discarded malformed packet (%d bytes)\n", packetSize);
    }
  }
}

// =============================================================================
// UBX (u-blox binary protocol)
// =============================================================================

// Transmits a UBX frame and appends the two Fletcher checksum bytes.
// The checksum covers every byte from index 2 onwards (class, id, length,
// payload) as specified in the u-blox 6 receiver description.
static void sendUbxFrame(const uint8_t* frame, size_t length) {
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < length; i++) {
    ckA += frame[i];
    ckB += ckA;
  }
  gpsSerial.write(frame, length);
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
  gpsSerial.flush();
}

// UBX-CFG-PRT: reconfigures UART1 on the NEO-6M to a new baud rate.
// The ESP32 UART is switched to match immediately after.
static void setGpsBaudRate(uint32_t targetBaud) {
  uint8_t frame[] = {
    0xB5, 0x62,               // UBX sync chars
    0x06, 0x00,               // Class CFG, ID PRT
    0x14, 0x00,               // Payload length: 20 bytes
    0x01,                     // Port ID: UART1
    0x00,                     // Reserved
    0x00, 0x00,               // txReady (disabled)
    0xD0, 0x08, 0x00, 0x00,  // UART mode: 8 data bits, no parity, 1 stop bit
    0x00, 0x00, 0x00, 0x00,  // Baud rate (little-endian, filled below)
    0x07, 0x00,               // inProtoMask:  UBX | NMEA | RTCM
    0x03, 0x00,               // outProtoMask: UBX | NMEA
    0x00, 0x00,               // Flags
    0x00, 0x00                // Reserved
  };

  frame[12] = (targetBaud)       & 0xFF;
  frame[13] = (targetBaud >>  8) & 0xFF;
  frame[14] = (targetBaud >> 16) & 0xFF;
  frame[15] = (targetBaud >> 24) & 0xFF;

  sendUbxFrame(frame, sizeof(frame));
  delay(100);

  gpsSerial.begin(targetBaud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(100);

  if (DEBUG_MODE) Serial.printf("[GPS] Baud rate → %lu\n", targetBaud);
}

// UBX-CFG-RATE: sets the navigation measurement interval in milliseconds.
// With PPS providing sub-second accuracy, 5 Hz (200 ms) is sufficient to keep
// satellite count and HDOP refreshed without saturating the UART.
static void setGpsUpdateRate(uint16_t intervalMs) {
  uint8_t frame[] = {
    0xB5, 0x62,     // UBX sync chars
    0x06, 0x08,     // Class CFG, ID RATE
    0x06, 0x00,     // Payload length: 6 bytes
    0x00, 0x00,     // measRate in ms (little-endian, filled below)
    0x01, 0x00,     // navRate: 1 solution per measurement cycle
    0x01, 0x00      // timeRef: align to GPS time
  };

  frame[6] = (intervalMs)      & 0xFF;
  frame[7] = (intervalMs >> 8) & 0xFF;

  sendUbxFrame(frame, sizeof(frame));
  delay(100);

  if (DEBUG_MODE) {
    Serial.printf("[GPS] Update rate → %u ms (%u Hz)\n",
                  intervalMs, 1000u / intervalMs);
  }
}

// =============================================================================
// SETUP ROUTINES
// =============================================================================

static void setupOled() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    if (DEBUG_MODE) Serial.println("[OLED] Initialisation failed");
    return;
  }

  display.clearDisplay();
  display.display();
  updateDisplay("ETH GPS NTP", "Booting...", "", "", "", "");
  if (DEBUG_MODE) Serial.println("[OLED] Ready");
}

static void setupEthernet() {
  updateDisplay("ETH GPS NTP", "ETH connecting...", "", "", "", "");

  pinMode(PIN_ETH_CS, OUTPUT);
  digitalWrite(PIN_ETH_CS, HIGH);

  SPI.begin(PIN_ETH_SCK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
  SPI.setFrequency(ETH_SPI_FREQUENCY);

  Ethernet.init(PIN_ETH_CS);
  Ethernet.begin(ETH_MAC, ETH_IP, ETH_DNS, ETH_GATEWAY, ETH_SUBNET);
  delay(1000);

  if (DEBUG_MODE) {
    Serial.print("[ETH] IP address: ");
    Serial.println(Ethernet.localIP());
  }
}

static void setupGps() {
  // Step 1 — start at the module default baud rate
  gpsSerial.begin(GPS_BAUD_DEFAULT, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  delay(500);
  if (DEBUG_MODE) Serial.printf("[GPS] Serial started at %d baud\n", GPS_BAUD_DEFAULT);

  // Step 2 — switch the module and the ESP32 UART to the faster baud rate
  setGpsBaudRate(GPS_BAUD_TARGET);

  // Step 3 — set the navigation update rate
  setGpsUpdateRate(GPS_UPDATE_RATE_MS);

  // Step 4 — attach the PPS interrupt on a rising edge
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), handlePpsInterrupt, RISING);
  if (DEBUG_MODE) Serial.printf("[GPS] PPS interrupt attached on GPIO %d\n", PIN_PPS);
}

// =============================================================================
// DISPLAY HELPER
// =============================================================================

// Builds the time string shown on the bottom OLED line.
// With PPS the milliseconds are interpolated from the last pulse (accurate).
// Without PPS, millis() % 1000 is used and a tilde suffix signals the estimate.
static String buildTimeString() {
  if (!isGpsTimeValid()) {
    return "No GPS time";
  }

  uint32_t ms;
  bool     accurate;

  if (ppsIsValid) {
    ms       = ((micros() - ppsCaptureMicros) / 1000UL) % 1000UL;
    accurate = true;
  } else {
    ms       = millis() % 1000UL;
    accurate = false;
  }

  char buf[24];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d%s",
           gps.time.hour(),
           gps.time.minute(),
           gps.time.second(),
           ms,
           accurate ? "" : "~");
  return String(buf);
}

// =============================================================================
// ARDUINO ENTRY POINTS
// =============================================================================

void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] ESP32 ETH GPS NTP Server starting");
  }

  setupOled();
  setupGps();
  setupEthernet();

  ntpFallbackClient.begin();
  ntpFallbackClient.setTimeOffset(0);
  ntpFallbackClient.forceUpdate();

  ntpServerUDP.begin(NTP_PORT);

  updateDisplay(
    "ETH GPS NTP",
    Ethernet.localIP().toString(),
    "Port: 123",
    "Waiting for GPS...",
    "",
    ""
  );

  if (DEBUG_MODE) Serial.println("[NTP] Server listening on UDP port 123");
}

void loop() {
  // Feed all available GPS bytes into the NMEA parser
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Handle incoming NTP requests as quickly as possible
  processNtpRequests();

  // Refresh the OLED at ~10 Hz
  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_MS) {
    lastDisplayUpdate = millis();

    refreshTime();

    const String ip        = Ethernet.localIP().toString();
    const String gpsStatus = isGpsTimeValid() ? "GPS OK" : "GPS WAIT";
    const String sats      = gps.satellites.isValid()
                               ? String(gps.satellites.value()) : "-";
    const String hdop      = gps.hdop.isValid()
                               ? String(gps.hdop.hdop(), 2)     : "-";
    const String timeStr   = buildTimeString();

    updateDisplay(
      "ETH GPS NTP",
      ip,
      gpsStatus + "  SAT: " + sats,
      "HDOP: " + hdop,
      "SRC: " + currentTimeSource,
      timeStr
    );

    if (DEBUG_MODE) {
      Serial.printf("[LOOP] %s | %s | SAT:%s | HDOP:%s | %s | PPS:%s\n",
                    ip.c_str(),
                    gpsStatus.c_str(),
                    sats.c_str(),
                    hdop.c_str(),
                    timeStr.c_str(),
                    ppsIsValid ? "yes" : "no");
    }
  }
}
