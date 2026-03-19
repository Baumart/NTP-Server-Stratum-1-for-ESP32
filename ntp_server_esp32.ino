#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <NTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// CONFIG
// =====================================================

// Debug
#define DEV_MODE true

// WiFi credentials
const char* ssid = "";
const char* password = "";

// Static IP configuration
IPAddress local_IP(10, 0, 0, 13);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(10, 0, 0, 1);
IPAddress secondaryDNS(10, 0, 0, 1);

// GPS UART
static const int RXPin = 16;
static const int TXPin = 17;
static const uint32_t GPSBaud = 9600;

// OLED I2C pins
static const int OLED_SCL = 18;
static const int OLED_SDA = 23;

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// NTP
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
const uint32_t NTP_UNIX_EPOCH_OFFSET = 2208988800UL;

// =====================================================
// GLOBALS
// =====================================================

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

WiFiUDP ntpServerUDP;
WiFiUDP ntpFallbackUDP;
NTPClient timeClient(ntpFallbackUDP, "pool.ntp.org", 0, 5000);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t packetBuffer[NTP_PACKET_SIZE];
volatile uint32_t unixTime = 0;
String currentTimeSource = "NONE";
unsigned long lastGPSValidMs = 0;
unsigned long lastDisplayUpdate = 0;

// =====================================================
// OLED HELPERS
// =====================================================

void oledPrintLine(int line, const String& text) {
  display.setCursor(0, line * 10);
  display.println(text);
}

void updateDisplay(const String& line1,
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

// =====================================================
// TIME HELPERS
// =====================================================

bool gpsDateTimeLooksValid() {
  return gps.date.isValid() &&
         gps.time.isValid() &&
         gps.date.year() >= 2024 &&
         gps.date.month() >= 1 && gps.date.month() <= 12 &&
         gps.date.day() >= 1 && gps.date.day() <= 31 &&
         gps.time.hour() <= 23 &&
         gps.time.minute() <= 59 &&
         gps.time.second() <= 60;
}

uint32_t getEpochFromGPS() {
  struct tm timeinfo = {};
  timeinfo.tm_year = gps.date.year() - 1900;
  timeinfo.tm_mon  = gps.date.month() - 1;
  timeinfo.tm_mday = gps.date.day();
  timeinfo.tm_hour = gps.time.hour();
  timeinfo.tm_min  = gps.time.minute();
  timeinfo.tm_sec  = gps.time.second();

  time_t t = mktime(&timeinfo);
  if (t > 1700000000UL) {
    return (uint32_t)t;
  }
  return 0;
}

bool updateEpochTime() {
  // GPS bevorzugt
  if (gpsDateTimeLooksValid() && gps.location.isValid()) {
    uint32_t t = getEpochFromGPS();
    if (t > 0) {
      unixTime = t;
      currentTimeSource = "GPS";
      lastGPSValidMs = millis();
      if (DEV_MODE) Serial.printf("🛰 GPS time updated: %lu\n", unixTime);
      return true;
    }
  }

  // Fallback NTP
  timeClient.update();
  if (timeClient.isTimeSet()) {
    unixTime = timeClient.getEpochTime();
    currentTimeSource = "NTP";
    if (DEV_MODE) Serial.printf("🌍 Fallback NTP time: %lu\n", unixTime);
    return true;
  }

  currentTimeSource = "NONE";
  unixTime = 0;
  if (DEV_MODE) Serial.println("⚠️ No valid time source available");
  return false;
}
// Returns current time as 64-bit NTP timestamp
uint64_t getCurrentTimeInNTP64BitFormat() {
  updateEpochTime();

  const uint64_t NTP_UNIX_OFFSET = 2208988800UL;
  uint64_t secondsSince1900 = (uint64_t)unixTime + NTP_UNIX_OFFSET;

  uint32_t us = micros() % 1000000UL;
  uint32_t fraction = (uint32_t)((double)us * 4294.967296); // 2^32 / 1_000_000

  return (secondsSince1900 << 32) | fraction;
}

void writeUint32BE(uint8_t* buf, int offset, uint32_t value) {
  buf[offset + 0] = (value >> 24) & 0xFF;
  buf[offset + 1] = (value >> 16) & 0xFF;
  buf[offset + 2] = (value >> 8) & 0xFF;
  buf[offset + 3] = value & 0xFF;
}

void writeUint64BE(uint8_t* buf, int offset, uint64_t value) {
  buf[offset + 0] = (value >> 56) & 0xFF;
  buf[offset + 1] = (value >> 48) & 0xFF;
  buf[offset + 2] = (value >> 40) & 0xFF;
  buf[offset + 3] = (value >> 32) & 0xFF;
  buf[offset + 4] = (value >> 24) & 0xFF;
  buf[offset + 5] = (value >> 16) & 0xFF;
  buf[offset + 6] = (value >> 8) & 0xFF;
  buf[offset + 7] = value & 0xFF;
}

String formatGpsTime() {
  if (!gpsDateTimeLooksValid()) return "No GPS time";

  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           gps.date.year(),
           gps.date.month(),
           gps.date.day(),
           gps.time.hour(),
           gps.time.minute(),
           gps.time.second());
  return String(buf);
}

// =====================================================
// NTP SERVER
// =====================================================

void sendNTPpacket(IPAddress remoteIP, uint16_t remotePort, uint8_t* requestBuffer) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  uint64_t receiveTime = getCurrentTimeInNTP64BitFormat();
  uint64_t transmitTime = getCurrentTimeInNTP64BitFormat();

  // LI=0, VN=4, Mode=4 (Server)
  packetBuffer[0] = 0b00100100;

  // Stratum 1 = GPS, 2 = fallback
  packetBuffer[1] = (currentTimeSource == "GPS") ? 1 : 2;

  packetBuffer[2] = 4;     // Poll
  packetBuffer[3] = 0xF7;  // Precision ~1ms-4ms

  writeUint32BE(packetBuffer, 4, 0);        // Root delay
  writeUint32BE(packetBuffer, 8, 0x50);    // Root dispersion

  // Reference ID
  if (currentTimeSource == "GPS") {
    packetBuffer[12] = 'G'; packetBuffer[13] = 'P'; packetBuffer[14] = 'S'; packetBuffer[15] = 0;
  } else {
    packetBuffer[12] = 'N'; packetBuffer[13] = 'T'; packetBuffer[14] = 'P'; packetBuffer[15] = 0;
  }

  writeUint64BE(packetBuffer, 16, transmitTime);  // Reference timestamp
  // Origin timestamp = request transmit timestamp
  memcpy(&packetBuffer[24], &requestBuffer[40], 8);
  writeUint64BE(packetBuffer, 32, receiveTime);   // Receive timestamp
  writeUint64BE(packetBuffer, 40, transmitTime);  // Transmit timestamp

  ntpServerUDP.beginPacket(remoteIP, remotePort);
  ntpServerUDP.write(packetBuffer, NTP_PACKET_SIZE);
  ntpServerUDP.endPacket();

  if (DEV_MODE) {
    Serial.printf("📡 Replied to %s:%u via %s\n",
                  remoteIP.toString().c_str(), remotePort, currentTimeSource.c_str());
  }
}

void processNTPRequests() {
  int packetSize = ntpServerUDP.parsePacket();
  if (packetSize == NTP_PACKET_SIZE) {
    IPAddress remoteIP = ntpServerUDP.remoteIP();
    uint16_t remotePort = ntpServerUDP.remotePort();

    uint8_t requestBuffer[NTP_PACKET_SIZE];
    ntpServerUDP.read(requestBuffer, NTP_PACKET_SIZE);

    sendNTPpacket(remoteIP, remotePort, requestBuffer);
  } else if (packetSize > 0) {
    ntpServerUDP.flush();
    if (DEV_MODE) Serial.printf("⚠️ Invalid NTP packet, length=%d\n", packetSize);
  }
}

// =====================================================
// SETUP
// =====================================================

void setupOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    if (DEV_MODE) {
      Serial.println("❌ OLED init failed");
    }
    return;
  }

  display.clearDisplay();
  display.display();
  updateDisplay("ESP32 GPS NTP", "Booting...", "", "", "", "");
}

void setupWiFi() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    if (DEV_MODE) Serial.println("⚠️ Failed to set static IP");
  }

  WiFi.begin(ssid, password);

  if (DEV_MODE) Serial.print("Connecting to WiFi");
  updateDisplay("ESP32 GPS NTP", "WiFi connecting...", "", "", "", "");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (DEV_MODE) Serial.print(".");
  }

  if (DEV_MODE) {
    Serial.println();
    Serial.println("✅ WiFi connected");
    Serial.print("📡 IP: ");
    Serial.println(WiFi.localIP());
  }
}

void setupGPS() {
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  if (DEV_MODE) {
    Serial.println("✅ GPS serial started");
  }
}

void setup() {
  if (DEV_MODE) {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("ESP32 GPS NTP Server starting...");
  }

  setupOLED();
  setupGPS();
  setupWiFi();

  timeClient.begin();
  timeClient.setTimeOffset(0);
  timeClient.forceUpdate();

  ntpServerUDP.begin(NTP_PORT);

  updateDisplay(
    "ESP32 GPS NTP",
    "Server running",
    "IP:",
    WiFi.localIP().toString(),
    "Port: 123",
    "Waiting for GPS..."
  );

  if (DEV_MODE) {
    Serial.println("✅ NTP server running on port 123");
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  processNTPRequests();

  if (millis() - lastDisplayUpdate > 1000 / 10) { // Update ca. 10x pro Sekunde für MS
    lastDisplayUpdate = millis();

    updateEpochTime();

    String gpsStatus = gpsDateTimeLooksValid() ? "GPS OK" : "GPS WAIT";
    String sats = "SAT: ";
    sats += gps.satellites.isValid() ? String(gps.satellites.value()) : "-";

    String src = "SRC: " + currentTimeSource;
    String ip = WiFi.localIP().toString();

    String line5;
    if (gpsDateTimeLooksValid()) {
      uint32_t ms = millis() % 1000;

      char tbuf[20];
      snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d.%03d UTC",
               gps.time.hour(), gps.time.minute(), gps.time.second(), ms);
      line5 = String(tbuf);
    } else {
      line5 = "No valid GPS time";
    }

    updateDisplay(
      "ESP32 GPS NTP",
      ip,
      gpsStatus + "  " + sats,
      src,
      line5,
      "UDP 123"
    );
  }
}
