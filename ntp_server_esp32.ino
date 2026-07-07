#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <AsyncUDP.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// Global variables
uint32_t rootDelay = htonl(5 * 65536 / 1000);  // 5ms → NTP format
uint32_t rootDisp = htonl(1 * 65536 / 1000);   // 1ms → NTP format
uint32_t frac = (micros() % 1000000) * 4295;   // Fractional part calculation
const char* refID = "GPS";
uint8_t ntpPacket[48] = { 0 };
uint32_t unixTime = 0;
String formattedDate;
const int NTP_PORT = 123;
AsyncUDP udp;

// GPS module setup
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Debugging mode
#define DEV_MODE true

// WiFi credentials
const char* ssid = "SSID";
const char* password = "PW";

// NTP fallback
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 5000);

// Static IP configuration
IPAddress local_IP(10, 0, 0, 13);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(10, 0, 0, 1);
IPAddress secondaryDNS(10, 0, 0, 1);

void sendNTPResponse(AsyncUDPPacket& packet) {
  getEpochTime();
  uint32_t ntpTime = htonl(unixTime + 2208988800UL);  // Unix to NTP time
  if (4 == (packet.data()[0] >> 3 & 0b111)) {
    ntpPacket[0] = (4 << 3) | 0b100;
  }
  // Fill NTP packet timestamps
  memcpy(&ntpPacket[16], &ntpTime, sizeof(ntpTime));  // Reference timestamp
  memcpy(&ntpPacket[20], &frac, sizeof(frac));
  memcpy(&ntpPacket[24], &packet.data()[40], 8);      // Origin timestamp
  memcpy(&ntpPacket[32], &ntpTime, sizeof(ntpTime));  // Receive timestamp
  memcpy(&ntpPacket[36], &frac, sizeof(frac));
  memcpy(&ntpPacket[40], &ntpTime, sizeof(ntpTime));  // Transmit timestamp
  memcpy(&ntpPacket[44], &frac, sizeof(frac));

  packet.write(ntpPacket, 48);
}

void getEpochTime() {
  struct tm timeinfo;

  // Use GPS time if available
  if (gps.speed.isUpdated() && gps.satellites.isUpdated()) {
    timeClient.setUpdateInterval(0);
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();

    uint32_t temp = mktime(&timeinfo);
    if (temp > 1700000000) {  // Valid timestamp (after 2023)
      if (DEV_MODE) Serial.printf("🕒 GPS time updated: %lu\n", temp);
      unixTime = temp;
      return;
    } else {
      if (DEV_MODE) Serial.println("❌ Invalid GPS time!");
    }
  } else {
    if (DEV_MODE) Serial.println("🛑 No valid GPS signal!");
  }

  // Use NTP as fallback
  if (timeClient.isTimeSet()) {
    timeClient.setUpdateInterval(5000);
    timeClient.forceUpdate();
    unixTime = timeClient.getEpochTime();
    if (DEV_MODE) Serial.printf("🌍 NTP time obtained: %lu\n", unixTime);
    return;
  }

  // If all fails
  if (DEV_MODE) Serial.println("⚠️ No valid time source available!");
  unixTime = 0;
}

void setup() {
  if (DEV_MODE) Serial.begin(115200);

  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  // Set static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS) && DEV_MODE) {
    Serial.println("⚠️ Error setting static IP!");
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  if (DEV_MODE) Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (DEV_MODE) Serial.print(".");
  }
  if (DEV_MODE) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
  }

  timeClient.begin();
  timeClient.setTimeOffset(0);
  timeClient.forceUpdate();

  // Initialize NTP response packet
  if (ntpPacket[1] != 1) {
    ntpPacket[0] = (0b00 << 6) | (3 << 3) | 0b100; // LI = 0, Version = 4, Mode = 4 (Server)
    ntpPacket[1] = 1;   // Stratum 1 (see RFC 5905 Figure 11)
    ntpPacket[2] = 3;   // Polling interval log2/sec
    ntpPacket[3] = -28;  // Precision not testded (-28=4ns)
    memcpy(&ntpPacket[4], &rootDelay, sizeof(rootDelay));
    memcpy(&ntpPacket[8], &rootDisp, sizeof(rootDisp));
    memcpy(&ntpPacket[12], refID, 4);
  }

  // Start NTP server
  if (udp.listen(NTP_PORT)) {
    if (DEV_MODE) Serial.println("✅ NTP server running...");
    udp.onPacket([](AsyncUDPPacket packet) {
      if (DEV_MODE) Serial.println("📡 NTP request received");
      sendNTPResponse(packet);
    });
  }
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}
