#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <AsyncUDP.h>

// GPS Setup
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Debugging 
#define DEV_MODE false

// WLAN-Zugangsdaten
const char* ssid = "SSID";
const char* password = "PW";

// Static IP-Adresse
IPAddress local_IP(10, 0, 0, 13);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(10, 0, 0, 1);
IPAddress secondaryDNS(10, 0, 0, 1);

AsyncUDP udp;
const int NTP_PORT = 123;
uint32_t epochTime = 0;

// Responds to NTP requests with the current GPS time
void sendNTPResponse(AsyncUDPPacket& packet) {
    if (epochTime == 0) return; // If no valid GPS time is available, do not send a response

    uint8_t ntpPacket[48] = {0};
    ntpPacket[0] = 0b00100100; // NTP Header
    uint32_t txTime = htonl(epochTime + 2208988800UL); // Convert Unix to NTP time
    memcpy(&ntpPacket[40], &txTime, sizeof(txTime));

    packet.write(ntpPacket, 48);
}

// Calculates the correct time zone taking into account summer/winter time
int getTimezoneOffset(int year, int month, int day, int weekday, int hour) {
    int timezoneOffset = 3600; // Standard MEZ (UTC+1)

    if (month > 3 && month < 10) {
        timezoneOffset = 7200; // MESZ (UTC+2)
    } else if (month == 3 && (day - weekday) >= 25 && hour >= 2) {
        timezoneOffset = 7200; // LLast Sunday in March, from 02:00 CEST
    } else if (month == 10 && (day - weekday) >= 25 && hour < 2) {
        timezoneOffset = 7200; // Still summertime
    }

    return timezoneOffset;
}

// Calculates the current Unix time with summer/winter time
uint32_t getEpochTime() {
    if (!gps.date.isValid() || !gps.time.isValid()) return 0;

    struct tm timeinfo;
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon  = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min  = gps.time.minute();
    timeinfo.tm_sec  = gps.time.second();

    uint32_t utcTime = mktime(&timeinfo);
    int weekday = timeinfo.tm_wday;

    return utcTime + getTimezoneOffset(gps.date.year(), gps.date.month(), gps.date.day(), weekday, gps.time.hour());
}

void setup() {
    if (DEV_MODE) Serial.begin(115200);

    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // Static IP set
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS) && DEV_MODE) {
        Serial.println("⚠️ Error when setting the static IP!");
    }

    WiFi.begin(ssid, password);
    if (DEV_MODE) Serial.print("Connect with WLAN...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (DEV_MODE) Serial.print(".");
    }
    if (DEV_MODE) {
        Serial.println("\n✅ WLAN connected!");
        Serial.print("📡 IP-Adresse: ");
        Serial.println(WiFi.localIP());
    }

    // NTP-Server start
    if (udp.listen(NTP_PORT)) {
        if (DEV_MODE) Serial.println("✅ NTP-Server runs...");
        udp.onPacket([](AsyncUDPPacket packet) {
            if (DEV_MODE) Serial.println("📡 NTP-Anfrage receive");
            sendNTPResponse(packet);
        });
    }
}

void loop() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    if (gps.date.isUpdated() && gps.time.isUpdated()) {
        epochTime = getEpochTime();
        if (DEV_MODE) Serial.printf("🕒 GPS-time updated: %lu\n", epochTime);
    }

    delay(1000); 

}