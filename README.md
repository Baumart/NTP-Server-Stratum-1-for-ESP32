# NTP Server for Home Setup

This project is an NTP server that synchronizes time using a GPS module. It provides precise time synchronization over a local network.

## 📜 Reference
For NTP protocol details, refer to [RFC 5905](https://datatracker.ietf.org/doc/html/rfc5905).

## 📡 Device Requirements
- **Wemos Lolin ESP32**
- **GPS6MV2 Module**

## 🔧 Setup
1. Connect the GPS module to the ESP32 (RX/TX).
2. Configure WiFi credentials in the code.
3. Flash the code to the ESP32.
4. The device will act as an NTP server on your local network.

## 🛠 Features
- Stratum 1 NTP server
- GPS-based time synchronization
- Automatic fallback to NTP pool if GPS is unavailable

## ⚡ Usage
- Connect your local devices to the ESP32's NTP server.
- Monitor debug output via serial to check time sync status.

## 📌 Notes
- Ensure the GPS module has a clear sky view for accurate time sync.
- The system uses both GPS and NTP fallback for reliability.

## Not yet implemented
- Leap Indicator changes
- Reference Timestamp, dont say when the system clock was last set
- Key Identifier
- Message Digest

