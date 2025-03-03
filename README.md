# NTP Server for Home Setup

This project is an NTP server that synchronizes time using a GPS module. It provides precise time synchronization over a local network.
Thanks to [Link](https://stuartsprojects.github.io/2024/09/21/How-not-to-read-a-GPS.html).
And when you have problem with windows, look [here](https://learn.microsoft.com/en-us/troubleshoot/windows-server/active-directory/time-synchronization-not-succeed-non-ntp).

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


## Copyright <2025>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. .

