# NTP Server for Home Setup – ESP32 + GPS + Ethernet

A high-precision **Stratum-1 NTP server** for ESP32 that synchronizes time using a GPS module with PPS (Pulse Per Second) support. Provides precise time synchronization over Ethernet to local network devices.

**Credits**: Thanks to [Stuart's Projects](https://stuartsprojects.github.io/2024/09/21/How-not-to-read-a-GPS.html) for GPS insights.
**Windows NTP Issues**: See [Microsoft's guide](https://learn.microsoft.com/en-us/troubleshoot/windows-server/active-directory/time-synchronization-not-succeed-non-ntp).

## 📜 Reference
- **NTP Protocol**: [RFC 5905](https://datatracker.ietf.org/doc/html/rfc5905)
- **Time Source Priority**: GPS+PPS > GPS > PPS > NTP Fallback

## 📡 Hardware Requirements

| Component | Model | Connection |
|-----------|-------|-----------|
| **Microcontroller** | Wemos Lolin ESP32 | - |
| **GPS Module** | NEO-6M | UART1: RX=GPIO16, TX=GPIO17 (9600 baud) |
| **PPS Signal** | GPS PPS Output | GPIO32 (Rising Edge Interrupt) |
| **Ethernet** | W5500 (SPI) | CS=GPIO14, SCK=GPIO27, MISO=GPIO26, MOSI=GPIO25 |
| **Display** | SSD1306 OLED 128×64 | I2C: SDA=GPIO23, SCL=GPIO18 (Address: 0x3C) |

## ⚙️ Architecture

### FreeRTOS Multi-Core Design (3 Tasks)

```
Core 1 (App CPU):
  └─ gpsTask [Priority: MAX]
      • NMEA sentence parsing via TinyGPS+
      • PPS interrupt synchronization
      • Updates time anchor point under timeMutex

Core 0 (PRO CPU):
  ├─ ntpTask [Priority: MAX-1]
  │   • UDP NTP server (port 123)
  │   • NTP fallback to pool.ntp.org every 30s
  │   • Constructs & sends NTP responses
  │
  └─ displayTask [Priority: 1]
      • OLED status display (2×/s)
      • Shows time, GPS satellites, signal quality
```

### Timing Architecture

**Precision Synchronization Approach:**
1. **PPS ISR** captures `esp_timer_get_time()` on pulse rising edge
2. **gpsTask** combines latest NMEA epoch + PPS timestamp into stable anchor point
3. **getPreciseTime()** interpolates elapsed microseconds from anchor
4. **NTP Responses** built from interpolated timestamps

**Time Source Priority (descending):**
1. **GPS+PPS**: ✓ NMEA valid + PPS pulse available → Anchor = (NMEA+1 sec, PPS ISR time)
2. **PPS only**: ✓ PPS available, last GPS epoch known → Anchor = (Last+1 sec, PPS ISR time)
3. **GPS (NMEA)**: ✓ Valid GPS time, no PPS → Anchor = (NMEA sec, current time)
4. **NTP Fallback**: Last resort if GPS unavailable → pool.ntp.org query every 30s

## 🛠 Features

### ✅ Implemented
- **Stratum-1 NTP Server** with precise PPS support
- **Multi-source time sync**: GPS+PPS, GPS, or NTP fallback
- **Microsecond-level precision** with PPS (±1 µs)
- **Adaptive NTP precision metrics**:
  - GPS+PPS: `2^-20` (~1 µs)
  - GPS only: `2^-16` (~15 µs)
  - NTP only: `2^-10` (~1 ms)
- **Real-time OLED display** (time, satellites, HDOP, source status)
- **Ethernet connectivity** (DHCP or static IP)
- **Graceful degradation** when GPS unavailable
- **Serial debug output** (115200 baud)

### ❌ Not Yet Implemented
- **Leap Indicator** (LI field in NTP header)
- **Reference Timestamp** (when system clock was last updated)
- **Symmetric Key Authentication** (Key Identifier + Message Digest)

## 🔧 Setup & Configuration

### 1. Hardware Wiring
Connect components according to the table above. Ensure GPS has **clear sky view** for signal acquisition.

### 2. Configuration (In Code)
Edit the following in `ntp_server_esp32.ino`:

```cpp
// Ethernet Static IP
IPAddress ETH_IP     (10, 0, 0, 13);   // Device IP
IPAddress ETH_GATEWAY(10, 0, 0,  1);   // Gateway
IPAddress ETH_SUBNET (255, 255, 0, 0); // Subnet
IPAddress ETH_DNS    (10, 0, 0,  1);   // DNS server
byte ETH_MAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // MAC address

// GPS Baud Rate (standard: 9600)
#define GPS_BAUD_DEFAULT 9600

// Enable debug output
#define DEBUG_MODE true
```

### 3. Flash to ESP32
- Install **Arduino IDE** with **ESP32 board support**
- Install required libraries:
  - `Adafruit_SSD1306`
  - `Adafruit_GFX`
  - `TinyGPSPlus`
  - `ArduinoNTPClient` (or equivalent)
- Connect ESP32 via USB and flash sketch
- Monitor serial output (115200 baud) for initialization logs

## ⚡ Usage

### Connecting NTP Clients
1. Configure your system/device to sync time from `10.0.0.13` (or configured IP)
   ```bash
   # Linux/macOS example
   ntpq -p 10.0.0.13
   
   # Windows example
   w32tm /query /peers
   ```

2. Monitor OLED display to verify:
   - ✓ GPS signal locked (SAT count > 0)
   - ✓ PPS pulse active (no `~` tilde on time)
   - ✓ Time source (GPS+PPS, GPS, or NTP)

### Serial Debug Output
```
[DISP] 10.0.0.13 | GPS OK | SAT:12 | HDOP:0.89 | 14:35:42.123456
[NTP] → 10.0.0.100:54321  src=GPS+PPS  pps=yes
```

## 📌 Important Notes

### GPS Acquisition
- NEO-6M module requires **clear line-of-sight** to satellites
- Initial lock can take **30–120 seconds** (cold start)
- Keep antenna placement **away from metal/RF interference**
- Indoor testing may fail; test outdoor or near window

### PPS Stale Timeout
- If no PPS pulse detected for > **2.1 seconds**, PPS is marked stale
- System automatically falls back to NMEA-only time
- Check GPIO32 wiring and GPS PPS output if timeout occurs

### Network Configuration
- **Ethernet only** (WiFi disabled to reduce jitter)
- Static IP configured; update as needed for your network
- MAC address is hardcoded; modify if conflicts occur

### Time Precision
- **With PPS**: ±1–10 µs typical
- **NMEA only**: ±50–100 ms typical
- **NTP fallback**: ±1–100 ms (depends on network latency)

## 🔍 Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| GPS says "WAIT" | No NMEA data received | Check RX/TX wiring, baud rate (9600) |
| SAT count = 0 | No satellite lock | Move antenna outdoors, clear sky view |
| PPS shows `~` | PPS signal not detected | Check GPIO32 wiring, GPS PPS output |
| NTP stratum = 2 | No GPS available | GPS will sync, fallback to pool.ntp.org |
| OLED blank | I2C init failed | Verify SDA/SCL pins, address (0x3C) |
| No Ethernet | Initialization error | Check SPI pins, W5500 power supply |

## 📚 Debug Compilation Flags

Toggle `DEBUG_MODE` to enable/disable serial logging:
```cpp
#define DEBUG_MODE true  // Enable debug output (115200 baud)
```

## 📋 Performance Metrics

- **Time Accuracy**: ±1 µs (GPS+PPS) to ±100 ms (NTP fallback)
- **NTP Response Latency**: ~5–10 ms (Ethernet)
- **Task Stack Usage**: ~16 KB combined
- **Memory Usage**: ~80 KB (typical)
- **Boot Time**: ~10–15 seconds (Ethernet + GPS init)

## 📄 License

Copyright © 2025

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
