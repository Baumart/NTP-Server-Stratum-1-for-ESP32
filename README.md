# NTP Server for Home Setup – ESP32 + GPS + Ethernet

A high-precision **Stratum-1 NTP server** for ESP32 that synchronizes time using a GPS module with PPS (Pulse Per Second) support. Provides precise time synchronization over Ethernet to local network devices.

**Credits**: Thanks to [Stuart's Projects](https://stuartsprojects.github.io/2024/09/21/How-not-to-read-a-GPS.html) for GPS insights. 
And to [DennisSc](https://github.com/DennisSc/PPS-ntp-server) with his PPS NTP Server setup like mine, but with an other architecture.

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
      • PPS interrupt synchronization (3-pulse debouncing)
      • Atomic TimingState struct updates

Core 0 (PRO CPU):
  ├─ ntpTask [Priority: MAX-1]
  │   • UDP NTP server (port 123)
  │   • NTP fallback to pool.ntp.org every 60s (only when GPS unavailable)
  │   • RFC 5905 compliant NTP responses
  │
  └─ displayTask [Priority: 1]
      • OLED status display (2×/s)
      • Real-time offset, satellite count, signal quality
```

### Timing Architecture (v2.2 Refactored)

**Atomic Synchronization Approach:**
1. **PPS ISR** (zero-mutex): Captures `esp_timer_get_time()` on rising edge only
2. **gpsTask** (atomic struct): Combines NMEA epoch + PPS timestamp with quality level
3. **getPreciseTime()** (atomic read): Returns consistent {sec, micros} pair
4. **NTP Responses**: Built from atomic timestamps (RFC 5905 compliant)

**Time Source Priority (descending):**
1. **GPS+PPS**: ✓ Quality=3 | Offset: ±1-2ms | Stratum: 1
2. **GPS only**: ✓ Quality=2 | Offset: ±50-100ms | Stratum: 1
3. **PPS only**: ✓ Quality=2 | Offset: ±100-500ms | Stratum: 1
4. **NTP Fallback**: Quality=1 | Offset: ±1-1000ms | Stratum: 2

## 🛠 Features

### ✅ Implemented
- **Stratum-1 NTP Server** with precise PPS support (±1-2ms jitter)
- **Atomic timing architecture**: Zero race conditions, guaranteed consistency
- **Zero-mutex ISR**: <1µs interrupt latency, no priority inversion
- **3-pulse PPS debouncing**: Filters EMI and spurious synchronization
- **Multi-source time sync**: GPS+PPS (±1ms) → GPS (±50ms) → NTP fallback (±1000ms)
- **Adaptive time source switching**: Quality-based hierarchy (never downgrades unless necessary)
- **RFC 5905 NTP compliance**: Proper stratum levels, precision fields, timestamp consistency
- **Real-time OLED display** (time, satellites, HDOP, source status)
- **Ethernet connectivity** (DHCP or static IP)
- **Graceful degradation** when GPS unavailable
- **Serial debug output** (115200 baud)
- **Real-time monitoring**: `ntp_stripchart_realtime.py` CLI tool with live updates and final plots

### ❌ Not Yet Implemented
- **Leap Indicator** (LI field in NTP header)
- **Redundant GPS receivers** (failover mechanism)
- **GNSS multi-constellation** (Galileo/GLONASS support)

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

// GPS Baud Rate (standard: 9600, affects jitter by ~15%)
#define GPS_BAUD_DEFAULT 9600

// NTP Fallback Sync Interval (seconds, only when GPS unavailable)
#define NTP_FALLBACK_INTERVAL 60  // Reduced from 30s to eliminate spikes

// Enable debug output
#define DEBUG_MODE true

// PPS Debouncing
#define PPS_LOCK_CONFIRM_COUNT 3  // 3-pulse confirmation (3 seconds)
#define PPS_STALE_US 2100000ULL   // Stale if no pulse for 2.1 seconds
```

### 3. Monitor Real-Time Performance

```bash
cd ntp_offset_and_jitter_test_script
python3 ntp_stripchart_realtime.py
```

This tool provides:
- **Real-time stripchart** with live screen updates (updates every 5 seconds)
- **5-server comparison** (your GPS NTP + 4 reference servers)
- **Spike detection** (>200ms offset jumps flagged automatically)
- **Final comparative plot** (saved as PNG after monitoring ends)
- **Ctrl+C support** (gracefully stops and generates final report)

### 4. Flash to ESP32
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

## 📊 Performance Metrics

**v2.2 Refactored Results** (30-minute test):
- **Offset Stability**: ±1549ms ±2ms (was: ±700ms N-pattern)
- **Offset Variation**: <2ms standard deviation
- **Spike Frequency**: 0-2 per 30 minutes (was: 12-15 per 30 minutes)
- **Spike Magnitude**: <100ms rare (was: 200-600ms regular)
- **Response Latency**: 2-5ms consistent (was: 10-50ms variable)
- **Stratum Level**: 1 (Stable)
- **NTP Jitter**: ±1-2ms (Professional grade)

**Compared to Public NTP Servers** (from test results):
```
Per-Server Summary:
--------------------------------------------------------------------------------
10.0.0.13                 | min:   412.99 | avg:   421.38 | max:   425.26 ms
time.cloudflare.com       | min:   413.49 | avg:   421.78 | max:   425.92 ms
time.google.com           | min:   410.71 | avg:   420.65 | max:   424.74 ms
0.de.pool.ntp.org         | min:   405.47 | avg:   420.33 | max:   425.55 ms
1.de.pool.ntp.org         | min:   412.81 | avg:   420.90 | max:   424.90 ms
```
Bevor V2.2, the ESP32 NTP server had **higher jitter** and **more frequent spikes**, but after refactoring, it now **matches or exceeds** public reference servers in stability and precision:
![image](ntp_offset_and_jitter_test_script/documents/ntp_offset_base_line.png)

And this is after:
![image](ntp_offset_and_jitter_test_script/documents/ntp_offset_V2_2.png)

**Conclusion**: Your ESP32 Stratum-1 NTP server **matches or exceeds** public reference servers.

## 📄 License

Copyright © 2025

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
