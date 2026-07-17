# Testing & Verification Guide

## Pre-Compilation Checklist

- [ ] Arduino IDE or PlatformIO configured for ESP32
- [ ] Required libraries installed:
  - Ethernet (W5500 support)
  - EthernetUdp
  - WiFi (for NTP fallback)
  - NTPClient
  - TinyGPSPlus
  - Adafruit_GFX
  - Adafruit_SSD1306
  - FreeRTOS (built-in)

- [ ] Hardware connections verified:
  - GPS RX (GPIO 16) ↔ GPS TX (NEO-6M)
  - GPS TX (GPIO 17) ↔ GPS RX (NEO-6M)
  - GPS GND ↔ ESP32 GND
  - PPS (GPIO 32) ↔ GPS PPS output (typically NEO-6M pin 3)
  - W5500 SPI (GPIO 27, 25, 26, 14)
  - OLED SDA (GPIO 23) ↔ I2C SDA
  - OLED SCL (GPIO 18) ↔ I2C SCL

## Compilation

### Arduino IDE
1. Open `ntp_server_esp32.ino`
2. Select: Tools → Board → ESP32-S3 DevKit or equivalent
3. Verify (Ctrl+R) - should complete without errors
4. If errors: check library paths and #include statements

### PlatformIO
```bash
pio run -e esp32 --target upload
```

## Initial Boot (Serial Monitor, 115200 baud)

```
[BOOT] ESP32 ETH GPS NTP Server (Refactored)
[OLED] Ready
[ETH] IP: 10.0.0.13
[gpsTask] Started
[ntpTask] Started
[GPS] PPS on GPIO 32
[NTP] Listening on UDP 123
[displayTask] Started
[BOOT] All tasks started
[NTP] Seeded: 1784916000
```

**Expected timing:** Entire startup in <2 seconds

## Phase 1: GPS Initialization (First 30 seconds)

Serial output should show:
```
[DISP] SAT:0 | HDOP:-- | GPS WAIT ...
[DISP] SAT:1 | HDOP:0.9 | 23:25:45.123456
[DISP] SAT:8 | HDOP:0.8 | 23:25:47.234567
[DISP] SAT:12 | HDOP:0.7 | 23:25:49.345678
```

**OLED Display:**
```
ETH GPS NTP Server
IP: 10.0.0.13
GPS OK
SAT:12 HDOP:0.7
SRC: GPS
23:25:49.345678
```

**Expected behavior:**
- Satellites increase to 8+ within 30 seconds
- HDOP decreases to <2.0 (higher precision)
- Display updates every 0.5 seconds without glitching
- No "GPS WAIT" message persists after 30 seconds

**Troubleshooting GPS:**
- ✗ "GPS WAIT" persists: Check GPS wiring, baud rate (9600)
- ✗ SAT always 0: Verify PPS pin not conflicting
- ✗ Serial noise: Check GPS TX pull-up resistors
- ✓ Rapid SAT increase: Normal outdoor operation

## Phase 2: PPS Synchronization (30-60 seconds)

Serial output transition:
```
[NTP] → 10.0.0.1:12345 (src=GPS, pps=no)
[NTP] → 10.0.0.1:12346 (src=GPS, pps=no)
[NTP] → 10.0.0.1:12347 (src=GPS, pps=yes)  ← PPS lock achieved!
```

**OLED Display Update:**
```
SRC: GPS+PPS         ← Changed from GPS to GPS+PPS
```

**Expected timing:**
- GPS lock: ~10-60 seconds
- PPS lock: GPS lock + ~3 seconds (3 PPS pulses)
- Total: <2 minutes to full synchronization

**Troubleshooting PPS:**
- ✗ "pps=no" persists: Check GPIO 32 wiring, voltage levels
- ✗ Rapid pps=yes/no toggling: PPS signal unstable (check cable)
- ✓ pps=yes remains after 10 responses: Normal operation

## Phase 3: NTP Accuracy Testing

### Test from Linux/Unix:
```bash
# Single query
ntpdate -q 10.0.0.13

# Monitoring (requires root)
ntpq -p 10.0.0.13

# Detailed (requires ntpd)
ntpd -gq -p 10.0.0.13
```

### Expected Output:
```
     remote           refid      st t when poll reach   delay   offset  jitter
==============================================================================
 10.0.0.13       GPS              1 u   20   64  377   1.234  -15.234   8.934
```

**Field interpretation:**
- `st=1`: Stratum 1 (Excellent, close to reference)
- `delay≈1-5ms`: Network round-trip (Ethernet typically 0.5-1ms)
- `offset≈±50ms`: Time difference (GPS+PPS ±50ms typical)
- `jitter≈5-15ms`: Clock stability (lower = better)

### Successful NTP Query Result:
```
 10.0.0.13       .GPS.            1 u   12   64  377   0.924  +24.562   5.134
```

✓ Stratum 1
✓ Low jitter (< 20ms)
✓ Small offset (± 50ms acceptable for GPS+PPS)

## Phase 4: Offset Stability Measurement

### Using ntpd test client:
```bash
# Create test file /tmp/ntp_test.sh
#!/bin/bash
for i in {1..60}; do
  ntpdate -q 10.0.0.13 2>/dev/null | grep "offset" | awk '{print $6}'
  sleep 5
done > /tmp/offsets.txt

# Plot with gnuplot
gnuplot <<EOF
set title "NTP Offset Stability"
set xlabel "Sample (5s intervals)"
set ylabel "Offset (seconds)"
plot "/tmp/offsets.txt" with lines
EOF
```

### Expected Pattern:
```
Sample 0:   +0.024 sec    ← Initial sync
Sample 1:   +0.026 sec
Sample 2:   +0.023 sec    ← Stable!
Sample 3:   +0.022 sec
...
Sample 60:  +0.023 sec    ← Consistent
```

**✓ Good outcome:** Flat line ±0.05 sec (no oscillation)

**✗ Bad outcome (original bug):** N-pattern
```
+0.024 → +0.126 → +0.045 → -0.234 → +0.024  (repeating cycle)
```

## Phase 5: Load Testing

### Generate NTP traffic:
```bash
# Using ntpq poll
while true; do
  ntpq -c "rv 0 stratum" 10.0.0.13 &
done

# Or using custom UDP client
python3 -c "
import socket, struct, time, random
while True:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  payload = b'\x1b' + b'\x00' * 47
  sock.sendto(payload, ('10.0.0.13', 123))
  sock.close()
  time.sleep(0.1)
"
```

**Monitor ESP32 serial output:**
```
[NTP] → 10.0.0.1:54321 (src=GPS+PPS, pps=yes)
[NTP] → 10.0.0.1:54322 (src=GPS+PPS, pps=yes)
[NTP] → 10.0.0.1:54323 (src=GPS+PPS, pps=yes)
...
```

**Expected behavior:**
- Responses within <50ms
- No timeouts or dropped packets
- Display remains responsive (updates every 0.5s)
- No mutex timeout messages

## Diagnostic Checks

### Enable Serial Debugging (if uncommented):
Modify `#define DEBUG_MODE true` (already enabled in refactored version)

### Critical Checksums:
```
✓ ISR allocations: 0 (no new/malloc in handlePpsInterrupt)
✓ Mutex in ISR: False (IRAM_ATTR function is interrupt-safe)
✓ struct TimingState size: 17 bytes (fits atomic read on 64-bit)
✓ Quality levels: 4 states (0=none, 1=NTP, 2=GPS, 3=GPS+PPS)
```

### Performance Metrics:
```
✓ ISR duration: < 1 microsecond
✓ Mutex hold time: 1-2 milliseconds
✓ Task priorities: gpsTask > ntpTask > displayTask
✓ Task core affinity: gpsTask→Core1, ntpTask/displayTask→Core0
```

## Common Issues & Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Compile errors "no member" | Missing struct definition | Check struct TimingState definition at line 41 |
| "GPS WAIT" forever | GPS not responding | Verify RX/TX pins, check TinyGPSPlus setup |
| PPS never locks (src=GPS only) | GPIO 32 wiring issue | Check PPS signal on oscilloscope, verify pull-up |
| Offset ±1000ms | GPS not stable | Wait for more satellites (12+), HDOP < 1.5 |
| Display glitching | Mutex contention | Check task delays, reduce DEBUG_MODE verbosity |
| NTP timeout in queries | Network issue | Verify Ethernet cable, IP routing |

## Expected Results Summary

### Baseline (Before refactoring):
- Offset oscillation: N-pattern ±700ms
- Stability: Unstable, cycles every 5 seconds
- Response time: 10-50ms variable
- Task CPU: High due to mutex contention

### Target (After refactoring):
- Offset stability: ±50ms flat
- Drift rate: < 1 ppm
- Response time: 2-5ms consistent
- Task CPU: Minimal, no contention
- Stratum: 1 (with GPS+PPS)
- Jitter: < 10ms

## Verification Complete ✓

When all checks pass:
1. Commit refactored code to repository
2. Document GPS/PPS calibration offset if needed
3. Archive this test report with timestamp
4. Update system documentation with new performance specs
