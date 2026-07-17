# 🔧 Hotfix v2.1 - +1 Second Calibration Removal

## Problem
Offset konsistent ~+1000ms höher als andere NTP-Server:
- Ihr Server: +2540ms
- Andere Server: +1540ms  
- Differenz: genau 1000ms

## Root Cause
Die PPS-Synchronisierungslogik addierte `+1` zur GPS-Epoch:

```cpp
// OLD (WRONG)
timingState.unixSec = gpsEpoch + 1;  // Added extra second
```

**Warum falsch?**
- TinyGPSPlus bereits die korrekte NMEA-Zeit liefert
- Das PPS-Signal synchronisiert zur **aktuellen** Sekunde
- Keine +1-Anpassung nötig

## Solution
Entfernen Sie die +1-Addition:

```cpp
// NEW (CORRECT)
timingState.unixSec = gpsEpoch;  // Direct, no adjustment
```

## Expected Impact

**Test vor Hotfix:**
```
w32tm /stripchart /computer:10.0.0.13
o:+02.5388866s
```

**Test nach Hotfix:**
```
w32tm /stripchart /computer:10.0.0.13
o:+01.5388866s  ← ~1000ms besser!
```

## Verify

### 1. Upload & Boot
```bash
Arduino IDE → Sketch → Upload
```

### 2. Test Offset (sollte jetzt ~1500ms sein)
```bash
ntpdate -q 10.0.0.13
```

### 3. Monitor Stabilität
```bash
w32tm /stripchart /computer:10.0.0.13 /period:2 /samples:30
```

### 4. Vergleich mit anderen Servern
```bash
# Should now be close to:
# time.cloudflare.com: ~1540ms
# time.google.com: ~1540ms
# Your server: ~1540ms  ✓ Matched!
```

## Rollback

Falls +1 doch nötig ist (unwahrscheinlich), wiederherstellen:

```cpp
timingState.unixSec = gpsEpoch + 1;
```

## Changes Made

File: `ntp_server_esp32.ino` Line ~436

```diff
- timingState.unixSec = gpsEpoch + 1;
+ timingState.unixSec = gpsEpoch;
```

## Status
✅ **Applied & Ready to Test**

---

## Notes zur Baud-Rate (FAQ)

**Q: Sollte ich die GPS-Baud-Rate erhöhen?**

**A: Nein.** 9600 Baud ist optimal für:
- NMEA Sätze kommen nur 1×/Sekunde
- Jeder Satz ist <100 Zeichen
- Serial-Timing ist unkritisch (Sekunden-Genauigkeit)

Das echte Problem war die +1-Sekunde-Anpassung, nicht die Serial-Geschwindigkeit.

---

## Files Modified
- ✅ ntp_server_esp32/ntp_server_esp32.ino (1 line changed)

## Next Steps
1. Upload the hotfixed code
2. Run ntpdate test
3. Verify offset ~1500ms (not 2500ms)
4. If stable, commit as "v2.1"

