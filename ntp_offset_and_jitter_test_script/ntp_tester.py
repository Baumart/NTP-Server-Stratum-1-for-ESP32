#!/usr/bin/env python3
"""
NTP Server Stripchart Monitor
Real-time graphical offset monitoring in CLI
"""

import ntplib
import time
import csv
from datetime import datetime
from collections import deque
import sys
import os

# Configuration
SERVERS = [
    "10.0.0.13",
    "time.cloudflare.com",
    "time.google.com",
    "0.de.pool.ntp.org",
    "1.de.pool.ntp.org"
]

INTERVAL = 5  # Seconds between measurements
DURATION = 1800  # 30 minutes
CHART_WIDTH = 60  # Characters wide
CHART_HEIGHT = 20  # Lines tall

CSV_FILE = "ntp_monitor.csv"

# Circular buffers for stripchart
buffers = {srv: deque(maxlen=CHART_WIDTH) for srv in SERVERS}
max_offset = 2000  # Initial scale

client = ntplib.NTPClient()


def scale_to_chart(offset_ms, max_val=2000):
    """Convert offset to chart position (0-19)"""
    if offset_ms is None:
        return None
    # Clamp to range
    pos = int((offset_ms / max_val) * (CHART_HEIGHT - 1))
    return max(0, min(CHART_HEIGHT - 1, pos))


def draw_stripchart(buffers, max_offset):
    """Draw ASCII stripchart"""
    lines = []

    # Title
    lines.append("+" + "=" * (CHART_WIDTH + 2) + "+")
    lines.append("| NTP OFFSET STRIPCHART (ms) " + " " * (CHART_WIDTH - 26) + "|")
    lines.append("+" + "=" * (CHART_WIDTH + 2) + "+")

    # Chart for each server
    for server in SERVERS:
        buffer = buffers[server]

        # Scale line
        lines.append(f"| {server:18} |")

        # Chart rows (top to bottom = high to low offset)
        for row in range(CHART_HEIGHT):
            line = "| " + " " * 18 + "|" + " " * CHART_WIDTH
            chars = list(line)

            # Plot points for this row
            for col, offset in enumerate(buffer):
                if offset is not None:
                    pos = scale_to_chart(offset, max_offset)
                    if pos == (CHART_HEIGHT - 1 - row):
                        chars[21 + col] = "#"  # Filled block
                    elif pos > (CHART_HEIGHT - 1 - row):
                        chars[21 + col] = "."  # Dot below

            lines.append("".join(chars))

        # Min/Max/Avg
        valid = [o for o in buffer if o is not None]
        if valid:
            min_o = min(valid)
            max_o = max(valid)
            avg_o = sum(valid) / len(valid)
            stat_line = (f"| {server:18}| "
                         f"min:{min_o:7.1f} avg:{avg_o:7.1f} max:{max_o:7.1f} ms")
            lines.append(stat_line)

        lines.append("+" + "-" * (CHART_WIDTH + 2) + "+")

    # Scale reference
    lines.append(f"| Scale: 0 to {max_offset} ms over {CHART_WIDTH} samples |")
    lines.append("+" + "=" * (CHART_WIDTH + 2) + "+")

    return "\n".join(lines)


def clear_screen():
    """Clear terminal"""
    os.system('clear' if os.name != 'nt' else 'cls')


def monitor():
    """Main monitoring loop"""
    global max_offset

    # CSV header
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "server", "offset_ms", "delay_ms", "stratum"])

    start_time = time.time()
    sample_count = 0
    spike_count = 0
    last_offset = {}

    print("NTP Offset Monitor - Warming up...")

    while time.time() - start_time < DURATION:
        now = datetime.now()
        elapsed = time.time() - start_time

        clear_screen()
        print(f"Elapsed: {elapsed:.0f}s / {DURATION}s | Samples: {sample_count}")
        print()

        # Measure all servers
        for server in SERVERS:
            try:
                r = client.request(server, version=4, timeout=3)
                offset = r.offset * 1000  # Convert to ms
                delay = r.delay * 1000

                # Detect spikes
                if server in last_offset:
                    delta = abs(offset - last_offset[server])
                    if delta > 200:  # 200ms jump
                        spike_count += 1
                        spike_marker = " [SPIKE]"
                    else:
                        spike_marker = ""
                else:
                    spike_marker = ""

                last_offset[server] = offset

                # Update buffer
                buffers[server].append(offset)

                # Update scale
                if offset > max_offset * 0.8:
                    max_offset = max(max_offset, offset * 1.2)

                # Log to CSV
                with open(CSV_FILE, "a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([now.isoformat(), server, offset, delay, r.stratum])

            except Exception as e:
                buffers[server].append(None)
                print(f"[ERROR] {server}: {e}")

        # Draw chart
        print(draw_stripchart(buffers, max_offset))

        print(f"\nSpikes detected (>200ms): {spike_count}")
        print(f"Logging to: {CSV_FILE}")
        print("\nPress Ctrl+C to stop...")

        sample_count += 1
        time.sleep(INTERVAL)

    print("\nMeasurement complete!")
    print(f"Total samples: {sample_count}")
    print(f"Total spikes: {spike_count}")
    print(f"Data saved to: {CSV_FILE}")


if __name__ == "__main__":
    try:
        monitor()
    except KeyboardInterrupt:
        print("\n\nStopped by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        sys.exit(1)
