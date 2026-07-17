#!/usr/bin/env python3
"""
NTP Server Real-Time Stripchart Monitor
Real-time offset monitoring with live screen updates and final comparative plot
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
CHART_HEIGHT = 15  # Lines tall

CSV_FILE = "ntp_monitor.csv"

# Circular buffers for stripchart
buffers = {srv: deque(maxlen=CHART_WIDTH) for srv in SERVERS}
max_offset = 2000  # Initial scale
client = ntplib.NTPClient()


def scale_to_chart(offset_ms, max_val=2000):
    """Convert offset to chart position (0-14)"""
    if offset_ms is None:
        return None
    pos = int((offset_ms / max_val) * (CHART_HEIGHT - 1))
    return max(0, min(CHART_HEIGHT - 1, pos))


def draw_stripchart(buffers, max_offset, spike_count):
    """Draw ASCII stripchart for live display"""
    lines = []

    # Title
    lines.append("=" * 80)
    lines.append("NTP OFFSET STRIPCHART MONITOR (Real-Time)")
    lines.append("=" * 80)

    # Chart for each server
    for idx, server in enumerate(SERVERS):
        buffer = buffers[server]

        # Server name
        lines.append(f"\n[{idx+1}] {server:25}")
        lines.append("-" * 80)

        # Chart rows (top to bottom = high to low offset)
        for row in range(CHART_HEIGHT):
            line = "| "
            for col, offset in enumerate(buffer):
                if offset is not None:
                    pos = scale_to_chart(offset, max_offset)
                    if pos == (CHART_HEIGHT - 1 - row):
                        line += "#"  # Filled block at this row
                    elif pos > (CHART_HEIGHT - 1 - row):
                        line += "."  # Dot below this row
                    else:
                        line += " "
                else:
                    line += " "
            line += " |"
            lines.append(line)

        # Min/Max/Avg
        valid = [o for o in buffer if o is not None]
        if valid:
            min_o = min(valid)
            max_o = max(valid)
            avg_o = sum(valid) / len(valid)
            lines.append(f"| min: {min_o:8.2f} ms | avg: {avg_o:8.2f} ms | max: {max_o:8.2f} ms |")
        else:
            lines.append("| min: --- ms | avg: --- ms | max: --- ms |")

    # Footer
    lines.append("=" * 80)
    lines.append(f"Scale: 0 to {max_offset} ms | Samples: {len(buffers['10.0.0.13'])} | Spikes: {spike_count}")
    lines.append("Press Ctrl+C to stop and generate final plot...")

    return "\n".join(lines)


def clear_screen():
    """Clear terminal"""
    os.system('clear' if os.name != 'nt' else 'cls')


def generate_final_plot(buffers, spike_count, total_samples):
    """Generate final comparison plot with all servers"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
        fig.suptitle('NTP Offset Comparison - Final Results', fontsize=14, fontweight='bold')

        colors = ['red', 'blue', 'green', 'orange', 'purple']

        for idx, (server, ax) in enumerate(zip(SERVERS, axes)):
            buffer = list(buffers[server])
            x = np.arange(len(buffer))
            
            # Filter None values
            valid_indices = [i for i, v in enumerate(buffer) if v is not None]
            valid_offsets = [buffer[i] for i in valid_indices]
            
            if valid_offsets:
                ax.plot(valid_indices, valid_offsets, marker='o', linestyle='-', 
                       color=colors[idx], label=server, markersize=4)
                ax.set_ylabel('Offset (ms)')
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right')
                
                # Add statistics
                min_o = min(valid_offsets)
                max_o = max(valid_offsets)
                avg_o = sum(valid_offsets) / len(valid_offsets)
                
                stats_text = f'min:{min_o:.1f} | avg:{avg_o:.1f} | max:{max_o:.1f} ms'
                ax.text(0.02, 0.95, stats_text, transform=ax.transAxes, 
                       verticalalignment='top', fontsize=9, bbox=dict(boxstyle='round', 
                       facecolor='wheat', alpha=0.5))
            else:
                ax.text(0.5, 0.5, 'NO DATA', ha='center', va='center')
                ax.set_ylabel('Offset (ms)')

        axes[-1].set_xlabel('Sample Number')
        plt.tight_layout()
        
        # Save plot
        plot_file = "ntp_final_comparison.png"
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"\nFinal plot saved: {plot_file}")
        plt.close()

    except ImportError:
        print("\nMatplotlib not installed. Skipping plot generation.")
        print("Install with: pip install matplotlib numpy")


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

    print("NTP Offset Monitor - Initializing...")
    time.sleep(2)

    try:
        while time.time() - start_time < DURATION:
            now = datetime.now()
            elapsed = time.time() - start_time

            clear_screen()

            # Status line
            progress_pct = (elapsed / DURATION) * 100
            print(f"\nStatus: {progress_pct:.1f}% complete | Elapsed: {elapsed:.0f}s / {DURATION}s | Samples: {sample_count}\n")

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

            # Draw chart
            print(draw_stripchart(buffers, max_offset, spike_count))

            sample_count += 1
            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n\nStopped by user - Generating final plot...\n")

    # Final statistics
    print("\n" + "=" * 80)
    print("MEASUREMENT COMPLETE - FINAL STATISTICS")
    print("=" * 80)
    print(f"Total samples: {sample_count}")
    print(f"Total spikes (>200ms): {spike_count}")
    print(f"Data saved to: {CSV_FILE}")

    print("\nPer-Server Summary:")
    print("-" * 80)
    for server in SERVERS:
        valid = [o for o in buffers[server] if o is not None]
        if valid:
            print(f"{server:25} | min: {min(valid):8.2f} | avg: {sum(valid)/len(valid):8.2f} | max: {max(valid):8.2f} ms")
        else:
            print(f"{server:25} | NO DATA")

    # Generate final plot
    print("\n" + "=" * 80)
    generate_final_plot(buffers, spike_count, sample_count)
    print("=" * 80)


if __name__ == "__main__":
    try:
        monitor()
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        sys.exit(1)
