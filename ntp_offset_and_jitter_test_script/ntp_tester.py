import ntplib
import time
import csv
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
#rev time time.nist.gov
servers = [
    "10.0.0.13",
    "time.cloudflare.com",
    "time.google.com",
    "0.de.pool.ntp.org",
    "1.de.pool.ntp.org"
]

INTERVAL = 5         # Sekunden zwischen Messungen
DURATION = 900      # Sekunden (15min)

CSV_FILE = "ntp_results.csv"
PLOT_FILE = "ntp_offset.png"


client = ntplib.NTPClient()

# CSV vorbereiten
with open(CSV_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "time",
        "server",
        "offset_ms",
        "delay_ms",
        "stratum"
    ])

start = time.time()

while time.time() - start < DURATION:
    now = datetime.now().isoformat()
    print("\n", now)
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        for server in servers:
            try:
                r = client.request(
                    server,
                    version=4,
                    timeout=3
                )
                offset = r.offset * 1000
                delay = r.delay * 1000
                print(
                    f"{server:25} "
                    f"Offset {offset:9.3f} ms "
                    f"Delay {delay:8.3f} ms "
                    f"Stratum {r.stratum}"
                )
                writer.writerow([
                    now,
                    server,
                    offset,
                    delay,
                    r.stratum
                ])
            except Exception as e:
                print(
                    f"{server:25} FEHLER {e}"
                )

    time.sleep(INTERVAL)

print("\nMessung beendet")
print("Erzeuge Plot...")

# Daten laden
df = pd.read_csv(CSV_FILE)

df["time"] = pd.to_datetime(df["time"])

# Plot nur dein GPS-NTP
gps = df[df.server == "10.0.0.13"]
plt.figure(figsize=(12,5))
plt.plot(
    gps["time"],
    gps["offset_ms"],
    marker="."
)
plt.title(
    "GPS NTP Server Offset"
)
plt.xlabel(
    "Zeit"
)
plt.ylabel(
    "Offset ms"
)
plt.grid(True)
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig(PLOT_FILE)
print(
    "Fertig:",
    CSV_FILE,
    PLOT_FILE
)