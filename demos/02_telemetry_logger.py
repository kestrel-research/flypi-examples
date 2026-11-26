"""Demo 2 - Telemetry logger: record every snapshot to CSV while flying"""

import csv
import threading
import time
from pathlib import Path

import flypi
from _common import ADDRESS, fmt, say

OUT = Path("out/02_telemetry_logger/telemetry.csv")


def logger(drone: flypi.Drone, stop: threading.Event, hz: float = 5.0) -> None:
    """Poll drone.telemetry (an immutable snapshot) and append rows to a CSV."""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "mode", "armed", "lat", "lon", "alt_rel", "vn", "ve", "vd",
                    "roll", "pitch", "yaw", "voltage", "current", "remaining", "sats"])
        t0 = time.monotonic()
        while not stop.is_set():
            t = drone.telemetry
            p, v, a, b = t.position, t.velocity, t.attitude, t.battery
            w.writerow([round(time.monotonic() - t0, 2), t.mode, t.armed, p.lat, p.lon,
                        p.alt_rel, v.north, v.east, v.down, a.roll, a.pitch, a.yaw,
                        b.voltage, b.current, b.remaining, t.gps.satellites])
            stop.wait(1 / hz)


with flypi.connect(ADDRESS, timeout=15) as drone:
    stop = threading.Event()
    threading.Thread(target=logger, args=(drone, stop), daemon=True).start()
    say("Logging telemetry at 5 Hz")

    drone.wait_ready(timeout=30)
    drone.arm()
    drone.takeoff(6)
    say(f"Up: {fmt(drone.telemetry)}")

    drone.set_mode("LOITER")
    say("LOITER for 4 s")
    time.sleep(4)

    t = drone.telemetry
    say(f"Attitude roll={t.attitude.roll:+.2f} pitch={t.attitude.pitch:+.2f} yaw={t.attitude.yaw:.1f} deg")
    say(f"Battery {t.battery.voltage:.2f} V, {t.battery.current:.1f} A, {t.battery.remaining}%")

    drone.land()
    stop.set()
    time.sleep(0.3)

rows = OUT.read_text().count("\n") - 1
say(f"Wrote {rows} rows to {OUT}")
say("Recent vehicle messages: " + " | ".join(drone.status_messages[-4:]))
