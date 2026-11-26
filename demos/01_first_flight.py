"""Demo 1 - First flight: connect, arm, take off, hover, land"""

import time

import flypi
from _common import ADDRESS, fmt, say

say(f"Connecting to {ADDRESS} ...")
with flypi.connect(ADDRESS, timeout=15) as drone:
    say(f"Connected: {drone!r}")
    say(f"Autopilot={drone.autopilot}  type={drone.vehicle_type}")

    say("Waiting for pre-arm checks and a 3D GPS fix ...")
    t = drone.wait_ready(timeout=30)
    say(f"Ready. GPS fix {t.gps.fix_type} with {t.gps.satellites} sats, battery {t.battery.voltage:.2f} V")

    drone.arm()
    say("Armed")

    drone.takeoff(10)  # blocks until ~10 m
    say(f"Reached altitude -> {fmt(drone.telemetry)}")

    for _ in range(5):  # hover and watch telemetry
        time.sleep(1)
        say(f"hovering   {fmt(drone.telemetry)}")

    say("Landing ...")
    drone.land()  # blocks until landed AND disarmed
    say(f"Landed and disarmed -> {fmt(drone.telemetry)}")
