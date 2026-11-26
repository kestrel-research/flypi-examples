"""Demo 5 - Battery watchdog: hover until 25% then return home"""

# Starts at 45% and drains ~8x faster than real life, so this takes seconds.
# sim-args: --battery-start 45 --battery-speed 8

import flypi
from _common import ADDRESS, fmt, say

RTL_AT = 25  # percent

with flypi.connect(ADDRESS, timeout=15) as drone:
    t = drone.wait_ready(timeout=30)
    say(f"Battery at start: {t.battery.remaining}% ({t.battery.voltage:.2f} V)")
    if t.battery.remaining is not None and t.battery.remaining < RTL_AT + 10:
        raise SystemExit("Battery too low to start the mission")

    drone.arm()
    drone.takeoff(12)
    drone.set_mode("LOITER")
    say(f"Hovering: {fmt(drone.telemetry)}")

    # Block until the battery reaches the threshold (or 60 s pass).
    t = drone.wait_until(
        lambda t: t.battery.remaining is not None and t.battery.remaining <= RTL_AT,
        timeout=60, description=f"battery <= {RTL_AT}%",
    )
    say(f"Battery {t.battery.remaining}% <= {RTL_AT}% -> RETURN TO LAUNCH")
    drone.return_to_launch(wait=True)  # blocks until landed + disarmed
    say(f"Home safe: {fmt(drone.telemetry)}")
