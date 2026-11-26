"""Demo 7 - Link loss: flypi raises ConnectionLost, the autopilot fails safe"""

# The simulator stops talking 14 s after it starts. With no ground-station
# heartbeat from flypi for 5 s, the autopilot's GCS failsafe switches to RTL.
# sim-args: --link-loss-after 14

import time

import flypi
from _common import ADDRESS, fmt, say

with flypi.connect(ADDRESS, timeout=15, heartbeat_timeout=3) as drone:
    drone.wait_ready(timeout=30)
    drone.arm()
    drone.takeoff(8)
    say(f"Airborne: {fmt(drone.telemetry)}")
    try:
        # A long wait that will never be satisfied - the link drops first.
        drone.wait_until(lambda t: t.position.alt_rel > 100, timeout=60, description="alt > 100 m")
    except flypi.ConnectionLost as e:
        say(f"ConnectionLost: {e}")
        say(f"is_connected={drone.is_connected}  last telemetry: {fmt(drone.telemetry)}")
        say("flypi cannot command the vehicle now. On the autopilot side, the GCS")
        say("failsafe (FS_GCS_ENABLE) takes over - see the RTL in the flight report.")
    time.sleep(22)  # keep the process alive so the plot shows the failsafe RTL
