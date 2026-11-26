"""Demo 3 - Flight-mode tour: GUIDED -> LOITER -> ALT_HOLD -> RTL"""

import time

import flypi
from _common import ADDRESS, fmt, say

with flypi.connect(ADDRESS, timeout=15) as drone:
    modes = drone.available_modes
    say(f"This vehicle accepts {len(modes)} modes, e.g. {', '.join(modes[:8])} ...")

    drone.wait_ready(timeout=30)
    drone.arm()
    drone.takeoff(8)  # takeoff() switches to GUIDED itself on ArduPilot
    say(f"Took off in {drone.mode}: {fmt(drone.telemetry)}")

    for mode in ("LOITER", "ALT_HOLD"):
        drone.set_mode(mode)  # sends DO_SET_MODE, waits for the heartbeat to confirm
        say(f"Now in {drone.mode}")
        time.sleep(3)

    say("RTL: climbs to RTL_ALT (15 m), returns home, lands, disarms")
    drone.return_to_launch()  # returns immediately (wait=False) ...

    # ... so we can watch it with wait_until():
    t = drone.wait_until(lambda t: (t.position.alt_rel or 0) > 14.5, timeout=30,
                         description="RTL climb to 15 m")
    say(f"RTL climb done: {fmt(t)}")
    t = drone.wait_until(lambda t: not t.armed, timeout=90, description="RTL landing")
    say(f"Back on the ground: {fmt(t)}")
