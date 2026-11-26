"""Demo 4 - Error handling: what flypi raises when things go wrong"""

# sim-args: --no-gps

import flypi
from _common import ADDRESS, say


def expect(label, fn, *exc_types):
    try:
        fn()
    except exc_types as e:
        say(f"OK  {label}\n        -> {type(e).__name__}: {e}")
        return e
    raise AssertionError(f"{label}: expected {exc_types}, nothing raised")


# 1. Nothing listening on this port -> ConnectionTimeout
expect("connect to an address with no vehicle",
       lambda: flypi.connect("udp:127.0.0.1:14599", timeout=2), flypi.ConnectionTimeout)

# The simulator for this demo never gets a GPS fix (--no-gps).
with flypi.connect(ADDRESS, timeout=15) as drone:
    say(f"Connected to {drone!r}")

    # 2. Typo in a mode name -> ModeError (lists valid modes)
    expect("set_mode('WARP')", lambda: drone.set_mode("WARP"), flypi.ModeError)

    # 3. Takeoff before arming -> NotArmedError (raised locally, nothing sent)
    expect("takeoff() while disarmed", lambda: drone.takeoff(5), flypi.NotArmedError)

    # 4. Bad argument -> ValueError
    expect("takeoff(-3)", lambda: drone.takeoff(-3), ValueError)

    # 5. No GPS fix -> wait_ready times out, error carries the autopilot's PreArm reason
    expect("wait_ready() with no GPS", lambda: drone.wait_ready(timeout=4), flypi.WaitTimeout)

    # 6. Arm anyway -> the autopilot refuses, CommandRejected includes "PreArm: Need 3D Fix"
    err = expect("arm() with failing pre-arm checks", drone.arm, flypi.CommandRejected)
    say(f"    err.command={err.command} err.result={err.result}")
    say(f"    err.status_messages={err.status_messages}")

    # 7. GUIDED needs a position estimate -> CommandRejected on set_mode
    expect("set_mode('GUIDED') without GPS", lambda: drone.set_mode("GUIDED"), flypi.CommandRejected)

    # Every one of these is also a flypi.FlypiError (except the ValueError):
    try:
        drone.arm()
    except flypi.FlypiError as e:
        say(f"Caught generic flypi.FlypiError: {type(e).__name__}")

    say(f"Drone is still safely disarmed: armed={drone.armed}")
