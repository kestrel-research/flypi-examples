"""Shared helpers for the demos (not part of flypi)."""

import os
import time

# Same scripts fly the bundled simulator, ArduPilot SITL, or real hardware:
#   FLYPI_ADDRESS=tcp:127.0.0.1:5760 python demos/01_first_flight.py
ADDRESS = os.environ.get("FLYPI_ADDRESS", "udp:127.0.0.1:14550")

_T0 = time.monotonic()


def say(text: str) -> None:
    """Print with a timestamp so you can follow the flight in the output."""
    print(f"[{time.monotonic() - _T0:5.1f}s] {text}", flush=True)


def fmt(t) -> str:
    """One-line summary of a flypi Telemetry snapshot."""
    p, v, b = t.position, t.velocity, t.battery
    alt = f"{p.alt_rel:5.1f} m" if p.alt_rel is not None else "  ?  "
    gs = f"{v.groundspeed:4.1f} m/s" if v.groundspeed is not None else "?"
    batt = f"{b.remaining}%" if b.remaining is not None else "?"
    return f"{t.mode:<9} armed={str(t.armed):<5} alt={alt}  speed={gs}  batt={batt}"
