"""Demo 6 - Prototype goto(): fly a 30 m square in GUIDED, then RTL"""

# flypi v0.1 has no goto() yet (it is on the roadmap). This demo prototypes it on
# top of flypi using SET_POSITION_TARGET_GLOBAL_INT - the same message a v0.2
# goto() would send. It uses Drone._send, a private hook: fine for a prototype,
# not for production code. Works on the bundled simulator and on ArduPilot SITL.
# sim-args: --wind 1.5

import math

from pymavlink import mavutil

import flypi
from _common import ADDRESS, fmt, say

mavlink = mavutil.mavlink
EARTH_M_PER_DEG = 111_320.0


def offset(lat, lon, north_m, east_m):
    """Lat/lon of a point north_m/east_m metres away (flat-earth, fine for <1 km)."""
    return (lat + north_m / EARTH_M_PER_DEG,
            lon + east_m / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


def distance_m(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * EARTH_M_PER_DEG
    de = (lon2 - lon1) * EARTH_M_PER_DEG * math.cos(math.radians(lat1))
    return math.hypot(dn, de)


def goto(drone: flypi.Drone, lat, lon, alt, *, tolerance=1.0, timeout=60):
    """Fly to lat/lon at alt metres above home, and wait until within tolerance."""
    drone.set_mode("GUIDED")
    type_mask = 0b0000_1111_1111_1000  # use position only
    drone._send(lambda mav: mav.set_position_target_global_int_send(
        0, drone.target_system, drone.target_component,
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
        int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, 0, 0))
    return drone.wait_until(
        lambda t: distance_m(t.position.lat, t.position.lon, lat, lon) < tolerance
        and abs(t.position.alt_rel - alt) < 0.5,
        timeout=timeout, description=f"arrival at {lat:.6f},{lon:.6f}")


with flypi.connect(ADDRESS, timeout=15) as drone:
    home = drone.wait_ready(timeout=30).position
    drone.arm()
    drone.takeoff(10)
    say(f"Airborne: {fmt(drone.telemetry)}")

    square = [(30, 0, 10), (30, 30, 14), (0, 30, 14), (0, 0, 10)]  # north, east, alt
    for i, (n, e, alt) in enumerate(square, 1):
        lat, lon = offset(home.lat, home.lon, n, e)
        t = goto(drone, lat, lon, alt)
        err = distance_m(t.position.lat, t.position.lon, lat, lon)
        say(f"Waypoint {i}/4 ({n:>2} N, {e:>2} E, {alt} m) reached, error {err:.2f} m")

    drone.return_to_launch(wait=True)
    say(f"Mission complete: {fmt(drone.telemetry)}")
