#!/usr/bin/env python3
"""fake_copter.py - a lightweight ArduCopter-like MAVLink vehicle for testing flypi.

It speaks enough MAVLink (ArduPilot dialect) for every flypi v0.1 call to work
end to end, without installing ArduPilot SITL:

* HEARTBEAT with ArduCopter custom modes (GUIDED, LOITER, RTL, LAND, ...)
* GLOBAL_POSITION_INT, ATTITUDE, VFR_HUD, SYS_STATUS, GPS_RAW_INT, STATUSTEXT
* COMMAND_LONG: ARM/DISARM, DO_SET_MODE, NAV_TAKEOFF, SET_MESSAGE_INTERVAL
* SET_POSITION_TARGET_GLOBAL_INT in GUIDED (to prototype a future goto())

The flight model is a simple point-mass with acceleration and speed limits, not
real aerodynamics. It mimics the ArduCopter *behaviours* flypi depends on:
pre-arm checks waiting on GPS, takeoff only in GUIDED, LAND/RTL auto-disarm on
touchdown, auto-disarm 10 s after arming if you never take off, and so on.

Every 0.1 s the true vehicle state is appended to a JSON-lines trace file so
flights can be plotted afterwards (see tools/plot_trace.py).

Usage:
    python sim/fake_copter.py                         # sends to udp:127.0.0.1:14550
    python sim/fake_copter.py --no-gps                # never gets a 3D fix
    python sim/fake_copter.py --battery-start 35 --battery-speed 20
    python sim/fake_copter.py --link-loss-after 25    # stop talking after 25 s
"""

from __future__ import annotations

import argparse
import json
import math
import random
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

# ArduCopter custom-mode numbers (same table pymavlink uses to decode them).
MODES = {name: num for num, name in mavutil.mode_mapping_acm.items()}
NOT_ARMABLE_MODES = {"LAND", "RTL", "AUTO", "SMART_RTL"}
GPS_MODES = {"GUIDED", "LOITER", "RTL", "AUTO", "POSHOLD", "CIRCLE", "SMART_RTL"}

G = 9.81
M_PER_DEG_LAT = 111_320.0
RTL_ALT = 15.0            # m, like ArduCopter RTL_ALT=1500
WPNAV_SPEED = 5.0         # m/s horizontal
WPNAV_SPEED_UP = 2.5      # m/s climb
LAND_SPEED_HIGH = 1.5     # m/s above 10 m
LAND_SPEED = 0.5          # m/s final descent
MAX_ACCEL = 3.0           # m/s^2
DISARM_DELAY = 10.0       # s on the ground armed before auto-disarm
CAPACITY_MAH = 5000.0


class FakeCopter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.link = mavutil.mavlink_connection(
            args.address, source_system=args.sysid, source_component=1
        )
        self.home_lat, self.home_lon, self.home_msl = args.home
        self.t0 = time.monotonic()

        # True state (local NED relative to home, alt positive up).
        self.n = self.e = 0.0
        self.alt = 0.0
        self.vn = self.ve = self.vu = 0.0
        self.an = self.ae = 0.0
        self.yaw = 0.0

        self.armed = False
        self.mode = "STABILIZE"
        self.landed = True
        self.armed_on_ground_since: float | None = None
        self.touchdown_at: float | None = None

        self.takeoff_alt: float | None = None
        self.guided_target: tuple[float, float, float] | None = None  # n, e, alt
        self.rtl_phase: str | None = None

        self.battery_used_mah = CAPACITY_MAH * (1 - args.battery_start / 100.0)
        self.current_a = 0.5
        self.low_batt_warned = False
        self.last_gcs_heartbeat: float | None = None
        self.gcs_failsafe = False
        self.hb_now = False  # send a heartbeat right away after a state change

        self.trace = open(args.trace, "w", buffering=1) if args.trace else None
        self.statustext("ArduCopter V4.5-fake (flypi demo simulator)")
        self.statustext("Frame: QUAD/X")

    # ---------------------------------------------------------------- helpers
    def now(self) -> float:
        return time.monotonic() - self.t0

    def log(self, **record) -> None:
        if self.trace:
            record["t"] = round(self.now(), 3)
            self.trace.write(json.dumps(record) + "\n")

    def statustext(self, text: str, severity: int = mavlink.MAV_SEVERITY_INFO) -> None:
        try:
            self.link.mav.statustext_send(severity, text.encode()[:50])
        except OSError:  # nobody listening yet
            pass
        self.log(event="statustext", text=text)

    def gps_fix(self) -> int:
        if self.args.no_gps:
            return 1
        return 3 if self.now() >= self.args.gps_delay else 1

    def satellites(self) -> int:
        if self.gps_fix() < 3:
            return min(5, int(self.now()))
        return 14

    def battery_pct(self) -> float:
        return max(0.0, 100.0 * (1 - self.battery_used_mah / CAPACITY_MAH))

    def prearm_failures(self) -> list[str]:
        problems = []
        if self.gps_fix() < 3:
            problems.append("PreArm: Need 3D Fix")
        elif self.now() < self.args.gps_delay + 1.0:
            problems.append("PreArm: EKF3 waiting for GPS config data")
        if self.battery_pct() < 15:
            problems.append("PreArm: Battery 1 below minimum capacity")
        return problems

    def latlon(self) -> tuple[float, float]:
        lat = self.home_lat + self.n / M_PER_DEG_LAT
        lon = self.home_lon + self.e / (M_PER_DEG_LAT * math.cos(math.radians(self.home_lat)))
        return lat, lon

    def ne_from_latlon(self, lat: float, lon: float) -> tuple[float, float]:
        n = (lat - self.home_lat) * M_PER_DEG_LAT
        e = (lon - self.home_lon) * M_PER_DEG_LAT * math.cos(math.radians(self.home_lat))
        return n, e

    def ack(self, command: int, result: int) -> None:
        try:
            self.link.mav.command_ack_send(command, result)
        except OSError:
            pass

    # ------------------------------------------------------------ mode logic
    def set_mode(self, name: str, reason: str = "") -> bool:
        if name not in MODES:
            return False
        if name in GPS_MODES and self.gps_fix() < 3:
            self.statustext(f"Mode change to {name} failed: needs GPS")
            return False
        if name == self.mode:
            return True
        self.mode = name
        self.takeoff_alt = None
        self.guided_target = None
        self.rtl_phase = "climb" if name == "RTL" else None
        self.hb_now = True
        self.log(event="mode", mode=name, reason=reason)
        if name == "RTL" and self.landed and self.armed:
            self.disarm("RTL on ground")
        return True

    def arm(self) -> tuple[bool, str]:
        if self.armed:
            return True, ""
        problems = self.prearm_failures()
        if problems:
            for p in problems:
                self.statustext(p, mavlink.MAV_SEVERITY_CRITICAL)
            return False, problems[0]
        if self.mode in NOT_ARMABLE_MODES:
            msg = f"Arm: Mode not armable ({self.mode})"
            self.statustext(msg, mavlink.MAV_SEVERITY_CRITICAL)
            return False, msg
        self.armed = True
        self.armed_on_ground_since = self.now()
        self.hb_now = True
        self.statustext("Arming motors")
        self.log(event="armed")
        return True, ""

    def disarm(self, why: str = "") -> None:
        if not self.armed:
            return
        self.armed = False
        self.hb_now = True
        self.takeoff_alt = None
        self.guided_target = None
        self.statustext("Disarming motors" + (f" ({why})" if why else ""))
        self.log(event="disarmed", reason=why)

    # -------------------------------------------------------- message input
    def handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "HEARTBEAT" and msg.type == mavlink.MAV_TYPE_GCS:
            self.last_gcs_heartbeat = self.now()
            self.gcs_failsafe = False
        elif mtype == "COMMAND_LONG":
            self.handle_command(msg)
        elif mtype == "SET_POSITION_TARGET_GLOBAL_INT":
            if self.mode == "GUIDED" and self.armed and not self.landed:
                n, e = self.ne_from_latlon(msg.lat_int / 1e7, msg.lon_int / 1e7)
                self.guided_target = (n, e, msg.alt)
                self.takeoff_alt = None
                self.log(event="cmd", text=f"position target n={n:.1f} e={e:.1f} alt={msg.alt:.1f}")

    def handle_command(self, msg) -> None:
        cmd = msg.command
        name = mavlink.enums["MAV_CMD"][cmd].name if cmd in mavlink.enums["MAV_CMD"] else str(cmd)
        if cmd != mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.log(event="cmd", text=name.replace("MAV_CMD_", ""),
                     params=[round(p, 2) for p in (msg.param1, msg.param2, msg.param7)])

        if cmd == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.ack(cmd, mavlink.MAV_RESULT_ACCEPTED)

        elif cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            force = int(msg.param2) == 21196
            if msg.param1 >= 0.5:
                if force and not self.armed:
                    self.armed = True
                    self.armed_on_ground_since = self.now()
                    self.hb_now = True
                    self.statustext("Arming motors (forced)")
                    self.log(event="armed", forced=True)
                    ok = True
                else:
                    ok, _ = self.arm()
                self.ack(cmd, mavlink.MAV_RESULT_ACCEPTED if ok else mavlink.MAV_RESULT_FAILED)
            else:
                if not self.landed and not force:
                    self.statustext("Disarm: vehicle is flying", mavlink.MAV_SEVERITY_WARNING)
                    self.ack(cmd, mavlink.MAV_RESULT_FAILED)
                else:
                    self.disarm("commanded")
                    self.ack(cmd, mavlink.MAV_RESULT_ACCEPTED)

        elif cmd == mavlink.MAV_CMD_DO_SET_MODE:
            mode = mavutil.mode_mapping_acm.get(int(msg.param2))
            ok = mode is not None and self.set_mode(mode, "commanded")
            self.ack(cmd, mavlink.MAV_RESULT_ACCEPTED if ok else mavlink.MAV_RESULT_FAILED)

        elif cmd == mavlink.MAV_CMD_NAV_TAKEOFF:
            if self.mode != "GUIDED" or not self.armed or not self.landed:
                self.statustext("Takeoff rejected: need GUIDED, armed, on ground",
                                mavlink.MAV_SEVERITY_WARNING)
                self.ack(cmd, mavlink.MAV_RESULT_FAILED)
            else:
                self.takeoff_alt = float(msg.param7)
                self.landed = False
                self.touchdown_at = None
                self.statustext(f"Takeoff to {self.takeoff_alt:.1f} m")
                self.ack(cmd, mavlink.MAV_RESULT_ACCEPTED)
        else:
            self.ack(cmd, mavlink.MAV_RESULT_UNSUPPORTED)

    # ------------------------------------------------------------ dynamics
    def desired_velocity(self) -> tuple[float, float, float]:
        """Velocity (north, east, up) the autopilot wants in the current mode."""
        if not self.armed or self.landed:
            return 0.0, 0.0, 0.0

        def toward(tn: float, te: float, talt: float, speed: float = WPNAV_SPEED):
            dn, de, du = tn - self.n, te - self.e, talt - self.alt
            dist = math.hypot(dn, de)
            # slow down near the target: v = sqrt(2 a d)
            h = min(speed, math.sqrt(2 * 0.8 * MAX_ACCEL * dist)) if dist > 0.05 else 0.0
            vn, ve = (dn / dist * h, de / dist * h) if dist > 0.05 else (0.0, 0.0)
            vu = max(-LAND_SPEED_HIGH, min(WPNAV_SPEED_UP, 1.2 * du))
            return vn, ve, vu

        def land_rate():
            return -(LAND_SPEED_HIGH if self.alt > 10 else LAND_SPEED if self.alt < 2.5 else 1.0)

        if self.mode == "GUIDED":
            if self.takeoff_alt is not None:
                return toward(self.n, self.e, self.takeoff_alt)
            if self.guided_target is not None:
                return toward(*self.guided_target)
            return 0.0, 0.0, 0.0
        if self.mode == "LAND":
            return 0.0, 0.0, land_rate()
        if self.mode == "RTL":
            if self.rtl_phase == "climb":
                target = max(RTL_ALT, self.alt)
                if self.alt >= target - 0.3:
                    self.rtl_phase = "return"
                    self.rtl_alt = target
                    self.log(event="rtl", phase="return")
                return 0.0, 0.0, min(WPNAV_SPEED_UP, 1.2 * (target - self.alt))
            if self.rtl_phase == "return":
                if math.hypot(self.n, self.e) < 0.5 and abs(self.vn) + abs(self.ve) < 0.3:
                    self.rtl_phase = "land"
                    self.log(event="rtl", phase="land")
                return toward(0.0, 0.0, self.rtl_alt)
            vn, ve, _ = toward(0.0, 0.0, self.alt)
            return vn, ve, land_rate()
        # LOITER, ALT_HOLD, POSHOLD, BRAKE, STABILIZE...: hold position, no pilot input.
        return 0.0, 0.0, 0.0

    def step(self, dt: float) -> None:
        dvn, dve, dvu = self.desired_velocity()
        wind = self.args.wind
        gust_n = wind * (0.6 + 0.4 * math.sin(self.now() * 0.7)) if not self.landed else 0.0
        gust_e = wind * 0.3 * math.sin(self.now() * 1.3) if not self.landed else 0.0

        def approach(v, target, limit):
            delta = max(-limit * dt, min(limit * dt, target - v))
            return v + delta, delta / dt

        # wind pushes the vehicle; the position controller fights it
        self.vn, self.an = approach(self.vn, dvn + 0.15 * gust_n, MAX_ACCEL)
        self.ve, self.ae = approach(self.ve, dve + 0.15 * gust_e, MAX_ACCEL)
        self.vu, _ = approach(self.vu, dvu, MAX_ACCEL)

        if not self.armed or self.landed:
            self.vn = self.ve = self.vu = 0.0
        self.n += self.vn * dt
        self.e += self.ve * dt
        self.alt += self.vu * dt

        # touchdown detection
        if self.alt <= 0.0:
            self.alt = 0.0
            if not self.landed and self.vu <= 0:
                self.landed = True
                self.vu = 0.0
                self.touchdown_at = self.now()
                self.statustext("Land complete" if self.mode in ("LAND", "RTL") else "Touchdown")
                self.log(event="landed")

        if self.landed and self.armed:
            if self.mode in ("LAND", "RTL") and self.touchdown_at is not None:
                if self.now() - self.touchdown_at > 1.0:
                    self.disarm("landed")
            elif self.armed_on_ground_since is not None and self.takeoff_alt is None:
                since = max(self.armed_on_ground_since, self.touchdown_at or 0.0)
                if self.now() - since > DISARM_DELAY:
                    self.disarm("auto-disarm: no takeoff")
        if not self.landed and self.mode == "GUIDED" and self.takeoff_alt is not None:
            if self.alt >= self.takeoff_alt - 0.1:
                self.takeoff_alt = None  # reached, now hold
                self.guided_target = (self.n, self.e, self.alt)

        # GCS failsafe (FS_GCS_ENABLE=1): no ground-station heartbeat for 5 s while flying -> RTL
        if (self.args.gcs_failsafe and self.armed and not self.landed and not self.gcs_failsafe
                and self.last_gcs_heartbeat is not None
                and self.now() - self.last_gcs_heartbeat > 5.0):
            self.gcs_failsafe = True
            self.statustext("GCS Failsafe - RTL", mavlink.MAV_SEVERITY_CRITICAL)
            self.set_mode("RTL", "GCS failsafe")

        speed_h = math.hypot(self.vn, self.ve)
        if speed_h > 1.0:
            self.yaw = math.atan2(self.ve, self.vn)

        # battery: hover current ~ 18 A, climbing/cruise more
        if self.armed and not self.landed:
            self.current_a = 18.0 + 3.0 * max(0.0, self.vu) + 0.8 * speed_h
        elif self.armed:
            self.current_a = 3.0
        else:
            self.current_a = 0.5
        self.battery_used_mah += self.current_a * 1000 / 3600 * dt * self.args.battery_speed
        if self.battery_pct() < 20 and not self.low_batt_warned:
            self.low_batt_warned = True
            self.statustext("Battery 1 is low 14.5V used %.0f mAh" % self.battery_used_mah,
                            mavlink.MAV_SEVERITY_CRITICAL)
            if self.args.battery_failsafe and self.armed and not self.landed:
                self.set_mode("RTL", "battery failsafe")
        if self.battery_pct() <= 0.0 and self.armed and not self.landed:
            self.statustext("Battery 1 is critical - forced LAND", mavlink.MAV_SEVERITY_EMERGENCY)
            self.set_mode("LAND", "battery critical")

    # ------------------------------------------------------------- output
    def send_heartbeat(self) -> None:
        base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED
        if self.armed:
            base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        state = mavlink.MAV_STATE_ACTIVE if self.armed else mavlink.MAV_STATE_STANDBY
        self.link.mav.heartbeat_send(
            mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base, MODES[self.mode], state,
        )

    def send_position(self) -> None:
        lat, lon = self.latlon()
        hdg = int(math.degrees(self.yaw) % 360 * 100)
        self.link.mav.global_position_int_send(
            int(self.now() * 1000) & 0xFFFFFFFF,
            int(lat * 1e7), int(lon * 1e7),
            int((self.home_msl + self.alt) * 1000), int(self.alt * 1000),
            int(self.vn * 100), int(self.ve * 100), int(-self.vu * 100), hdg,
        )

    def send_attitude(self) -> None:
        # tilt needed to produce the horizontal acceleration, rotated to body frame
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a_fwd = self.an * c + self.ae * s
        a_right = -self.an * s + self.ae * c
        noise = 0.004 * (1 + self.args.wind) if self.armed and not self.landed else 0.0
        pitch = -math.atan2(a_fwd, G) + random.gauss(0, noise)
        roll = math.atan2(a_right, G) + random.gauss(0, noise)
        self.link.mav.attitude_send(int(self.now() * 1000) & 0xFFFFFFFF,
                                    roll, pitch, self.yaw, 0, 0, 0)

    def send_vfr_hud(self) -> None:
        speed = math.hypot(self.vn, self.ve)
        throttle = 0 if not self.armed else (10 if self.landed else int(45 + 10 * self.vu))
        self.link.mav.vfr_hud_send(speed, speed, int(math.degrees(self.yaw) % 360),
                                   max(0, min(100, throttle)), self.home_msl + self.alt, self.vu)

    def send_sys_status(self) -> None:
        prearm = mavlink.MAV_SYS_STATUS_PREARM_CHECK
        gps = mavlink.MAV_SYS_STATUS_SENSOR_GPS
        present = prearm | gps | mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        health = mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        if not self.prearm_failures():
            health |= prearm
        if self.gps_fix() >= 3:
            health |= gps
        pct = self.battery_pct()
        voltage = 14.0 + 2.8 * pct / 100.0 - 0.02 * self.current_a
        self.link.mav.sys_status_send(present, present, health, 300,
                                      int(voltage * 1000), int(self.current_a * 100), int(pct),
                                      0, 0, 0, 0, 0, 0)

    def send_gps(self) -> None:
        lat, lon = self.latlon()
        fix = self.gps_fix()
        self.link.mav.gps_raw_int_send(
            int(self.now() * 1e6), fix, int(lat * 1e7) if fix >= 3 else 0,
            int(lon * 1e7) if fix >= 3 else 0, int((self.home_msl + self.alt) * 1000),
            80 if fix >= 3 else 9999, 120 if fix >= 3 else 9999,
            int(math.hypot(self.vn, self.ve) * 100), 65535, self.satellites(),
        )

    # ------------------------------------------------------------- main loop
    def run(self) -> None:
        dt = 0.02
        schedule = {  # name: (period s, sender)
            "hb": (1.0, self.send_heartbeat),
            "pos": (0.2, self.send_position),
            "att": (0.1, self.send_attitude),
            "hud": (0.25, self.send_vfr_hud),
            "sys": (0.5, self.send_sys_status),
            "gps": (0.5, self.send_gps),
        }
        next_due = {k: 0.0 for k in schedule}
        next_trace = 0.0
        prearm_nag_at = 2.0
        last = time.monotonic()
        print(f"[fake_copter] sending MAVLink to {self.args.address} (Ctrl+C to stop)", flush=True)
        while True:
            if self.args.duration and self.now() > self.args.duration:
                break
            silent = self.args.link_loss_after and self.now() > self.args.link_loss_after
            while (msg := self.link.recv_match(blocking=False)) is not None:
                if not silent and msg.get_type() != "BAD_DATA":
                    self.handle(msg)
            t = time.monotonic()
            self.step(t - last)
            last = t
            if not silent:
                if self.hb_now:
                    next_due["hb"] = 0.0
                    self.hb_now = False
                for key, (period, fn) in schedule.items():
                    if self.now() >= next_due[key]:
                        try:
                            fn()
                        except OSError:
                            pass
                        next_due[key] = self.now() + period
                if not self.armed and self.now() >= prearm_nag_at:
                    for p in self.prearm_failures():
                        self.statustext(p, mavlink.MAV_SEVERITY_CRITICAL)
                    prearm_nag_at = self.now() + 15.0
            if self.now() >= next_trace:
                lat, lon = self.latlon()
                self.log(n=round(self.n, 3), e=round(self.e, 3), alt=round(self.alt, 3),
                         vn=round(self.vn, 3), ve=round(self.ve, 3), vu=round(self.vu, 3),
                         lat=round(lat, 7), lon=round(lon, 7), mode=self.mode, armed=self.armed,
                         batt=round(self.battery_pct(), 2), cur=round(self.current_a, 2),
                         link=not silent)
                next_trace = self.now() + 0.1
            time.sleep(dt)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default="udpout:127.0.0.1:14550",
                   help="pymavlink address to send to (flypi listens on udp:127.0.0.1:14550)")
    p.add_argument("--sysid", type=int, default=1)
    p.add_argument("--home", type=float, nargs=3, default=(-35.363261, 149.165230, 584.0),
                   metavar=("LAT", "LON", "ALT_MSL"), help="home position (default: ArduPilot CMAC)")
    p.add_argument("--gps-delay", type=float, default=3.0, help="seconds until 3D fix")
    p.add_argument("--no-gps", action="store_true", help="never get a 3D fix (pre-arm fails)")
    p.add_argument("--battery-start", type=float, default=100.0, help="starting charge, percent")
    p.add_argument("--battery-speed", type=float, default=1.0, help="battery drain multiplier")
    p.add_argument("--battery-failsafe", action="store_true", help="autopilot RTLs itself at 20%%")
    p.add_argument("--no-gcs-failsafe", dest="gcs_failsafe", action="store_false",
                   help="disable the RTL when the flypi heartbeat stops for 5 s")
    p.add_argument("--wind", type=float, default=0.0, help="gust strength (m/s), adds drift + jitter")
    p.add_argument("--link-loss-after", type=float, default=0.0,
                   help="stop sending/receiving after N seconds (0 = never)")
    p.add_argument("--duration", type=float, default=0.0, help="exit after N seconds (0 = run forever)")
    p.add_argument("--trace", default="", help="write a JSON-lines flight trace here")
    args = p.parse_args()
    try:
        FakeCopter(args).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
