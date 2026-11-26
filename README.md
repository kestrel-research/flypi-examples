# flypi simulated demos

Seven runnable Python scripts that fly a simulated drone with
[flypi 0.1.0](https://pypi.org/project/flypi/). You don't need ArduPilot SITL:
a small MAVLink copter simulator is included.

```bash
pip install flypi matplotlib
python run_demo.py all                        # run all 7 demos, ~4 min
python run_demo.py demos/01_first_flight.py   # or just one
```

Each run writes `out/<demo>/`:
- `output.txt`: what the script printed
- `trace.jsonl`: the simulator's true state every 0.1 s, plus every command it received
- `flight.png`: a flight report with the top-down track, altitude with the commands
  flypi sent, a flight-mode strip, and battery

## How a script controls the drone

```
your script ──flypi call──▶ flypi ──MAVLink over UDP :14550──▶ fake_copter.py (or SITL / real FC)
   drone.arm()            COMMAND_LONG ARM_DISARM          checks pre-arm, ACKs, arms
   (blocks)          ◀──  COMMAND_ACK + HEARTBEAT(armed) ◀─ 
```

Every flypi command sends a MAVLink command and checks the `COMMAND_ACK`. It then
blocks until the vehicle's own telemetry shows the result: the heartbeat says
armed, the mode changed, the altitude was reached, or the vehicle disarmed after
landing. A background thread keeps `drone.telemetry` current and sends a 1 Hz
ground-station heartbeat.

## The demos

| # | Script | What it shows | Simulator setup |
|---|---|---|---|
| 1 | `01_first_flight.py` | `connect`, `wait_ready`, `arm`, `takeoff(10)`, hover, `land` | defaults |
| 2 | `02_telemetry_logger.py` | Logs `drone.telemetry` to CSV at 5 Hz from a thread; attitude and battery | defaults |
| 3 | `03_mode_tour.py` | `available_modes`, `set_mode` (LOITER, ALT_HOLD), `return_to_launch` + `wait_until` | defaults |
| 4 | `04_error_handling.py` | `ConnectionTimeout`, `ModeError`, `NotArmedError`, `WaitTimeout`, `CommandRejected` (with the `PreArm:` reason) | `--no-gps` |
| 5 | `05_battery_watchdog.py` | `wait_until(battery <= 25%)` triggers an RTL | 45% start, 8× drain |
| 6 | `06_goto_square_prototype.py` | Prototype of the roadmap `goto()`: flies a 30 m square with `SET_POSITION_TARGET_GLOBAL_INT` | 1.5 m/s wind |
| 7 | `07_link_loss_failsafe.py` | Link drops: flypi raises `ConnectionLost` and the autopilot's GCS failsafe RTLs | link lost at 14 s |

A demo sets its own simulator options with a `# sim-args: ...` comment, which `run_demo.py` reads.

## The simulator (`sim/fake_copter.py`)

It is an ArduCopter-like point-mass model that speaks the ArduPilot MAVLink dialect.
It copies the autopilot behaviours that matter to flypi:

- GPS fix after 3 s, and pre-arm fails with `PreArm: Need 3D Fix` until then
- Takeoff is accepted only when the vehicle is in GUIDED, armed and on the ground. It refuses to arm in LAND or RTL
- LAND and RTL disarm automatically on touchdown. If it's armed but doesn't take off within 10 s, it disarms
- RTL climbs to 15 m, flies home at 5 m/s, then lands
- GCS failsafe: if flypi's heartbeat stops for 5 s in flight, it switches to RTL
- Battery model: 5000 mAh, 18 A when hovering. It warns at 20%, and `--battery-failsafe` makes it RTL by itself

You can run it alone with `python sim/fake_copter.py --help` to see the options
(`--no-gps`, `--battery-start`, `--battery-speed`, `--wind`, `--link-loss-after`, `--home`, `--trace`).
Then, in another terminal, run any script with `python demos/<script>.py`.

The flight physics are simplified, and it is no replacement for SITL before real hardware.

## Same scripts on real ArduPilot SITL

```bash
sim_vehicle.py -v ArduCopter --console --map     # sends MAVLink to udp:127.0.0.1:14550
python demos/01_first_flight.py
# or point at another address:
FLYPI_ADDRESS=tcp:127.0.0.1:5760 python demos/03_mode_tour.py
```

Demos 4, 5 and 7 depend on simulator options. On SITL you get the same failures with
parameters instead: `SIM_GPS_DISABLE=1`, `SIM_BATT_CAP_AH`, or by killing MAVProxy.
