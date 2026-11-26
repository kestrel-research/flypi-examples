#!/usr/bin/env python3
"""run_demo.py - start the simulator, run a flypi demo against it, plot the flight.

    python run_demo.py demos/01_first_flight.py
    python run_demo.py demos/04_error_handling.py
    python run_demo.py all                       # run every demo in demos/

Each demo can declare simulator options in a comment line, e.g.:

    # sim-args: --battery-start 40 --battery-speed 25

Results go to out/<demo name>/: output.txt (what the script printed),
trace.jsonl (the simulator's ground truth) and flight.png (the report).

To fly the same scripts against real ArduPilot SITL instead, skip this runner:
start `sim_vehicle.py -v ArduCopter` and run `python demos/01_first_flight.py`.
"""

from __future__ import annotations

import shlex
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools"))
from plot_trace import plot  # noqa: E402


def sim_args_for(demo: Path) -> list[str]:
    for line in demo.read_text().splitlines():
        if line.startswith("# sim-args:"):
            return shlex.split(line.split(":", 1)[1])
    return []


def title_for(demo: Path) -> str:
    text = demo.read_text()
    if text.startswith('"""'):
        return text.split('"""')[1].strip().splitlines()[0]
    return demo.stem


def run(demo: Path) -> int:
    out = ROOT / "out" / demo.stem
    out.mkdir(parents=True, exist_ok=True)
    trace = out / "trace.jsonl"
    extra = sim_args_for(demo)
    print(f"\n=== {demo.name}  (sim {' '.join(extra) or 'defaults'}) ===", flush=True)

    sim = subprocess.Popen(
        [sys.executable, str(ROOT / "sim" / "fake_copter.py"), "--trace", str(trace), *extra],
        stdout=subprocess.DEVNULL,
    )
    time.sleep(0.3)
    try:
        with open(out / "output.txt", "w") as log:
            proc = subprocess.Popen(
                [sys.executable, "-u", str(demo)], stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, cwd=ROOT,
            )
            for line in proc.stdout:
                print(line, end="", flush=True)
                log.write(line)
            code = proc.wait()
        time.sleep(1.0)  # capture the last moments (e.g. disarm) in the trace
    finally:
        sim.terminate()
        sim.wait()

    plot(trace, out / "flight.png", title_for(demo))
    print(f"--- exit code {code}; report: {(out / 'flight.png').relative_to(ROOT)}")
    return code


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    target = sys.argv[1]
    demos = sorted((ROOT / "demos").glob("[0-9]*.py")) if target == "all" else [Path(target)]
    codes = {d.name: run(d.resolve()) for d in demos}
    if len(codes) > 1:
        print("\nSummary:")
        for name, code in codes.items():
            print(f"  {'PASS' if code == 0 else 'FAIL'}  {name}")
    sys.exit(max(codes.values()))


if __name__ == "__main__":
    main()
