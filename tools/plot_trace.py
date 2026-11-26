#!/usr/bin/env python3
"""plot_trace.py - turn a fake_copter trace (JSON lines) into a flight report PNG.

    python tools/plot_trace.py out/01_first_flight/trace.jsonl -o report.png --title "First flight"

Panels: top-down track, altitude over time (with flight-mode strip and the
commands flypi sent), and battery over time.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
TRACK = "#2a78d6"
# Fixed mode -> colour mapping (categorical slots in fixed order), never by rank.
MODE_COLORS = {
    "GUIDED": "#2a78d6",
    "RTL": "#eb6834",
    "LOITER": "#1baf7a",
    "LAND": "#6250d6",
    "STABILIZE": "#b9b8b2",
    "ALT_HOLD": "#e87ba4",
}
OTHER_MODE = "#8f8e88"


def load(path: Path):
    states, events = [], []
    for line in path.read_text().splitlines():
        rec = json.loads(line)
        (events if "event" in rec else states).append(rec)
    return states, events


def mode_spans(states):
    spans = []
    for s in states:
        if spans and spans[-1][2] == s["mode"]:
            spans[-1][1] = s["t"]
        else:
            spans.append([s["t"], s["t"], s["mode"]])
    return spans


def style(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=11, color=INK, fontweight="bold")
    ax.grid(True, color=GRID, linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)


def plot(trace: Path, out: Path, title: str) -> None:
    states, events = load(trace)
    if not states:
        raise SystemExit(f"{trace} has no state records")
    t = [s["t"] for s in states]
    t_end = t[-1]

    fig = plt.figure(figsize=(13, 7.5), facecolor=SURFACE)
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.9], height_ratios=[3, 0.35, 1.6],
                          hspace=0.45, wspace=0.18)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_mode = fig.add_subplot(gs[1, 1], sharex=ax_alt)
    ax_bat = fig.add_subplot(gs[2, 1], sharex=ax_alt)
    fig.suptitle(title, x=0.06, ha="left", fontsize=15, color=INK, fontweight="bold")

    # --- top-down track ---------------------------------------------------
    style(ax_map, "Track (top-down, metres from home)")
    e = [s["e"] for s in states]
    n = [s["n"] for s in states]
    ax_map.plot(e, n, color=TRACK, linewidth=2, solid_capstyle="round")
    ax_map.plot(0, 0, marker="^", markersize=11, color=INK, linestyle="none")
    ax_map.annotate("home", (0, 0), xytext=(8, -14), textcoords="offset points",
                    fontsize=9, color=INK_2)
    far = max(states, key=lambda s: s["n"] ** 2 + s["e"] ** 2)
    if far["n"] ** 2 + far["e"] ** 2 > 1:
        ax_map.plot(far["e"], far["n"], marker="o", markersize=8, color=TRACK,
                    markeredgecolor=SURFACE, markeredgewidth=2, linestyle="none")
        dist = (far["n"] ** 2 + far["e"] ** 2) ** 0.5
        ax_map.annotate(f"furthest {dist:.0f} m", (far["e"], far["n"]), xytext=(8, 6),
                        textcoords="offset points", fontsize=9, color=INK_2)
    span = max(10.0, max(abs(v) for v in e + n) * 1.25)
    ax_map.set_xlim(-span, span)
    ax_map.set_ylim(-span, span)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("east (m)")
    ax_map.set_ylabel("north (m)")

    # --- altitude -------------------------------------------------------------
    style(ax_alt, "Altitude above home (m)")
    ax_alt.plot(t, [s["alt"] for s in states], color=TRACK, linewidth=2)
    # armed periods as a faint band
    armed_start = None
    for s in states + [{"t": t_end, "armed": False}]:
        if s["armed"] and armed_start is None:
            armed_start = s["t"]
        elif not s["armed"] and armed_start is not None:
            ax_alt.axvspan(armed_start, s["t"], color="#eeedea", zorder=0)
            armed_start = None
    lost = [s["t"] for s in states if not s.get("link", True)]
    if lost:
        ax_alt.axvspan(lost[0], lost[-1], color="#fde8e8", zorder=0)
        ax_alt.annotate("link lost", (lost[0], 0), xytext=(4, 4), textcoords="offset points",
                        fontsize=9, color="#b42318")
    top = max(1.0, max(s["alt"] for s in states)) * 1.3
    ax_alt.set_ylim(-0.3, top)
    # commands flypi sent (selective labels)
    cmds = [ev for ev in events if ev["event"] == "cmd"]
    level = 0
    last_t = -99.0
    for ev in cmds:
        ax_alt.axvline(ev["t"], color=INK_2, linewidth=0.8, linestyle=(0, (2, 3)), zorder=1)
        level = level + 1 if ev["t"] - last_t < 2.5 else 0
        last_t = ev["t"]
        label = ev["text"].replace("COMPONENT_", "").replace("DO_SET_MODE", "SET_MODE")
        if label.startswith("SET_MODE") and "params" in ev:
            from pymavlink import mavutil
            label = "mode " + mavutil.mode_mapping_acm.get(int(ev["params"][1]), "?")
        elif label.startswith("ARM_DISARM") and "params" in ev:
            label = "arm" if ev["params"][0] >= 0.5 else "disarm"
        elif label.startswith("NAV_TAKEOFF"):
            label = f"takeoff {ev['params'][2]:g} m"
        elif label.startswith("position target"):
            label = "goto"
        ax_alt.annotate(label, (ev["t"], top), xytext=(3, -12 - 11 * (level % 3)),
                        textcoords="offset points", fontsize=8, color=INK_2)
    ax_alt.legend(handles=[Patch(color="#eeedea", label="armed")], loc="upper right",
                  frameon=False, fontsize=8, bbox_to_anchor=(1.0, 1.18))
    plt.setp(ax_alt.get_xticklabels(), visible=False)

    # --- mode strip -------------------------------------------------------
    ax_mode.set_facecolor(SURFACE)
    seen = []
    for t0, t1, mode in mode_spans(states):
        color = MODE_COLORS.get(mode, OTHER_MODE)
        ax_mode.axvspan(t0, max(t1, t0 + 0.1), color=color, linewidth=0)
        if mode not in seen:
            seen.append(mode)
        if t1 - t0 > 0.08 * t_end:
            ax_mode.text((t0 + t1) / 2, 0.5, mode, ha="center", va="center", fontsize=8,
                         color="white" if mode != "STABILIZE" else INK, fontweight="bold")
    ax_mode.set_yticks([])
    ax_mode.set_ylabel("mode", rotation=0, ha="right", va="center", fontsize=9, color=INK_2)
    for side in ax_mode.spines.values():
        side.set_visible(False)
    plt.setp(ax_mode.get_xticklabels(), visible=False)
    ax_mode.tick_params(length=0)
    ax_mode.legend(handles=[Patch(color=MODE_COLORS.get(m, OTHER_MODE), label=m) for m in seen],
                   loc="upper left", bbox_to_anchor=(0, -0.1), ncol=len(seen), frameon=False,
                   fontsize=8)

    # --- battery ----------------------------------------------------------
    style(ax_bat, "Battery remaining (%)")
    ax_bat.plot(t, [s["batt"] for s in states], color=TRACK, linewidth=2)
    lo = min(s["batt"] for s in states)
    ax_bat.set_ylim(max(0, lo - 10), 101)
    if lo < 20:
        ax_bat.axhline(20, color="#b42318", linewidth=1, linestyle=(0, (4, 3)))
        ax_bat.annotate("autopilot low-battery warning (20%)", (0, 20), xytext=(4, 3),
                        textcoords="offset points", fontsize=8, color="#b42318")
    ax_bat.set_xlabel("time since simulator start (s)")
    ax_bat.set_xlim(0, t_end)

    fig.savefig(out, dpi=130, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("trace", type=Path)
    p.add_argument("-o", "--out", type=Path, default=None)
    p.add_argument("--title", default="flypi flight")
    a = p.parse_args()
    plot(a.trace, a.out or a.trace.with_suffix(".png"), a.title)


if __name__ == "__main__":
    main()
