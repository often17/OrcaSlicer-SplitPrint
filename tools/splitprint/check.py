#!/usr/bin/env python3
"""Independent collision check for SplitPrint dual-stream G-code.

Two G-code files, one per toolhead, in bed coordinates. Both contain the same
sequence of ``; SPLITPRINT_SYNC <id>`` markers. A sync is a barrier: neither head
continues past it until both have reached it.

An interval between two syncs is checked one of two ways:

* Untimed (the default): the heads run with no timing guarantee, so the check
  takes the whole stretch of the split axis each head can occupy in the interval
  and requires max(T0) + gap <= min(T1).
* Timed (it starts with ``; SPLITPRINT_TIMED``): both heads start the interval
  together and the slicer kept them apart by the time each move takes. This check
  estimates those times itself, with a model of Klipper's motion planner, and
  requires the gap to hold at every moment even when either head runs up to
  ``margin_abs + margin_rel * t`` seconds ahead of the other (t = time since the
  interval started).

Also:

* No-go zone: at most one head is inside [band_lo, band_hi] at a time (for the
  whole interval when untimed). Without --band this is not checked on its own; it
  follows from the gap rule when the zone is no wider than the gap.
* Shared Z: Z changes only after a sync and before any extrusion in that
  interval, and both heads reach the same Z in the same interval.

Moves before the first sync (start G-code, homing) are not checked, and neither
are moves after the ``end`` sync (each head's own end G-code, which may home).
Both are reported as warnings.

Usage: check.py T0.gcode T1.gcode --axis x --gap 60 [--band 45 55] [--timing-margin 10]
Exit code 0 = safe, 1 = violation, 2 = bad input.
"""

import argparse
import collections
import math
import re
import sys
from dataclasses import dataclass, field

END_SYNC = "end"  # written by the slicer before each head's end G-code
SYNC_RE = re.compile(r";\s*SPLITPRINT_SYNC\s+(\S+)")
TIMED_RE = re.compile(r";\s*SPLITPRINT_TIMED\b")
WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
PARAM_RE = re.compile(r"(\w+)\s*=\s*([-+]?(?:\d+\.?\d*|\.\d+))")
CONFIG_RE = re.compile(r";\s*(\w+)\s*=\s*(.*)$")
TIME_BIN = 0.05  # s, time resolution of the timed check


@dataclass
class Move:
    """One line of a timed interval, for the time model."""
    line: int
    lo: float              # swept along the split axis
    hi: float
    dist: float = 0.0      # XYZ length
    axes: tuple = (0.0, 0.0, 0.0)  # unit direction
    dz: float = 0.0
    de: float = 0.0
    feed: float = 0.0      # mm/s
    accel: float = 0.0
    accel_to_decel: float = 0.0  # Klipper's smoothed acceleration, which caps the peak speed
    scv: float = 5.0
    dwell: float = 0.0
    time: float = 0.0


@dataclass
class Interval:
    sync_id: str            # sync that opens the interval
    lo: float = math.inf    # extent along the split axis
    hi: float = -math.inf
    lo_line: int = 0
    hi_line: int = 0
    in_band_line: int = 0   # first line that enters the band, 0 = never
    z_after: float = None   # Z reached in this interval, if it changes
    bad_z_line: int = 0     # Z change after an extrusion in this interval
    extruded: bool = False
    timed: bool = False
    start_u: float = 0.0    # where the head is when the interval starts
    end_u: float = 0.0
    moves: list = field(default_factory=list)

    def touch(self, u, line):
        if u < self.lo:
            self.lo, self.lo_line = u, line
        if u > self.hi:
            self.hi, self.hi_line = u, line


@dataclass
class Stream:
    path: str
    band: tuple = None
    intervals: list = field(default_factory=list)
    unchecked_moves: int = 0  # before the first sync
    unchecked_end: int = 0    # after the end sync
    config: dict = field(default_factory=dict)


def first_number(text, default):
    m = re.match(r"\s*([-+]?(?:\d+\.?\d*|\.\d+))", text or "")
    return float(m.group(1)) if m else default


def read_config(path):
    """The slicer's settings block at the end of the file (comments '; key = value')."""
    cfg = {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            c = CONFIG_RE.match(raw.strip())
            if c:
                cfg[c.group(1)] = c.group(2).strip()
    return cfg


def parse(path, axis, band):
    """Split one stream into sync intervals and record what each one sweeps."""
    s = Stream(path, band)
    s.config = read_config(path)
    max_v = first_number(s.config.get("machine_max_speed_x"), 500.0)
    max_z = first_number(s.config.get("machine_max_speed_z"), 10.0)
    accel = first_number(s.config.get("default_acceleration"), 0.0) or \
        first_number(s.config.get("machine_max_acceleration_extruding"), 3000.0)
    scv = 5.0
    cruise_ratio, accel_to_decel = 0.5, None  # Klipper's default minimum_cruise_ratio
    feed = 50.0
    pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "E": 0.0}
    absolute, abs_e = True, True
    cur = None
    ended = False
    a = axis.upper()
    with open(path, encoding="utf-8", errors="replace") as f:
        for n, raw in enumerate(f, 1):
            m = SYNC_RE.search(raw)
            if m and not ended and m.group(1) == END_SYNC:
                ended = True
                continue
            if ended:
                if raw.split(";", 1)[0].strip():
                    s.unchecked_end += 1
                continue
            if m:
                if cur is not None:
                    cur.end_u = pos[a]
                cur = Interval(m.group(1), start_u=pos[a], end_u=pos[a])
                s.intervals.append(cur)
                cur.touch(pos[a], n)  # the head sits here while waiting
                if band and band[0] < pos[a] < band[1]:
                    cur.in_band_line = n
                continue
            if cur is not None and TIMED_RE.match(raw.strip()):
                cur.timed = True
                continue
            code = raw.split(";", 1)[0].strip()
            if not code:
                continue
            cmd = code.split()[0].upper()
            if cmd == "SET_VELOCITY_LIMIT":
                p = {k.upper(): float(v) for k, v in PARAM_RE.findall(code)}
                accel = p.get("ACCEL", accel)
                scv = p.get("SQUARE_CORNER_VELOCITY", scv)
                max_v = p.get("VELOCITY", max_v)
                if "MINIMUM_CRUISE_RATIO" in p:
                    cruise_ratio, accel_to_decel = p["MINIMUM_CRUISE_RATIO"], None
                elif "ACCEL_TO_DECEL" in p:
                    accel_to_decel = p["ACCEL_TO_DECEL"]
                continue
            words = {k.upper(): float(v) for k, v in WORD_RE.findall(code)}
            if cmd == "G90":
                absolute = abs_e = True
            elif cmd == "G91":
                absolute = abs_e = False
            elif cmd == "M82":
                abs_e = True
            elif cmd == "M83":
                abs_e = False
            elif cmd == "M204":
                if "S" in words:
                    accel = words["S"]
                elif "P" in words and "T" in words:
                    accel = min(words["P"], words["T"])
            elif cmd == "G4":
                if cur is not None:
                    cur.moves.append(Move(n, pos[a], pos[a], dwell=words.get("P", 0.0) / 1000.0 + words.get("S", 0.0)))
            elif cmd == "G92":
                for k in "XYZE":
                    if k in words:
                        pos[k] = words[k]
            elif cmd == "G28":
                if cur is not None:
                    raise SystemExit(f"{path}:{n}: G28 after the first sync cannot be checked")
            elif cmd in ("G0", "G1", "G2", "G3"):
                if "F" in words:
                    feed = words["F"] / 60.0
                start = dict(pos)
                for k in "XYZ":
                    if k in words:
                        pos[k] = words[k] if absolute else pos[k] + words[k]
                if "E" in words:
                    new_e = words["E"] if abs_e else pos["E"] + words["E"]
                    extruding = new_e > pos["E"] and ("X" in words or "Y" in words)
                    de = new_e - pos["E"]
                    pos["E"] = new_e
                else:
                    extruding, de = False, 0.0
                if cur is None:
                    s.unchecked_moves += 1
                    continue
                pts = [start[a], pos[a]]
                d = [pos[k] - start[k] for k in "XYZ"]
                dist = math.sqrt(sum(v * v for v in d))
                if cmd in ("G2", "G3"):
                    # Conservative: the arc stays inside its full circle.
                    i, j = words.get("I", 0.0), words.get("J", 0.0)
                    c = start[a] + (i if a == "X" else j)
                    r = math.hypot(i, j)
                    pts += [c - r, c + r]
                    cx, cy = start["X"] + i, start["Y"] + j
                    a0 = math.atan2(start["Y"] - cy, start["X"] - cx)
                    a1 = math.atan2(pos["Y"] - cy, pos["X"] - cx)
                    sweep = (a0 - a1) if cmd == "G2" else (a1 - a0)
                    sweep = sweep % (2 * math.pi) or 2 * math.pi
                    dist = math.hypot(r * sweep, d[2])
                    d = [0.0, 0.0, 0.0]  # no direction: treated as a stop on each side
                for u in pts:
                    cur.touch(u, n)
                if band and not cur.in_band_line and max(pts) > band[0] and min(pts) < band[1]:
                    cur.in_band_line = n
                if pos["Z"] != start["Z"]:
                    if cur.extruded and not cur.bad_z_line:
                        cur.bad_z_line = n
                    cur.z_after = pos["Z"]
                if extruding:
                    cur.extruded = True
                axes = tuple(v / dist for v in d) if dist > 0 and any(d) else (0.0, 0.0, 0.0)
                v = min(feed, max_v)
                if d[2] and dist > 0:
                    v = min(v, max_z * dist / abs(d[2]))
                a2d = accel_to_decel if accel_to_decel is not None else accel * (1.0 - cruise_ratio)
                cur.moves.append(Move(n, min(pts), max(pts), dist, axes, d[2], de, v, accel, min(a2d, accel) or accel, scv))
    if cur is not None:
        cur.end_u = pos[a]
    for iv in s.intervals:
        if iv.timed:
            plan_times(iv.moves)
    return s


def plan_times(moves):
    """Seconds each move takes, with Klipper's look-ahead: junction speeds from the square
    corner velocity and trapezoids with the current acceleration. The interval starts and
    ends at rest (each one ends with M400)."""
    # Groups of moves that flow into each other; extruder-only moves, arcs and dwells stand alone.
    runs, run = [], []
    for mv in moves:
        if mv.dist > 0 and any(mv.axes) and mv.accel > 0 and mv.feed > 0:
            run.append(mv)
            continue
        if run:
            runs.append(run)
            run = []
        if mv.dwell:
            mv.time = mv.dwell
        elif mv.dist > 0 and mv.feed > 0:
            mv.time = trapezoid(mv.dist, mv.feed ** 2, max(mv.accel, 1.0), 0.0, 0.0, mv.accel_to_decel)
        elif mv.de and mv.feed > 0:
            mv.time = abs(mv.de) / mv.feed
    if run:
        runs.append(run)
    for run in runs:
        n = len(run)
        cruise_v2 = [mv.feed ** 2 for mv in run]
        delta_v2 = [2.0 * mv.accel * mv.dist for mv in run]
        max_start = [0.0] * n
        for k in range(1, n):
            p, mv = run[k - 1], run[k]
            cos_theta = -sum(x * y for x, y in zip(p.axes, mv.axes))
            sin_d2 = math.sqrt(max(0.5 * (1.0 - cos_theta), 0.0))
            cos_d2 = math.sqrt(max(0.5 * (1.0 + cos_theta), 0.0))
            v2 = min(cruise_v2[k], cruise_v2[k - 1])
            if 1.0 - sin_d2 > 0.0 and cos_d2 > 0.0:
                r_jd = sin_d2 / (1.0 - sin_d2)
                jd = mv.scv ** 2 * (math.sqrt(2.0) - 1.0)  # junction deviation times accel
                quarter_tan = 0.25 * sin_d2 / cos_d2
                v2 = min(v2, r_jd * jd, delta_v2[k] * quarter_tan, delta_v2[k - 1] * quarter_tan)
            max_start[k] = v2
        # Backward: the fastest each move may end so the next ones can still slow down in time.
        end_v2 = [0.0] * n
        nxt = 0.0
        for k in range(n - 1, -1, -1):
            end_v2[k] = nxt
            nxt = min(max_start[k], nxt + delta_v2[k])
        # Forward: what each move can actually reach from where the last one ended.
        start = 0.0
        for k in range(n):
            start = min(start, max_start[k]) if k else 0.0
            end = min(end_v2[k], start + delta_v2[k])
            run[k].time = trapezoid(run[k].dist, cruise_v2[k], run[k].accel, start, end, run[k].accel_to_decel)
            start = end


def trapezoid(dist, cruise_v2, accel, start_v2, end_v2, accel_to_decel=0.0):
    # Klipper caps the peak of a short move with the smoothed acceleration, so it spends part of
    # every move cruising (minimum_cruise_ratio).
    peak_v2 = min(cruise_v2, (start_v2 + end_v2) / 2.0 + (accel_to_decel or accel) * dist)
    v0, v1, vp = math.sqrt(start_v2), math.sqrt(end_v2), math.sqrt(peak_v2)
    d_acc = max(peak_v2 - start_v2, 0.0) / (2.0 * accel)
    d_dec = max(peak_v2 - end_v2, 0.0) / (2.0 * accel)
    d_cruise = max(dist - d_acc - d_dec, 0.0)
    t = (vp - v0) / accel + (vp - v1) / accel
    return t + (d_cruise / vp if vp > 0 else 0.0)


def bins_of(iv, n, pick):
    """Per time bin, the furthest (pick=max) or nearest (pick=min) the head is on the split axis."""
    out = [None] * n
    t = 0.0
    for mv in iv.moves:
        k0 = min(n - 1, int(t / TIME_BIN))
        t += mv.time
        k1 = min(n - 1, int(max(t - 1e-9, 0.0) / TIME_BIN)) if mv.time > 0 else k0
        val = mv.hi if pick is max else mv.lo
        for k in range(k0, k1 + 1):
            out[k] = val if out[k] is None else pick(out[k], val)
    for k in range(min(n - 1, int(t / TIME_BIN)), n):
        out[k] = iv.end_u if out[k] is None else pick(out[k], iv.end_u)
    first = iv.start_u
    for k in range(n):  # before its first move the head is where it started
        if out[k] is not None:
            break
        out[k] = first
    return out, t


def timed_conflicts(a, b, gap, margin_abs, margin_rel, band):
    """Errors for one timed interval of T0 (a) and T1 (b)."""
    busy = max(sum(m.time for m in a.moves), sum(m.time for m in b.moves))
    n = int(math.ceil((busy + margin_abs + margin_rel * busy) / TIME_BIN)) + 2
    hi0, _ = bins_of(a, n, max)
    lo1, _ = bins_of(b, n, min)
    lo0, _ = bins_of(a, n, min)
    hi1, _ = bins_of(b, n, max)
    errors = []
    # Sliding window over T0's bins: both ends only move forward as k grows.
    win, win_band = collections.deque(), collections.deque()
    right = -1
    for k in range(n):
        r = int(math.ceil((margin_abs + margin_rel * (k + 1) * TIME_BIN) / TIME_BIN))
        lo_k, hi_k = max(0, k - r), min(n - 1, k + r)
        while right < hi_k:
            right += 1
            while win and hi0[win[-1]] <= hi0[right]:
                win.pop()
            win.append(right)
            if band and hi0[right] > band[0] and lo0[right] < band[1]:
                win_band.append(right)
        while win and win[0] < lo_k:
            win.popleft()
        while win_band and win_band[0] < lo_k:
            win_band.popleft()
        if hi0[win[0]] + gap > lo1[k] + 1e-6:
            errors.append(f"after sync {a.sync_id}: gantries may come closer than {gap:.3f} mm about {k * TIME_BIN:.2f} s "
                          f"into the interval (T0 up to {hi0[win[0]]:.3f}, T1 at {lo1[k]:.3f})")
            break
        if band and win_band and hi1[k] > band[0] and lo1[k] < band[1]:
            errors.append(f"after sync {a.sync_id}: both heads may be in the no-go zone about {k * TIME_BIN:.2f} s into the interval")
            break
    return errors


def check(t0, t1, gap, margin_abs=0.5, margin_rel=0.1):
    """Return a list of violation messages (empty = safe)."""
    errors = []
    ids0 = [iv.sync_id for iv in t0.intervals]
    ids1 = [iv.sync_id for iv in t1.intervals]
    if ids0 != ids1:
        k = next((i for i, (x, y) in enumerate(zip(ids0, ids1)) if x != y), min(len(ids0), len(ids1)))
        errors.append(f"sync markers differ at #{k}: T0 has {ids0[k:k+1]}, T1 has {ids1[k:k+1]}")
        return errors
    if not ids0:
        errors.append("no SPLITPRINT_SYNC markers: nothing can be checked")
        return errors
    band = t0.band
    for a, b in zip(t0.intervals, t1.intervals):
        where = f"after sync {a.sync_id}"
        if a.timed != b.timed:
            errors.append(f"{where}: only one head's interval is timed")
        elif a.timed:
            errors += timed_conflicts(a, b, gap, margin_abs, margin_rel, band)
        elif a.hi != -math.inf and b.lo != math.inf:
            if a.hi + gap > b.lo:
                errors.append(
                    f"{where}: gantries too close: T0 reaches {a.hi:.3f} ({t0.path}:{a.hi_line}), "
                    f"T1 reaches {b.lo:.3f} ({t1.path}:{b.lo_line}), need gap {gap:.3f}")
            if a.in_band_line and b.in_band_line:
                errors.append(
                    f"{where}: both heads enter the no-go zone ({t0.path}:{a.in_band_line}, {t1.path}:{b.in_band_line})")
        for s, iv in ((t0, a), (t1, b)):
            if iv.bad_z_line:
                errors.append(f"{where}: Z moves after extruding without a sync ({s.path}:{iv.bad_z_line})")
        if a.z_after != b.z_after and (a.z_after is not None or b.z_after is not None):
            errors.append(f"{where}: heads disagree on Z (T0 {a.z_after}, T1 {b.z_after})")
    return errors


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("t0")
    ap.add_argument("t1")
    ap.add_argument("--axis", choices=("x", "y"), default="x", help="split axis (the axis the gantries can't cross on)")
    ap.add_argument("--gap", type=float, required=True, help="minimum nozzle distance along the split axis, mm")
    ap.add_argument("--band", type=float, nargs=2, metavar=("LO", "HI"), help="no-go zone, mm (optional, see above)")
    ap.add_argument("--timing-margin", type=float, default=10.0,
                    help="timed intervals: how far one head may run ahead, %% of the time since the interval started")
    ap.add_argument("--timing-margin-abs", type=float, default=0.5, help="timed intervals: plus this many seconds")
    args = ap.parse_args(argv)
    band = tuple(sorted(args.band)) if args.band else None
    try:
        t0 = parse(args.t0, args.axis, band)
        t1 = parse(args.t1, args.axis, band)
    except OSError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    for s in (t0, t1):
        if s.unchecked_moves:
            print(f"warning: {s.path}: {s.unchecked_moves} moves before the first sync are not checked")
        if s.unchecked_end:
            print(f"warning: {s.path}: {s.unchecked_end} lines after the end sync are not checked")
    errors = check(t0, t1, args.gap, args.timing_margin_abs, args.timing_margin / 100.0)
    for e in errors:
        print("FAIL", e)
    if errors:
        return 1
    timed = sum(1 for iv in t0.intervals if iv.timed)
    print(f"OK: {len(t0.intervals)} sync intervals ({timed} timed), no conflicts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
