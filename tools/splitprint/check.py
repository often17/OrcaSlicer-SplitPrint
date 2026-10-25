#!/usr/bin/env python3
"""Independent collision check for SplitPrint dual-stream G-code.

Two G-code files, one per toolhead, in bed coordinates. Both contain the same
sequence of ``; SPLITPRINT_SYNC <id>`` markers. A sync is a barrier: neither head
continues past it until both have reached it.

Between two syncs the heads run with no timing guarantee. So this check does not
simulate time. It takes the whole stretch of the split axis each head can occupy
in that interval and requires:

  1. Gantry order and gap: max(T0) + gap <= min(T1).
  2. No-go zone: at most one head enters [band_lo, band_hi].
  3. Shared Z: Z changes only after a sync and before any extrusion in that
     interval, and both heads reach the same Z in the same interval.

Moves before the first sync (start G-code, homing) are not checked, and neither
are moves after the ``end`` sync (each head's own end G-code, which may home).
Both are reported as warnings.

Usage: check.py T0.gcode T1.gcode --axis x --gap 60 --band 45 55
Exit code 0 = safe, 1 = violation, 2 = bad input.
"""

import argparse
import math
import re
import sys
from dataclasses import dataclass, field

END_SYNC = "end"  # written by the slicer before each head's end G-code
SYNC_RE = re.compile(r";\s*SPLITPRINT_SYNC\s+(\S+)")
WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


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

    def touch(self, u, line):
        if u < self.lo:
            self.lo, self.lo_line = u, line
        if u > self.hi:
            self.hi, self.hi_line = u, line


@dataclass
class Stream:
    path: str
    intervals: list = field(default_factory=list)
    unchecked_moves: int = 0  # before the first sync
    unchecked_end: int = 0    # after the end sync


def parse(path, axis, band):
    """Split one stream into sync intervals and record what each one sweeps."""
    s = Stream(path)
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
                cur = Interval(m.group(1))
                s.intervals.append(cur)
                cur.touch(pos[a], n)  # the head sits here while waiting
                if band[0] < pos[a] < band[1]:
                    cur.in_band_line = n
                continue
            code = raw.split(";", 1)[0].strip()
            if not code:
                continue
            words = {k.upper(): float(v) for k, v in WORD_RE.findall(code)}
            cmd = code.split()[0].upper()
            if cmd == "G90":
                absolute = abs_e = True
            elif cmd == "G91":
                absolute = abs_e = False
            elif cmd == "M82":
                abs_e = True
            elif cmd == "M83":
                abs_e = False
            elif cmd == "G92":
                for k in "XYZE":
                    if k in words:
                        pos[k] = words[k]
            elif cmd == "G28":
                if cur is not None:
                    raise SystemExit(f"{path}:{n}: G28 after the first sync cannot be checked")
            elif cmd in ("G0", "G1", "G2", "G3"):
                start = dict(pos)
                for k in "XYZ":
                    if k in words:
                        pos[k] = words[k] if absolute else pos[k] + words[k]
                if "E" in words:
                    new_e = words["E"] if abs_e else pos["E"] + words["E"]
                    extruding = new_e > pos["E"] if abs_e else words["E"] > 0
                    pos["E"] = new_e
                else:
                    extruding = False
                if cur is None:
                    s.unchecked_moves += 1
                    continue
                pts = [start[a], pos[a]]
                if cmd in ("G2", "G3"):
                    # Conservative: the arc stays inside its full circle.
                    i, j = words.get("I", 0.0), words.get("J", 0.0)
                    c = start[a] + (i if a == "X" else j)
                    r = math.hypot(i, j)
                    pts += [c - r, c + r]
                for u in pts:
                    cur.touch(u, n)
                if not cur.in_band_line and max(pts) > band[0] and min(pts) < band[1]:
                    cur.in_band_line = n
                if pos["Z"] != start["Z"]:
                    if cur.extruded and not cur.bad_z_line:
                        cur.bad_z_line = n
                    cur.z_after = pos["Z"]
                if extruding:
                    cur.extruded = True
    return s


def check(t0, t1, gap):
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
    for a, b in zip(t0.intervals, t1.intervals):
        where = f"after sync {a.sync_id}"
        if a.hi == -math.inf or b.lo == math.inf:
            continue
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
    ap.add_argument("--band", type=float, nargs=2, required=True, metavar=("LO", "HI"), help="no-go zone, mm")
    args = ap.parse_args(argv)
    band = tuple(sorted(args.band))
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
    errors = check(t0, t1, args.gap)
    for e in errors:
        print("FAIL", e)
    if errors:
        return 1
    print(f"OK: {len(t0.intervals)} sync intervals, no conflicts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
