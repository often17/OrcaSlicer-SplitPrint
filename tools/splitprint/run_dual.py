#!/usr/bin/env python3
"""Run a SplitPrint job on two Klipper printers that share one bed.

Each gantry has its own Klipper + Moonraker instance. This script streams each head
its own G-code (part.T0.gcode, part.T1.gcode) one sync interval at a time:

  1. Checks both files with check.py and refuses to start if they fail.
  2. Checks that both Moonraker instances report Klippy "ready".
  3. Runs the start G-code (everything before the first sync) on T1 first, then on
     T0. T1 homes towards its own end of the bed before T0 homes X and Z, so the
     two gantries never home towards each other.
  4. For every ``; SPLITPRINT_SYNC`` interval, sends both heads their lines plus a
     trailing M400 through ``POST /printer/gcode/script``. That call returns when
     the commands have run, and M400 waits for the moves to finish, so when both
     calls have returned both heads are at the barrier. Only then does the next
     interval go out.
     A timed interval (a whole layer with no stop in it) is safe only while the two
     heads keep to the slicer's timing. Its ``; SPLITPRINT_TIMED <s>`` line gives each
     head's expected time; if one head finishes more than half the timing margin
     ahead of the other, relative to those times, the print stops at that layer
     change, before the next layer.
  5. After the ``end`` sync, runs each head's end G-code (both at once). The end
     G-code must only move a head towards its own end of the bed.

If either side fails, or on Ctrl+C, it calls ``/printer/emergency_stop`` on both.

Usage:
  run_dual.py part.T0.gcode part.T1.gcode --t0 http://host:7125 --t1 http://host:7126
  run_dual.py part.T0.gcode part.T1.gcode --dry-run
"""

import argparse
import concurrent.futures
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check  # noqa: E402

SYNC_RE = re.compile(r";\s*SPLITPRINT_SYNC\s+(\S+)")
TIMED_RE = re.compile(r";\s*SPLITPRINT_TIMED\s+([-+]?(?:\d+\.?\d*|\.\d+))")
MARGIN_ABS = 0.5  # s, the slicer's fixed timing slack
CONFIG_RE = re.compile(r";\s*(\w+)\s*=\s*(.*)$")


class Job:
    """One head's G-code cut at the sync markers."""

    def __init__(self, path):
        self.path = path
        self.start = []      # lines before the first sync
        self.intervals = []  # (sync_id, lines) up to the end sync
        self.expected = {}   # interval index -> seconds, for timed intervals
        self.end = []        # lines after the end sync
        self.config = {}
        cur = self.start
        with open(path, encoding="utf-8", errors="replace") as f:
            for raw in f:
                line = raw.rstrip("\r\n")
                m = SYNC_RE.match(line.strip())
                if m and cur is not self.end:
                    if m.group(1) == check.END_SYNC:
                        cur = self.end
                    else:
                        self.intervals.append((m.group(1), []))
                        cur = self.intervals[-1][1]
                    continue
                t = TIMED_RE.match(line.strip())
                if t and self.intervals and cur is self.intervals[-1][1]:
                    self.expected[len(self.intervals) - 1] = float(t.group(1))
                    continue
                c = CONFIG_RE.match(line)
                if c:
                    self.config[c.group(1)] = c.group(2).strip()
                code = line.split(";", 1)[0].strip()
                if code:
                    cur.append(code)
        if cur is not self.end:
            raise ValueError(f"{path}: no '; SPLITPRINT_SYNC {check.END_SYNC}' marker; is it a SplitPrint head file?")


def script_of(lines):
    """One gcode/script call: the lines, then M400 so the call returns only once the moves are done."""
    return "\n".join(list(lines) + ["M400"])


class Moonraker:
    def __init__(self, name, url, timeout=None):
        self.name = name
        self.url = url.rstrip("/")
        self.timeout = timeout

    def _request(self, method, path, body=None, timeout=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.loads(r.read().decode() or "null")
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")
            raise RuntimeError(f"{self.name}: {method} {path} failed: HTTP {e.code} {detail}") from e
        except urllib.error.URLError as e:
            raise RuntimeError(f"{self.name}: cannot reach {self.url}: {e.reason}") from e

    def state(self):
        info = self._request("GET", "/printer/info", timeout=10)
        return (info or {}).get("result", {}).get("state")

    def run(self, lines):
        if not lines:
            return
        res = self._request("POST", "/printer/gcode/script", {"script": script_of(lines)}, timeout=self.timeout)
        if (res or {}).get("result") != "ok":
            raise RuntimeError(f"{self.name}: unexpected reply {res!r}")

    def emergency_stop(self):
        self._request("POST", "/printer/emergency_stop", timeout=10)


def emergency_stop_all(heads):
    for h in heads:
        try:
            h.emergency_stop()
            print(f"{h.name}: emergency stop sent", file=sys.stderr)
        except Exception as e:  # keep going: the other head must be stopped too
            print(f"{h.name}: EMERGENCY STOP FAILED: {e}", file=sys.stderr)


def timed_run(head, lines):
    t = time.monotonic()
    head.run(lines)
    return time.monotonic() - t


def run_both(pool, heads, work):
    """Runs work[i] on heads[i] at once; raises the first error as soon as it happens.
    Returns how long each head took, in seconds."""
    futures = [pool.submit(timed_run, h, w) for h, w in zip(heads, work)]
    done, _ = concurrent.futures.wait(futures, return_when=concurrent.futures.FIRST_EXCEPTION)
    for f in done:
        f.result()  # raises the error, if any
    return [f.result() for f in futures]


class DriftError(Exception):
    """The heads did not keep to the slicer's timing; both are stopped at a barrier."""


def check_drift(sync_id, expected, took, margin_rel):
    """Raises DriftError when one head ran further ahead of the other than half the margin."""
    ahead = (took[1] - expected[1]) - (took[0] - expected[0])  # > 0: T0 ran ahead of T1
    allowed = 0.5 * (MARGIN_ABS + margin_rel * max(expected))
    if abs(ahead) > allowed:
        raise DriftError(
            f"layer after sync {sync_id}: T0 took {took[0]:.2f} s (expected {expected[0]:.2f}), "
            f"T1 took {took[1]:.2f} s (expected {expected[1]:.2f}): one head ran {abs(ahead):.2f} s ahead of the "
            f"other, more than the {allowed:.2f} s allowed. The slicer's move times do not match this printer; "
            f"check the velocity and acceleration limits in printer.cfg against the printer profile, or raise "
            f"the timing margin.")


def timing_margin(job):
    """The relative timing margin the file was sliced with."""
    return float(job.config.get("split_timing_margin", "10").rstrip("%")) / 100.0


def preflight(jobs, band):
    """Checks the two files against each other; returns a list of problems."""
    problems = []
    ids = [[i for i, _ in j.intervals] for j in jobs]
    if ids[0] != ids[1]:
        problems.append("the two files have different sync markers; were they sliced together?")
    if set(jobs[0].expected) != set(jobs[1].expected):
        problems.append("the two files time different layers; were they sliced together?")
    cfg = jobs[0].config
    try:
        gap = float(cfg["split_gantry_min_gap"])
        width = float(cfg.get("split_nogo_width", "0"))
        axis = cfg.get("split_axis", "x").lower()
        margin = timing_margin(jobs[0])
    except (KeyError, ValueError):
        return problems + [f"{jobs[0].path}: no split settings in its config block"]
    if band is None and width > gap:
        problems.append(f"the no-go zone ({width} mm) is wider than the head gap ({gap} mm): pass --band LO HI")
    streams = [check.parse(j.path, axis, band) for j in jobs]
    problems += check.check(streams[0], streams[1], gap, MARGIN_ABS, margin)
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("t0_gcode")
    ap.add_argument("t1_gcode")
    ap.add_argument("--t0", help="Moonraker URL of the T0 (low side) printer, e.g. http://localhost:7125")
    ap.add_argument("--t1", help="Moonraker URL of the T1 (high side) printer, e.g. http://localhost:7126")
    ap.add_argument("--band", type=float, nargs=2, metavar=("LO", "HI"), help="no-go zone on the bed, mm")
    ap.add_argument("--dry-run", action="store_true", help="check and cut the files, send nothing")
    args = ap.parse_args(argv)

    try:
        jobs = [Job(args.t0_gcode), Job(args.t1_gcode)]
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    problems = preflight(jobs, tuple(sorted(args.band)) if args.band else None)
    if problems:
        for p in problems:
            print("FAIL", p, file=sys.stderr)
        print("Refusing to start: the collision check failed.", file=sys.stderr)
        return 1
    count = len(jobs[0].intervals)
    margin = timing_margin(jobs[0])
    print(f"Check passed: {count} sync intervals, {len(jobs[0].expected)} layers without stops.")
    if args.dry_run:
        for j, name in zip(jobs, ("T0", "T1")):
            lines = sum(len(l) for _, l in j.intervals)
            print(f"{name}: {len(j.start)} start lines, {lines} lines in intervals, {len(j.end)} end lines")
        return 0
    if not args.t0 or not args.t1:
        print("error: --t0 and --t1 are required unless --dry-run", file=sys.stderr)
        return 2

    heads = [Moonraker("T0", args.t0), Moonraker("T1", args.t1)]
    for h in heads:
        try:
            state = h.state()
        except RuntimeError as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
        if state != "ready":
            print(f"error: {h.name} Klippy state is {state!r}, not 'ready'", file=sys.stderr)
            return 2

    pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
    try:
        print("Start G-code: T1, then T0")
        heads[1].run(jobs[1].start)
        heads[0].run(jobs[0].start)
        for k in range(count):
            sync_id = jobs[0].intervals[k][0]
            if re.fullmatch(r"L\d+", sync_id):
                print(f"Layer {sync_id[1:]} ({k + 1}/{count} intervals)", flush=True)
            took = run_both(pool, heads, [jobs[0].intervals[k][1], jobs[1].intervals[k][1]])
            if k in jobs[0].expected:
                check_drift(sync_id, [jobs[0].expected[k], jobs[1].expected[k]], took, margin)
        print("End G-code")
        run_both(pool, heads, [jobs[0].end, jobs[1].end])
    except DriftError as e:
        # Both heads are idle at the barrier: nothing needs an emergency stop.
        print(f"Stopped: {e}\nBoth heads are idle; Klipper's idle timeout turns the heaters off.", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("Interrupted: stopping both printers", file=sys.stderr)
        emergency_stop_all(heads)
        return 130
    except Exception as e:
        print(f"error: {e}\nStopping both printers", file=sys.stderr)
        emergency_stop_all(heads)
        return 1
    finally:
        pool.shutdown(wait=False, cancel_futures=True)
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
