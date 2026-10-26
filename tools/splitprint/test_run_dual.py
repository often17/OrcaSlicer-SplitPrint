"""Tests for run_dual.py against two fake Moonraker servers. Run: python -I tools/splitprint/test_run_dual.py"""

import http.server
import json
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_dual  # noqa: E402

CONFIG = "; CONFIG_BLOCK_START\n; split_axis = x\n; split_gantry_min_gap = 10\n; split_nogo_width = 4\n; CONFIG_BLOCK_END\n"


def head_file(start_x, layers, end="G28\n"):
    """A head stream: start G-code, `layers` layer syncs that each move along X, then the end sync."""
    out = f"G28\nG90\nM83\nG1 X{start_x}\n"
    for n, x in enumerate(layers):
        out += f"; SPLITPRINT_SYNC L{n}\nG1 X{x} E1 ; extrude\n"
    return out + "; SPLITPRINT_SYNC end\n" + end + CONFIG


class FakeMoonraker:
    """Records every script with the time it started and finished."""

    def __init__(self, name, delay=0.02, fail_on=None, state="ready"):
        self.name, self.delay, self.fail_on, self.state = name, delay, fail_on, state
        self.calls, self.estops = [], 0
        fake = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def reply(self, code, body):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path == "/printer/info":
                    return self.reply(200, {"result": {"state": fake.state}})
                self.reply(404, {"error": "not found"})

            def do_POST(self):
                body = self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0)
                if self.path == "/printer/emergency_stop":
                    fake.estops += 1
                    return self.reply(200, {"result": "ok"})
                if self.path == "/printer/gcode/script":
                    script = json.loads(body)["script"]
                    t0 = time.monotonic()
                    time.sleep(fake.delay)
                    fake.calls.append((t0, time.monotonic(), script))
                    if fake.fail_on and fake.fail_on in script:
                        return self.reply(400, {"error": {"message": "Move out of range"}})
                    return self.reply(200, {"result": "ok"})
                self.reply(404, {"error": "not found"})

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class RunDualTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.fakes = []

    def tearDown(self):
        for f in self.fakes:
            f.close()
        self.dir.cleanup()

    def write(self, name, text):
        path = os.path.join(self.dir.name, name)
        with open(path, "w") as f:
            f.write(text)
        return path

    def fake(self, *a, **kw):
        f = FakeMoonraker(*a, **kw)
        self.fakes.append(f)
        return f

    def run_job(self, t0_text, t1_text, t0=None, t1=None):
        p0, p1 = self.write("p.T0.gcode", t0_text), self.write("p.T1.gcode", t1_text)
        self.t0 = t0 or self.fake("T0")
        self.t1 = t1 or self.fake("T1", delay=0.05)
        return run_dual.main([p0, p1, "--t0", self.t0.url, "--t1", self.t1.url])

    def test_intervals_go_out_in_step_and_end_with_m400(self):
        code = self.run_job(head_file(0, [10, 20, 30]), head_file(100, [90, 80, 70]))
        self.assertEqual(code, 0)
        # start, 3 layers, end on each head
        for f in (self.t0, self.t1):
            self.assertEqual(len(f.calls), 5)
            self.assertTrue(all(c[2].endswith("M400") for c in f.calls))
        # T1's start G-code finishes before T0's begins.
        self.assertLessEqual(self.t1.calls[0][1], self.t0.calls[0][0])
        # Barrier: no head starts interval k+1 before both finished interval k.
        for k in range(1, 4):
            finished = max(self.t0.calls[k][1], self.t1.calls[k][1])
            self.assertGreaterEqual(min(self.t0.calls[k + 1][0], self.t1.calls[k + 1][0]), finished - 1e-3)

    def test_comments_are_not_sent(self):
        self.run_job(head_file(0, [10]), head_file(100, [90]))
        self.assertNotIn(";", "".join(c[2] for c in self.t0.calls))

    def test_an_error_on_one_head_stops_both(self):
        t1 = self.fake("T1", fail_on="X80")
        code = self.run_job(head_file(0, [10, 20, 30]), head_file(100, [90, 80, 70]), t1=t1)
        self.assertEqual(code, 1)
        self.assertEqual(self.t0.estops, 1)
        self.assertEqual(self.t1.estops, 1)
        # Nothing after the failed interval was sent.
        self.assertFalse(any("X30" in c[2] for c in self.t0.calls))

    def test_files_that_fail_the_check_are_never_sent(self):
        # Heads 5 mm apart with a 10 mm gap.
        code = self.run_job(head_file(0, [48]), head_file(100, [53]))
        self.assertEqual(code, 1)
        self.assertEqual(self.t0.calls + self.t1.calls, [])

    def test_a_printer_that_is_not_ready_is_never_sent_to(self):
        t0 = self.fake("T0", state="shutdown")
        code = self.run_job(head_file(0, [10]), head_file(100, [90]), t0=t0)
        self.assertEqual(code, 2)
        self.assertEqual(self.t0.calls + self.t1.calls, [])

    def test_a_no_go_zone_wider_than_the_gap_needs_its_position(self):
        wide = CONFIG.replace("split_nogo_width = 4", "split_nogo_width = 20")
        text0 = head_file(0, [10]).replace(CONFIG, wide)
        text1 = head_file(100, [90]).replace(CONFIG, wide)
        self.assertEqual(self.run_job(text0, text1), 1)

    def test_dry_run_sends_nothing(self):
        p0 = self.write("p.T0.gcode", head_file(0, [10]))
        p1 = self.write("p.T1.gcode", head_file(100, [90]))
        self.assertEqual(run_dual.main([p0, p1, "--dry-run"]), 0)

    def timed_file(self, start_x, x, expected):
        """One timed layer far from the other head, expected to take `expected` seconds."""
        out = f"G28\nG90\nM83\nG1 X{start_x}\n; SPLITPRINT_SYNC L0\n; SPLITPRINT_TIMED {expected}\nG1 X{x} E1 F600\n"
        return out + "; SPLITPRINT_SYNC end\nG28\n" + CONFIG

    def test_heads_keeping_to_the_timing_go_on(self):
        code = self.run_job(self.timed_file(0, 5, 0.05), self.timed_file(100, 95, 0.05),
                            t0=self.fake("T0", delay=0.05), t1=self.fake("T1", delay=0.05))
        self.assertEqual(code, 0)

    def test_a_head_falling_behind_its_timing_stops_the_print_at_the_layer_change(self):
        # T1 takes a second longer than expected: far beyond half the 0.5 s + 10 % margin.
        t0, t1 = self.fake("T0", delay=0.05), self.fake("T1", delay=1.05)
        code = self.run_job(self.timed_file(0, 5, 0.05), self.timed_file(100, 95, 0.05), t0=t0, t1=t1)
        self.assertEqual(code, 3)
        # Stopped at the barrier: the end G-code never went out, and no emergency stop was needed.
        self.assertEqual(len(t0.calls), 2)
        self.assertEqual(t0.estops + t1.estops, 0)


if __name__ == "__main__":
    unittest.main()
