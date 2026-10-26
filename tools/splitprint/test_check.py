"""Tests for check.py. Run: python -I tools/splitprint/test_check.py"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check  # noqa: E402

# Band 45..55, gap 10. T0 works left, T1 works right.
ARGS = ["--axis", "x", "--gap", "10", "--band", "45", "55"]

START = "G28\nG90\nM83\n"


class CheckTest(unittest.TestCase):
    def run_pair(self, t0, t1):
        with tempfile.TemporaryDirectory() as d:
            p0, p1 = os.path.join(d, "t0.gcode"), os.path.join(d, "t1.gcode")
            with open(p0, "w") as f:
                f.write(START + t0)
            with open(p1, "w") as f:
                f.write(START + t1)
            return check.main([p0, p1] + ARGS)

    def test_heads_on_their_own_sides_pass(self):
        t0 = "G1 X10 Y10\n; SPLITPRINT_SYNC 1\nG1 X40 Y10 E1\n; SPLITPRINT_SYNC 2\n"
        t1 = "G1 X90 Y10\n; SPLITPRINT_SYNC 1\nG1 X60 Y10 E1\n; SPLITPRINT_SYNC 2\n"
        self.assertEqual(self.run_pair(t0, t1), 0)

    def test_heads_closer_than_gap_fail(self):
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 X44 E1\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X50 E1\n"
        self.assertEqual(self.run_pair(t0, t1), 1)

    def test_taking_turns_in_the_zone_passes(self):
        # T0 does its fingers while T1 waits far right, then they swap.
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 X53 E1\nG1 X30\n; SPLITPRINT_SYNC 2\nG1 X35 E1\n; SPLITPRINT_SYNC 3\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X80 E1\n; SPLITPRINT_SYNC 2\nG1 X47 E1\nG1 X70\n; SPLITPRINT_SYNC 3\n"
        self.assertEqual(self.run_pair(t0, t1), 0)

    def test_both_heads_in_the_zone_fail(self):
        # Far enough apart for the gap, but both inside the band in one interval.
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 X46 E1\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X56 E1\nG1 X54.5\n"
        self.assertEqual(self.run_pair(t0, t1), 1)

    def test_mismatched_syncs_fail(self):
        t0 = "; SPLITPRINT_SYNC 1\n; SPLITPRINT_SYNC 2\n"
        t1 = "; SPLITPRINT_SYNC 1\n"
        self.assertEqual(self.run_pair(t0, t1), 1)

    def test_z_move_after_extruding_without_sync_fails(self):
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 X20 E1\nG1 Z0.4\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X80 E1\nG1 Z0.4\n"
        self.assertEqual(self.run_pair(t0, t1), 1)

    def test_z_move_right_after_sync_passes(self):
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 Z0.4\nG1 X20 E1\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 Z0.4\nG1 X80 E1\n"
        self.assertEqual(self.run_pair(t0, t1), 0)

    def test_arc_bulging_toward_the_other_head_fails(self):
        # Endpoints stay at X40, but the arc's circle reaches X50, inside T1's gap.
        t0 = "G1 X30 Y10\n; SPLITPRINT_SYNC 1\nG2 X30 Y30 I10 J10 E1\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X58 E1\n"
        self.assertEqual(self.run_pair(t0, t1), 1)

    def test_end_gcode_after_the_end_sync_is_not_checked(self):
        # After the end sync each head homes and parks on its own, like the default end G-code.
        t0 = "G1 X10\n; SPLITPRINT_SYNC 1\nG1 X20 E1\n; SPLITPRINT_SYNC end\nG28\nG1 X90\n"
        t1 = "G1 X90\n; SPLITPRINT_SYNC 1\nG1 X80 E1\n; SPLITPRINT_SYNC end\nG28\n"
        self.assertEqual(self.run_pair(t0, t1), 0)

    def test_park_before_the_first_sync_sets_the_start_position(self):
        # The slicer parks each head at its own bed end right before the first layer sync.
        t0 = "G1 X50 Y5\nG1 X0 F7200 ; SPLITPRINT park before the first layer\n; SPLITPRINT_SYNC L0\nG1 X20 E1\n"
        t1 = "G1 X50 Y5\nG1 X100 F7200 ; SPLITPRINT park before the first layer\n; SPLITPRINT_SYNC L0\nG1 X80 E1\n"
        self.assertEqual(self.run_pair(t0, t1), 0)

    # Timed intervals: T0 goes to X44 at 10 mm/s (about 3.4 s), prints, and comes back to X20;
    # T1 waits at X90, then comes to X50, only 6 mm from where T0 was (gap 10).
    T0_TIMED = ("G1 X10\n; SPLITPRINT_SYNC L0\n; SPLITPRINT_TIMED 8\nSET_VELOCITY_LIMIT ACCEL=1000\n"
                "G1 X44 F600\nG1 Y10 E1\nG1 X20 F600\n; SPLITPRINT_SYNC L1\n")

    def t1_timed(self, wait_ms):
        return (f"G1 X90\n; SPLITPRINT_SYNC L0\n; SPLITPRINT_TIMED 9\nG4 P{wait_ms}\n"
                "SET_VELOCITY_LIMIT ACCEL=1000\nG1 X50 F600\nG1 Y10 E1\n; SPLITPRINT_SYNC L1\n")

    def test_timed_turns_with_time_to_spare_pass(self):
        # T0 is back at X20 after about 6.8 s; T1 reaches X50 after its 9 s wait and 4 s travel,
        # more than the 0.5 s + 10 % margin later.
        self.assertEqual(self.run_pair(self.T0_TIMED, self.t1_timed(9000)), 0)

    def test_timed_turns_without_time_to_spare_fail(self):
        # Same moves, but T1 only waits 0.5 s: it reaches X50 while T0 is still near X44.
        self.assertEqual(self.run_pair(self.T0_TIMED, self.t1_timed(500)), 1)

    def test_timed_interval_on_one_head_only_fails(self):
        t1 = self.t1_timed(9000).replace("; SPLITPRINT_TIMED 9\n", "")
        self.assertEqual(self.run_pair(self.T0_TIMED, t1), 1)

    def test_time_model_follows_klipper_trapezoids(self):
        # 100 mm at 100 mm/s with 1000 mm/s^2 and no cruise-ratio cap: 0.1 s up, 0.8 s cruise, 0.1 s down.
        moves = [check.Move(1, 0, 100, dist=100.0, axes=(1.0, 0.0, 0.0), feed=100.0, accel=1000.0, accel_to_decel=1000.0)]
        check.plan_times(moves)
        self.assertAlmostEqual(moves[0].time, 1.1, places=6)
        # Two collinear moves flow into each other at full speed: no stop in between.
        moves = [check.Move(1, 0, 50, dist=50.0, axes=(1.0, 0.0, 0.0), feed=100.0, accel=1000.0, accel_to_decel=1000.0),
                 check.Move(2, 50, 100, dist=50.0, axes=(1.0, 0.0, 0.0), feed=100.0, accel=1000.0, accel_to_decel=1000.0)]
        check.plan_times(moves)
        self.assertAlmostEqual(moves[0].time + moves[1].time, 1.1, places=6)


if __name__ == "__main__":
    unittest.main()
