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


if __name__ == "__main__":
    unittest.main()
