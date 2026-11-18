import math
import unittest

from scripts.station_edge_contact import GRAVITY_MPS2, RADIUS_M, SLOPE_RAD, departure


class StationEdgeContactTests(unittest.TestCase):
    def test_slow_entry_follows_edge_until_normal_force_vanishes(self):
        for speed in (0.0, 0.5):
            with self.subTest(speed=speed):
                state = departure(speed)
                self.assertEqual(state["mode"], "supported-arc")
                x = state["center_x_m_from_edge"]
                z = state["center_z_m_from_edge"]
                vx = state["velocity_x_mps"]
                vz = state["velocity_z_mps"]
                self.assertGreater(x, RADIUS_M * math.sin(SLOPE_RAD))
                self.assertAlmostEqual(math.hypot(x, z), RADIUS_M)
                self.assertAlmostEqual(x * vx + z * vz, 0)
                self.assertAlmostEqual(vx**2 + vz**2, GRAVITY_MPS2 * z)
                drop = RADIUS_M * math.cos(SLOPE_RAD) - z
                self.assertAlmostEqual(vx**2 + vz**2, speed**2 + 2 * GRAVITY_MPS2 * drop)

    def test_threshold_and_fast_entries_depart_at_initial_tangent(self):
        threshold = math.sqrt(GRAVITY_MPS2 * RADIUS_M * math.cos(SLOPE_RAD))
        for speed in (threshold, 1, 2):
            with self.subTest(speed=speed):
                state = departure(speed)
                self.assertEqual(state["mode"], "immediate-separation")
                self.assertAlmostEqual(state["velocity_x_mps"], speed * math.cos(SLOPE_RAD))
                self.assertAlmostEqual(state["velocity_z_mps"], -speed * math.sin(SLOPE_RAD))
                self.assertAlmostEqual(state["separation_angle_deg_from_up"], 35)
        slow = departure(threshold - 1e-8)
        self.assertEqual(slow["mode"], "supported-arc")
        self.assertAlmostEqual(slow["center_x_m_from_edge"], departure(threshold)["center_x_m_from_edge"])

    def test_invalid_entry_speed(self):
        for speed in (-0.01, math.inf, math.nan):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                departure(speed)
