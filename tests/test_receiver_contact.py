import math
import unittest

from scripts import receiver_contact as model


class ReceiverContactTests(unittest.TestCase):
    def test_extended_shelf_first_contact(self):
        for speed in model.DEFAULT_SPEEDS_MPS:
            with self.subTest(speed=speed):
                result = model.screen(speed, model.EXTENDED_FRONT)
                self.assertEqual(result["screen"], "first-shelf-contact-on-top")
                self.assertGreater(result["minimum_panel_gap_lower_bound_m"], 0.005)
                self.assertLess(result["normal_velocity_mps_at_plane"], 0)
                state, time, path = model.path_to_shelf(speed)
                self.assertAlmostEqual(path[-1][1], model.SHELF_TOP + model.RADIUS_M)
                self.assertTrue(all(p[1] > path[-1][1] for p in path[:-1]))
                after = model.flight_center(state, time + 0.0001)
                self.assertLess(after[1] - model.RADIUS_M, model.SHELF_TOP)
                self.assertAlmostEqual(result["center_x_m_at_plane"], path[-1][0])

    def test_retracting_shelf_removes_contact_through_crossing(self):
        for speed in model.DEFAULT_SPEEDS_MPS:
            with self.subTest(speed=speed):
                result = model.screen(speed, model.RETRACTED_FRONT)
                self.assertEqual(result["screen"], "short-of-shelf-through-plane-crossing")
                self.assertLess(result["center_x_m_at_plane"] + model.RADIUS_M, model.RETRACTED_FRONT)

    def test_near_corner_is_left_unresolved(self):
        result = model.screen(0.5, 0.09)
        self.assertEqual(result["screen"], "shelf-corner-contact-unresolved")

    def test_premature_slow_flight_intersects_lip(self):
        # Negative control: pretend the slow entry detaches at the 35-degree tangent.
        speed = 0.5
        dt = 0.005
        x, z = model.supported_center(model.SLOPE_RAD)
        x += speed * math.cos(model.SLOPE_RAD) * dt
        z -= speed * math.sin(model.SLOPE_RAD) * dt + model.GRAVITY_MPS2 * dt**2 / 2
        gap = math.hypot(x - model.LIP_X, z - model.LIP_Z) - model.RADIUS_M
        self.assertAlmostEqual(gap * 1000, -0.04254, places=4)

    def test_panel_enclosure_detects_possible_interval_contact(self):
        # Both endpoints clear the corner while the enclosed interval could cross it.
        points = [(0, model.PANEL_TOP + 0.09), (0.09, model.PANEL_TOP)]
        self.assertLess(model.panel_clearance_bound(points), 0)
