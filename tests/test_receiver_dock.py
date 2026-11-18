import json
import math
import unittest

from scripts import receiver_dock


class DockingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = json.loads(receiver_dock.receiver.DESIGN.read_text())

    def case(self, yaw, lateral=0, extension=240):
        return receiver_dock.dock(self.design,0.5,yaw,lateral,extension,6)

    def test_rotated_bumper_extents_are_worst_of_four_corners(self):
        for angle in (-15, -2, 0, 5, 12):
            case = self.case(angle)
            yaw = math.radians(angle)
            corners = [dx*math.cos(yaw)-dy*math.sin(yaw) for dx in (-425,425) for dy in (-425,425)]
            self.assertAlmostEqual(min(corners)+case["after_retreat_chassis_x_mm"],9.525)
        self.assertLess(self.case(2)["bumper_gap_at_original_pose_mm"],0)

    def test_centered_finite_width_and_side_edge(self):
        self.assertAlmostEqual(self.case(0)["flat_contact_length_mm"],301.625)
        self.assertAlmostEqual(self.case(0,100)["flat_contact_length_mm"],260.8125)
        self.assertEqual(self.case(0,400)["flat_contact_length_mm"],0)

    def test_mirrored_offsets_mirror_the_contact_segment(self):
        a = self.case(5,-45)["flat_contact_interval_y_mm"]
        b = self.case(-5,45)["flat_contact_interval_y_mm"]
        self.assertIsNotNone(a)
        self.assertAlmostEqual(a[0],-b[1])
        self.assertAlmostEqual(a[1],-b[0])

    def test_extended_shelf_retains_more_cross_tube_intersection(self):
        self.assertAlmostEqual(self.case(2)["flat_fraction"],1)
        self.assertLess(self.case(2,extension=220)["flat_fraction"],0.26)
        self.assertTrue(0 < self.case(5)["flat_fraction"] < 0.5)
        self.assertEqual(self.case(10)["flat_fraction"],0)
