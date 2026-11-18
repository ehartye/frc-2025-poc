import json
import unittest

from scripts.kickoff_route_clearance import MODEL, check_route, sweep
from scripts.kickoff_field_occupancy import boundary_gap, cage_height_screen, choke_screen


class FiniteSpace(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = json.loads(MODEL.read_text(encoding="utf-8"))
        cls.constraints = json.loads((MODEL.parent / "field-constraints.json").read_text(encoding="utf-8"))

    def test_swept_hit_between_clear_endpoints_and_parallel_miss(self):
        square = [(-1,-1),(1,-1),(1,1),(-1,1)]
        hit, gap = sweep((-4,0), (4,0), square, square)
        self.assertAlmostEqual(hit, 0.25)
        self.assertEqual(gap, 0)
        self.assertEqual(sweep((-4,3), (4,3), square, square), (None, 1))

    def test_station_and_reef_finite_envelopes(self):
        self.assertAlmostEqual(boundary_gap(self.model, self.constraints, [45,42], 15)["gapIn"], -1.092305842)
        route = next(r for r in self.model["routeHypotheses"]
                     if r["phase"] == "TELEOP" and r["destination"] == "net")
        self.assertAlmostEqual(check_route(self.model, route, 0)["firstContactDistanceIn"], 10.144077109)
        route = {**route, "routeIn": [route["routeIn"][0], [228,101], route["routeIn"][-1]]}
        self.assertAlmostEqual(check_route(self.model, route, 0)["minimumPlanarGapIn"], 4.683557368)

    def test_clock_does_not_count_post_auto_assessment_as_drive_time(self):
        report = choke_screen(self.model, self.constraints, "AUTO", [12.6,13])
        self.assertEqual(report["openingIn"], 35.5)
        self.assertFalse(report["twoTrialWidthsFit"])
        first, second = report["reservationHypotheses"]
        self.assertTrue(first["completesWithinActivePhase"])
        self.assertAlmostEqual(second["waitSeconds"], 5/6)
        self.assertFalse(second["completesWithinActivePhase"])
        self.assertAlmostEqual(second["exitSeconds"], 15+1/15)

    def test_low_cage_and_high_cage_differ_for_24_inch_box(self):
        self.assertEqual(cage_height_screen(self.constraints["barge"], 24),
                         {"deep": True, "shallow": False})
        self.assertTrue(cage_height_screen(self.constraints["barge"], 29.375)["shallow"])
