import json
import unittest

from mission import moving_traffic as mt


class MovingTrafficTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results=mt.matrix()

    def test_checked_in_matrix_reproduces(self):
        self.assertEqual(self.results,json.loads((mt.ROOT/"mission/moving-traffic.json").read_text()))

    def test_different_robots_block_the_two_early_departures(self):
        for key,blocker in (("immediate","red3"),("opponent","blue3")):
            row=self.results["scenarios"][key]
            self.assertEqual(row["first_sampled_overlap"][0:2],[blocker,"red2"])
            self.assertLess(row["first_sampled_overlap"][3],0)
            self.assertFalse(row["separated_for_full_clock"])
            self.assertEqual(row["support_observations"],[])
            self.assertEqual(row["source_arrivals_before_clock"],[])

    def test_reservation_and_native_witnesses(self):
        row=self.results["scenarios"]["reserved"]
        self.assertGreater(row["min_separating_bound_m"],.099)
        self.assertGreater(row["other_robot_to_target_min_bound_m"],1.34)
        self.assertTrue(row["empty_finish_available"])
        self.assertLess(row["planned_empty_finish_s"],135)
        self.assertEqual(row["source_arrivals_before_clock"],["coral-A","coral-B"])
        for edition in mt.assess_coral.EDITIONS:
            samples=row["edition_assessments"][edition]["samples"]
            self.assertEqual([s["pieces"][0]["l1CriteriaAtInstant"] for s in samples],["yes","yes"])

    def test_station_resource_and_match_clock_limit_deliveries(self):
        cases=self.results["scenarios"]
        self.assertEqual(cases["late"]["source_arrivals_before_clock"],["coral-A","coral-B"])
        self.assertEqual(cases["depleted"]["source_arrivals_before_clock"],["coral-A"])
        for name in ("late","depleted"):
            self.assertEqual([p["id"] for p in cases[name]["support_observations"]],["coral-A"])
            self.assertFalse(cases[name]["empty_finish_available"])
        self.assertIsNone(mt.feeder(0,0,0,"A",{"A":True}))
        with self.assertRaises(ValueError):
            mt.analyze(stock=(0,float("nan")))
