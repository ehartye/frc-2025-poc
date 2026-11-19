import unittest

from scripts.kickoff_partner_timing import next_pickup, schedule
from scripts.kickoff_shuttle import read


class PartnerTiming(unittest.TestCase):
    def test_equal_spatial_exposure_differs_with_arrival_time(self):
        session = read("trials.json")
        route = [[48,45],[122,158.5]]
        early = schedule(session["trials"][0], session, route, 80, [(0.5,3)])
        late = schedule(session["trials"][0], session, route, 80, [(2.1,4.6)])
        self.assertEqual(early["piecesByDeadline"], 2)
        self.assertEqual(late["piecesByDeadline"], 3)
        self.assertAlmostEqual(early["goalMarginS"], -0.968288733)
        self.assertAlmostEqual(late["goalMarginS"], 1.031711267)

    def test_empty_or_touching_reservations_allow_immediate_service(self):
        self.assertEqual(next_pickup(1, 1, []), 1)
        self.assertEqual(next_pickup(1, 1, [(2,3)]), 1)
        self.assertEqual(next_pickup(1, 1, [(1.5,2),(2.5,3)]), 3)
