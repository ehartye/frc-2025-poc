import json
from pathlib import Path
import unittest


STUDY = Path(__file__).resolve().parents[1] / "fixed-height-match"


class LowMatchTest(unittest.TestCase):
    def test_reserved_blocked_and_alternate_ending(self):
        cases = json.loads((STUDY / "match.json").read_bytes())
        main = cases["reserved"]
        self.assertEqual(cases["stationHold"]["conditionalPoints"], 3)
        self.assertFalse(cases["stationHold"]["finiteIds"])
        self.assertEqual(main["observerAuthorizedL1Count"], 0)
        self.assertEqual(len(main["sixRobotPaths"]), 6)
        self.assertTrue(all(main["sixRobotPaths"].values()))
        self.assertEqual(main["finiteIds"], ["coral-A", "coral-B"])
        self.assertEqual(main["conditionalPoints"], 9)
        self.assertLess(main["parkCageScreen"]["uprightUnlimitedYawLowerMm"], 0)
        refined = cases["inboard"]
        self.assertGreater(refined["parkCageScreen"]["remainingLateralSwingMm"], 110)
        self.assertAlmostEqual(refined["finishTeleopS"], 92.899, places=3)
        self.assertEqual(refined["conditionalPoints"], 9)
        self.assertGreater(main["pairBound"][0], .105)
        self.assertGreater(main["otherRobotToSettledPipeBoundM"], 1.6)
        self.assertEqual(cases["blocked"]["firstOverlap"][:2], ["red1", "red2"])
        self.assertIsNone(cases["blocked"]["conditionalPoints"])
        self.assertEqual(cases["lateSecond"]["finiteIds"], ["coral-A"])
        self.assertEqual(cases["lateSecond"]["conditionalPoints"], 5)
        self.assertEqual(cases["returnInstead"]["conditionalPoints"], 7)
        self.assertFalse(cases["returnInstead"]["park"])

    def test_two_distinct_pieces_through_assessment(self):
        gates = json.loads((STUDY / "observer.json").read_bytes())["pieces"]
        self.assertEqual([gates[k]["immediateTimingOnlyPhases"] for k in ("coral-A", "coral-B")], [11, 12])
        self.assertTrue(all(p["issuedTimelyPhases"] == 0 and not p["matchAuthorized"] for p in gates.values()))
        placements = json.loads((STUDY / "placements.json").read_bytes())["placements"]
        for i, piece in enumerate(placements):
            self.assertEqual(len(piece["stockedPipeBounds"]), 24+i)
            self.assertGreater(piece["carriedPipeInsideMovingChassisMarginM"], 0)
            self.assertGreater(piece["finalBranchLowerMm"], 160)
            self.assertGreater(piece["stillSeconds"], 56)
            self.assertEqual(piece["supportAt138"], "sustained-paired-floor-and-lip")
        self.assertGreater(placements[1]["stockedPipeBounds"]["coral-A"]["lower_bound_m"], .077)

    def test_native_park_and_bounded_observer(self):
        native = json.loads((STUDY / "native.json").read_bytes())
        event = json.loads((STUDY / "park-event.json").read_bytes())
        self.assertEqual(native["bodyCount"], 577)
        self.assertGreater(native["firstTubeToSecondDockMm"]["gap"], 37)
        self.assertGreater(sum(native["startBumperTapeMm3"].values()), 23500)
        self.assertGreater(sum(native["parkClothInZoneMm3"].values()), 339500)
        self.assertEqual(event["volumeMm3"], native["parkClothInZoneMm3"][event["subject"]])
        self.assertEqual((event["route"], event["wholeField"]), ("supported", "pending"))
        alternate = json.loads((STUDY / "inboard-event.json").read_bytes())
        self.assertGreater(alternate["volumeMm3"], 200500)
        self.assertEqual(alternate["volumeMm3"], native["inboardClothInZoneMm3"][alternate["subject"]])
