import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "conditioned-match"

def read(filename):
    return json.loads((HERE / filename).read_bytes())

class ConditionedMatchTests(unittest.TestCase):
    def test_docking_requires_both_power_and_actual_stereo_before_the_feed(self):
        configs = read("tuning.json")
        final = read("opening.json")
        for name, source in final.items():
            self.assertTrue(any(r["case"] == name and r["wheelAccelMmS2"] == 290 and r["pace"] == .94
                                and r["status"] == "observed stationary" for r in configs))
            self.assertEqual(source["status"], "observed stationary")
            self.assertLess(source["wallS"], 21.1)
            self.assertEqual(source["disableTranslationMm"], 0)
            self.assertEqual(source["disableHeadingChangeDeg"], 0)
            self.assertGreater(min(source["stationaryPeerTo153Mm"].values()), 2200)
            self.assertGreater(source["leaveFullFootprintAcrossTapeMm"], 0)
            for pack in source["power"].values():
                self.assertIsNone(pack["firstReferenceInhibitWallS"])
                self.assertLess(pack["peakMotorPhaseA"], 38)
                self.assertGreater(pack["minTerminalV"], 9.5)
            self.assertGreater(read("power.json")[name]["fresh"]["peakMotorPhaseA"], 38)
        self.assertTrue(any(r["case"] == "rightSmall" and r["wheelAccelMmS2"] == 230 and r["pace"] == .94
                            and r["status"] != "observed stationary" for r in configs))

    def test_native_tails_and_late_camera_have_separate_coordinates(self):
        pads = read("pad.json")
        self.assertEqual(pads["stepSha256"], hashlib.sha256((ROOT/"release-observer/stereo-mount.step").read_bytes()).hexdigest())
        for rows in pads["cases"].values():
            self.assertEqual(len({r["nativeId"] for r in rows}), 3)
            self.assertTrue(all(1.2 < r["maximumInsideMm"] < 1.5 for r in rows))
        for side,nav in read("navigation.json").items():
            self.assertEqual(nav["corrected"]["decodedTagFrames"], 0)
            self.assertGreater(nav["corrected-mono"]["decodedTagFrames"], 20)
            self.assertFalse(nav["corrected-mono"]["accepted"])
            for mode in ("corrected","corrected-mono"):
                gaps=read("occupancy.json")[side+"-"+mode]
                self.assertGreater(gaps["outbound"]["minimumBarrierMm"], 10)
                self.assertGreater(gaps["outbound"]["minimumStockMm"], 0)
                self.assertGreater(gaps["outbound"]["minimumPeerMm"], 0)
                self.assertGreater(min(gaps["atRestPeerMm"].values()), 0)

    def test_two_distinct_stock_identities_and_source_bound_acquisition(self):
        ledger = read("match.json")
        for case in ledger["cases"]:
            self.assertFalse(case["hardwareQualified"])
            self.assertEqual(case["released"], [])
            self.assertEqual(case["secondPipe"], "coral-B stays at station")
            self.assertEqual(case["placementPoints"]+case["parkPoints"], 0)
            self.assertEqual(case["events"][-1]["wallS"], 156)
            self.assertEqual([e["wallS"] for e in case["events"]], sorted(e["wallS"] for e in case["events"]))
            if case["material"].startswith("elastic") or case["material"]=="lower-damping":
                self.assertFalse(case["detectionOnlyDrive"])
        high = [v for v in read("feeds.json") if v["dtS"] == .00003125]
        self.assertEqual(len(high), 2)
        for row in high:
            self.assertEqual(row["sourceRobotMmDeg"], read("opening.json")[row["case"]]["sourceRobotMmDeg"])
            if row["state"]=="IN_BAND":
                pair=read("seat-images.json")[row["case"]]
                self.assertEqual(pair["trajectorySha256"], row["trajectorySha256"])
                self.assertEqual(len(pair["frames"]), 5)
                self.assertTrue(all(f["containsSimulatedCenter"] for f in pair["frames"]))

    def test_historical_positive_and_source_specific_negatives_remain_replayable(self):
        original=json.loads((ROOT/"relieved-match/match.json").read_bytes())["cases"]["nominal"]
        self.assertEqual(original["points"],9)
        self.assertEqual(read("match.json")["preselectedSouth"]["park"],2)
        self.assertIn("Two pipes on one match clock",(HERE/"replay.html").read_text(encoding="utf8"))
        for case in read("power-review.json")["cases"].values():
            self.assertEqual(len(case["review"]["trials"]),2)
