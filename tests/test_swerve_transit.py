import hashlib
import json
import unittest
from pathlib import Path


STUDY = Path(__file__).resolve().parents[1] / "experiments/swerve-drive/transit"


class LoadedCorridor(unittest.TestCase):
    def test_common_start_and_distinct_clearances(self):
        record = json.loads((STUDY / "corridor.json").read_bytes())
        previous = STUDY.parent / "motion/loaded-departure.json"
        self.assertEqual(
            record["priorMatch"], hashlib.sha256(previous.read_bytes()).hexdigest()
        )
        self.assertEqual(set(record["cases"]), {"plus", "minus"})
        sources = json.loads(previous.read_bytes())["cases"]
        for name, case in record["cases"].items():
            before = next(
                row for row in sources if row["case"] == name and row["stepUs"] == 31.25
            )["routes"]["600"]
            paths = case["routes"]
            for r in paths.values():
                self.assertEqual(
                    r["sourceWheel"], before["wheel"]["wheelStateAndEncoderTrace"]
                )
                self.assertEqual(r["sourcePipe"], before["pipe"]["terminalStateSha256"])
                self.assertAlmostEqual(r["startWallS"], 28.535)
            self.assertLess(paths["full"]["field"]["minimumStockMm"], 0)
            self.assertLess(paths["settled"]["field"]["minimumPeerMm"], 0)
            cleared = paths["south-offset"]
            for key in ("minimumBarrierMm", "minimumStockMm", "minimumPeerMm"):
                self.assertGreater(cleared["field"][key], 80)
            self.assertTrue(cleared["battery"]["weak"]["supplied"])
            self.assertAlmostEqual(
                cleared["targets"][-1][0], paths["full"]["targets"][-1][0] - 0.300
            )
            self.assertAlmostEqual(
                cleared["targets"][1][1], paths["settled"]["targets"][1][1] - 0.450
            )
            self.assertEqual(cleared["timedPath"][0], paths["full"]["timedPath"][0])
