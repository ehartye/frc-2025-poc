import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

STUDY = Path(__file__).resolve().parents[1] / "moving-harness"
spec = importlib.util.spec_from_file_location("camera_harness_routing", STUDY/"routing.py")
routing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(routing)


class MovingCameraHarnessTest(unittest.TestCase):
    def test_rolling_bend_conserves_length_and_reaches_handoff(self):
        for stroke in range(371):
            sections = routing.rolling_loop(stroke,-329)
            self.assertAlmostEqual(sum(length for length,_ in sections),23*30.5)
            np.testing.assert_allclose(sections[-1][1][-1],[-72.5+stroke,-329,800])
            self.assertLessEqual(sections[0][1][-1,0],-5)
            self.assertLessEqual(sections[-1][1][0,0],-72.5+stroke)

    def test_reopened_shop_and_electrical_receipts(self):
        geom = json.loads((STUDY/"geometry.json").read_bytes())
        receipt = json.loads((STUDY.parent/"camera-datum-shop/native-place.json").read_bytes())
        self.assertEqual(geom["shopStepSha256"],receipt["step_sha256"])
        self.assertEqual(geom["nativeBodies"],603)
        self.assertGreater(geom["carrier"]["clearanceLowerMm"],11)
        self.assertGreater(geom["rejectedLaneRailIntersectionMm3"],700)
        field = json.loads((STUDY/"field.json").read_bytes())
        prior = json.loads((STUDY.parent/"full-match/installed-stock.json").read_bytes())
        self.assertEqual(field["fieldSha256"],prior["field_step_sha256"])
        self.assertGreater(field["station"]["lowerMm"],7)
        self.assertGreater(min(v["occupiedBallLowerMm"] for v in field["sectors"].values()),19)
        report = json.loads((STUDY/"power/review.json").read_bytes())
        self.assertEqual([row["status"] for row in report["trials"]],["complete","incomplete"])
        self.assertTrue(all(row["outcome"]["status"]=="unknown" for row in report["trials"]))
