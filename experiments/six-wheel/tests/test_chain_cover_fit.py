"""Catch loss of wheel-bolt geometry, substituted gauges and stale mass credit."""
from hashlib import sha256
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "chain-covers"


class ChainCoverFit(unittest.TestCase):
    def test_repacked_stack_and_wheel_fasteners(self):
        study = json.loads((HERE / "study.json").read_bytes())
        for name, digest in study["sourceSha256"].items():
            if name != "603-body shop STEP":
                self.assertEqual(sha256((ROOT / name).read_bytes()).hexdigest(), digest, name)
        repack = study["repack"]
        self.assertEqual(sha256((HERE / "front-loop-repack.step").read_bytes()).hexdigest(), repack["stepSha256"])
        self.assertEqual(len(repack["volumeChangesMm3"]), 14)
        for side in ("L", "R"):
            for axle in ("center", "front"):
                pair = [value for key, value in repack["volumeChangesMm3"].items()
                        if key.startswith("spacer-" + axle) and key.endswith(side)]
                self.assertEqual(len(pair), 2)
                self.assertGreater(max(pair), 280)
                self.assertAlmostEqual(sum(pair), 0, places=4)
        self.assertEqual(set(repack["fit"]["adjacentSpacerToSprocketMm"].values()), {0.05})
        self.assertGreater(repack["fit"]["broadPhasePairs"], 0)
        self.assertTrue(all(new <= old+.001 for old, new in repack["fit"]["nativeOverlapBeforeAfterMm3"].values()))
        original = study["cases"]["original"]["installed"]["upper-face-L"]
        self.assertTrue(original["nearestBody"].startswith("wheel-hardware-thread"))
        self.assertLess(original["nearestMm"], .43)
        for name in ("repacked-light", "repacked-thick"):
            case = study["cases"][name]
            self.assertEqual(sha256((HERE / (name+".step")).read_bytes()).hexdigest(), case["stepSha256"])
            self.assertEqual(len(case["volumeMm3"]), 8)
            self.assertTrue(all(v > 0 for v in case["continuousDriveLowerMm"].values()))
            self.assertTrue(all(v > 1 for v in case["negativeControls"].values()))
            for side in ("L", "R"):
                for face in ("upper-face-", "lower-face-"):
                    self.assertGreater(case["installed"][face+side]["nearestMm"], 2.6)

    def test_complete_mass_estimate_keeps_attachment_open(self):
        study = json.loads((HERE / "study.json").read_bytes())
        masses = json.loads((HERE / "accounting.json").read_bytes())
        for name, mass in masses.items():
            grams = study["cases"][name]["sheetGrams"]
            self.assertAlmostEqual(mass["conditionalDeltaKg"][1]*1000, grams*1.05)
            expected = study["availableGrams"]-grams*1.05
            self.assertAlmostEqual(1000*mass["representedMarginKg"][0], expected)
            self.assertIsNone(mass["physicalMarginKg"])
        self.assertGreater(masses["repacked-light"]["representedMarginKg"][0], 0)
        self.assertLess(masses["repacked-thick"]["representedMarginKg"][0], 0)
