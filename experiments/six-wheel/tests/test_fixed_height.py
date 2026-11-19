import hashlib
import json
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1] / "design" / "fixed-height"


def receipt(name):
    return json.loads((PACKAGE / name).read_bytes())


class FixedHeightTest(unittest.TestCase):
    def test_installed_lock_geometry_and_mass_scope(self):
        native = receipt("native.json")
        for role, filename, solids in (("red", "locks.step", 50), ("low-red-place", "low-locks.step", 38)):
            self.assertEqual(native[role]["exportSha256"], hashlib.sha256((PACKAGE / filename).read_bytes()).hexdigest())
            self.assertEqual(native[role]["reopenedSolids"], solids)
        mass = receipt("review.json")["mass"]
        for color in ("blue", "red"):
            for role, margin in (("high", .615), ("low", .640)):
                case = mass[f"{role}-{color}"]
                self.assertEqual(case["unrepresented"], [])
                self.assertAlmostEqual(case["representedMarginKg"][0], margin, delta=.001)
                self.assertIsNone(case["configurationMarginKg"])
                self.assertFalse(case["orderQuantitiesKnown"])

    def test_optical_success_and_actual_rigid_contact(self):
        native = receipt("clearance.json")
        for speed, filename in (("0.45", "ports.json"), ("0.65", "ports-065.json")):
            ports = receipt(filename)
            self.assertEqual(len(ports["cases"]), 40)
            self.assertEqual({case["outcome"] for case in ports["cases"].values()}, {"IN_BAND"})
            self.assertFalse(ports["carryAuthorized"])
            self.assertEqual(native[speed]["inboundChordOverlapsMm3"], {})
            self.assertEqual(native[speed]["nativeBodiesTested"], 574)
        overlaps = native["0.45"]["annularMaterialAtMinimumMm3"]
        self.assertEqual(set(overlaps), {"grip-L-drive-22T", "grip-R-drive-22T"})
        self.assertTrue(all(volume > 2.6 for volume in overlaps.values()))
        self.assertEqual(native["0.65"]["continuousSupportedBoundingTubeMm3"], {})

    def test_separate_pipe_ids_and_post_contact_outcomes(self):
        rows = [row for row in receipt("handoff.json") if row["acquisitionCounterfeedMps"] == .65]
        for pipe, time_s in (("coral-A", 30.283), ("coral-B", 80.373)):
            trials = {row["command"]: row for row in rows if row["sourceId"] == pipe}
            self.assertEqual(trials["HISTORICAL"]["after-separation"]["status"], "inner-riser-bound")
            shaped = trials["RAMP_090"]
            self.assertEqual(shaped["startState"]["state"], "IN_BAND")
            self.assertEqual(shaped["fixed"]["status"], "receiver-recontact")
            self.assertEqual(shaped["after-separation"]["status"], "sustained-paired-floor-and-lip")
            self.assertAlmostEqual(shaped["after-separation"]["targetContactTeleopS"], time_s, delta=.001)
