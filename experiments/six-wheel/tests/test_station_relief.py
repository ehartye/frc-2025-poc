"""Assert the mechanical option, the source lineages and the independent negative examples."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"station-relief"


def read(name):
    return json.loads((HERE/name).read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class StationReliefTests(unittest.TestCase):
    def test_geometry_is_source_bound_and_mass_decreases(self):
        geometry=read("geometry.json")
        for name,digest in geometry["sourceSha256"].items():
            self.assertEqual(sha(ROOT/name),digest)
        self.assertEqual(sha(HERE/"cheeks.step"),geometry["reopenedStepSha256"])
        self.assertEqual(len(geometry["stock"]),8)
        self.assertEqual(geometry["sourceBodies"],545)
        self.assertGreater(sum(v["removed_mm3"] for v in geometry["stock"].values()),1700)
        cases=geometry["cases"]["revised-pocket"]
        for row in cases:
            if row["lateral_mm"] in (.8,1.):
                self.assertGreater(row["hardware_lower_mm"],.25)
                self.assertLess(row["max_belt_excursion_mm"],geometry["outwardAvailableMm"])
        wider=next(r for r in cases if r["lateral_mm"]==1.25)
        self.assertGreater(wider["hardware_lower_mm"],0)
        self.assertGreater(wider["max_belt_excursion_mm"],geometry["outwardAvailableMm"])
        prior=next(r for r in geometry["cases"]["retained-insert-web"] if r["lateral_mm"]==.8)
        self.assertTrue(prior["unresolved_material"])
        shims=read("shims.json")
        self.assertEqual(sha(HERE/"shims.step"),shims["nativeSha256"])
        self.assertEqual(len(shims["parts"]),4)
        self.assertAlmostEqual(shims["fourShimVolumeMm3"],69.2721,places=3)

    def test_spin_finite_spring_and_driven_seat(self):
        capture=read("capture.json")
        for name,digest in capture["sourceSha256"].items():
            self.assertEqual(sha(ROOT/name),digest)
        centered,original=capture["centered"],capture["issuedCentered"]
        self.assertAlmostEqual(centered["terminalUMm"]/1000,original["seat_u_m"],places=7)
        self.assertAlmostEqual(centered["terminalSpinRadS"],original["seat_spin_rad_s"],places=7)
        for rows in capture["shimmedCorners"].values():
            self.assertEqual(len(rows),8)
            self.assertEqual({r["status"] for r in rows},{"conditional-seat"})
            self.assertGreater(min(r["minimumUMm"] for r in rows),-20)
        for name in ("lowFrictionPreload","widenBothUnchangedBelts1mm"):
            self.assertEqual(capture["controls"][name]["status"],"lost-shelf-edge")
        self.assertEqual(capture["controls"]["blockedOutward"]["status"],"hold-outboard-stop")
        for grid in read("sensitivity.json")["interiorGrids"].values():
            self.assertEqual(grid["statuses"],{"conditional-seat":27})

    def test_new_sweep_is_clear_and_old_sample_reproduces_a_material_overlap(self):
        sweep=read("swept.json")
        self.assertEqual(sha(HERE/"capture.json"),sweep["contactSha256"])
        self.assertEqual(sha(HERE/"shims.json"),sweep["shimSha256"])
        self.assertEqual(sha(HERE/"geometry.json"),sweep["geometrySha256"])
        for item in sweep["cases"]:
            minimum=min(b["hardwareLowerMm"] for b in item["bins"])
            if item["preloadN"]==11.5:
                self.assertGreater(minimum,.13)
                self.assertFalse(any(b["envelopeOverlapMm3"] for b in item["bins"]))
            else:
                self.assertEqual(minimum,0)
        witness=read("witness.json")
        self.assertEqual(sha(HERE/"capture.json"),witness["sourceSha256"])
        for case in witness["cases"]:
            if case["preloadN"]==10:
                self.assertTrue(any(h["materialMm3"]>.001 and h["body"].endswith("rear-riser") for h in case["nativeHits"]))
            else:
                self.assertFalse(case["nativeHits"])
        survey=read("survey.json")
        self.assertEqual(sha(ROOT/"station-localization/pose-study.json"),survey["sourceSha256"])
        right=survey["atNominalAlignment"]["(0.8, 0.25)"]["right"]
        self.assertGreater(right,.125)
        self.assertLess(right,.127)
        for row in read("downstream.json")["cases"].values():
            self.assertEqual(row["target"]["stop"],"sustained-paired-floor-and-lip")
