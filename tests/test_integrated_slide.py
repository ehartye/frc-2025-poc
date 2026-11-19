"""The combined sensor mass, camera observations and actuator traces must agree on their sources."""
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"integrated-slide"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class IntegratedSlide(unittest.TestCase):
    def test_selected_sources_and_moving_mass(self):
        mass=read("mass.json")
        for path,digest in {**mass["massSources"],**read("result.json")["sources"]}.items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest)
        self.assertEqual(mass["selected"][-3:],["clearance-fit","selected-optics","sp1-install"])
        moving=mass["moving"]
        self.assertAlmostEqual(moving["extraToIssuedPlantKg"],sum(moving["selectedMovingDeltaTermsKg"].values())+.19)
        self.assertLess(moving["conditionalExtraKg"],moving["extraToIssuedPlantKg"])
        self.assertIsNone(mass["physicalMarginKg"])
        self.assertEqual(len(mass["activeAttachments"]),12)

    def test_visible_mount_and_lost_view_controls(self):
        r=read("result.json")
        self.assertEqual(r["nativeScene"]["selectedPartsIncludingWire"],991)
        self.assertEqual(len(r["detectorRuns"]),19)
        self.assertEqual(sum(v["frames"] for v in r["detectorRuns"].values()),3994)
        self.assertTrue(all(v["matchesIssued"] for v in r["detectorRuns"].values()))
        for image in ("coral-A-fixed","coral-B-fixed"):
            eyes=read("visibility.json")["eyesLeftRight"]["selected/"+image]
            self.assertEqual([v["withinDepthTolerance"] for v in eyes],[18,2])
        lost=r["detectorRuns"]["marked/coral-B-moving-barrel-lost-eye-critical"]
        self.assertEqual(lost["statuses"],{"unbacked-end":63,"unresolved-end-curvature":7})
        self.assertEqual((r["packetCases"],r["packetEventsMatch"]),(28,True))
        self.assertNotEqual(r["newObservationRegister"],r["sources"]["selected-optics/controller-register.json"])

    def test_supported_and_rejected_controller_runs(self):
        r=read("result.json")
        self.assertEqual(len(r["rackCases"]),35)
        self.assertEqual(r["differentRackPathCount"],47)
        self.assertEqual(r["rackCases"]["ramp94"]["coral-B"]["stow"],100845)
        failed=r["rackCases"]["ramp-drag-adverse94"]
        self.assertIn("hold",failed["coral-A"])
        self.assertEqual(failed["coral-B"],{"traceDiffersFromIssued":False})
        self.assertEqual(len(r["contactPower"]),13)
        for case,trial in r["contactPower"].items():
            with self.subTest(case=case):
                self.assertEqual(trial["outcomes"],["supported"]*4)
                self.assertGreater(trial["minimumFrontM"],.0326)
                self.assertGreater(trial["power"]["0.027"]["minimumV"],9.5)
                self.assertLess(trial["power"]["0.032"]["minimumV"],9.5)
