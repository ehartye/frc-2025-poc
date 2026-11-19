"""Independent checks on fit, source registration, and the selected sensor purchase."""
import hashlib
import json
import unittest
from pathlib import Path
from xml.etree import ElementTree

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"sp1-mount"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class SP1MountTests(unittest.TestCase):
    def test_machining_kit_binds_seven_parts_and_excludes_supplier_geometry(self):
        fit=read("fit.json")
        for path, expected in fit["source"].items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),expected)
        self.assertEqual(hashlib.sha256((STUDY/"sp1-mount.step").read_bytes()).hexdigest(),fit["stepSha256"])
        self.assertEqual(len(fit["volumeMm3"]),7)
        self.assertFalse(set(fit["volumeMm3"]) & set(fit["supplierExcluded"]))
        self.assertEqual(set(fit["volumeMm3"]),set(read("mass-input.json")["materialVolumeMm3"]))
        self.assertEqual(fit["priorKitSha256"],json.loads((ROOT/"selected-match/kit.json").read_bytes())["kitSha256"])

    def test_swept_clearance_control_and_service(self):
        fit=read("fit.json")
        self.assertEqual(fit["nativeScan"]["findings"],{})
        self.assertEqual(fit["nativeScan"]["comparisonBodiesAndReservations"],964)
        self.assertGreater(fit["oldTargetControl"]["unboredOldTabEyeMm3"],25)
        installs=fit["installation"]
        self.assertEqual(installs["grossThirdBodyCandidates"],{})
        self.assertEqual(installs["freeStrokeMm"],[25.1,395.1])
        for stage in ("0","370"):
            service=installs["serviceToolHits"][stage]
            self.assertEqual(set(service),{"5.5AF-socket","counterhold-blade","removed-pin","foot-driver-0","foot-driver-1"})
            self.assertTrue(all(v=={} for v in service.values()))
        for name,target in (("A",10.23655),("B",7.09965)):
            self.assertAlmostEqual(fit["registeredDocks"][name]["structure"]["minimum_mm"],target,places=4)

    def test_combined_signed_accounting_and_drawing(self):
        fit,mass,source=read("fit.json"),read("mass.json"),read("mass-input.json")
        self.assertEqual(source["retainedSuppliedArticle"],"slide-position-SP1-25")
        self.assertEqual(source["exchangeSha256"],fit["stepSha256"])
        self.assertEqual(set(source["removedMachined"]),{"slide-position-saddle","slide-position-target"})
        self.assertEqual(source["combinedRangeKg"],[sum(v) for v in zip(source["budgetDeltaKg"],source["priorRangeKg"])])
        self.assertAlmostEqual(mass["representedMarginKg"][0],.1838042,places=6)
        self.assertIsNone(mass["physicalMarginKg"])
        text=" ".join(ElementTree.parse(STUDY/"shop-drawing.svg").getroot().itertext())
        for datum in ("Z427.2","X −31.3","Ø4.10","10.80","3.2 × 7 × 0.5"):
            self.assertIn(datum,text)
