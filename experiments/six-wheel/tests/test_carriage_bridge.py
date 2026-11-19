import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"design/carriage-bridge"


def data(path):return json.loads(path.read_bytes())


class CarriageBridge(unittest.TestCase):
    def test_registration_all_mating_lands_and_rejected_void(self):
        native=data(HERE/"native.json")
        self.assertEqual(hashlib.sha256((HERE/"bridge-place.step").read_bytes()).hexdigest(),native["exportSha256"])
        source=data(ROOT/"design/competition-package/shop-duplex/red/review.json")
        for pose,r in native["poses"].items():
            self.assertEqual(r["sourceSha256"],source[pose]["export_sha256"])
            self.assertLess(r["newMaterialOutsideSourceMm3"],.001)
            self.assertEqual(len(r["contactWitnessesMm3"]),11)
            self.assertGreater(sum(r["contactWitnessesMm3"]["slide-fixed-rack-bed"]),96)
        negative=native["rejectedRackWitnessMm3"]
        self.assertAlmostEqual(negative["stock"]-negative["uncorrectedVoid"],64,places=4)
        with (HERE/"windows.csv").open(newline="") as f:
            self.assertEqual(len(list(csv.DictReader(f))),15)

    def test_mass_is_a_single_replacement_and_fault_survives(self):
        q=data(HERE/"review.json")
        for path,digest in q["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest,path)
        for new,delta in zip(q["newAllowanceKg"],q["changeKg"]):
            self.assertAlmostEqual(new-q["retiredBookedKg"],delta)
        prior=data(ROOT/"design/competition-package/shop-duplex/mass.json")
        self.assertAlmostEqual(q["R103Kg"][1],prior["poses"]["stowed"]["R103_kg"][1]+q["changeKg"][1])
        self.assertAlmostEqual(q["liftMassRangeKg"][1],prior["liftLoadedUpperKg"]+q["changeKg"][1])
        self.assertGreater(q["headroomWithLightClampsAndEndStopsKg"],.6)
        self.assertIsNone(q["asBuiltMassKg"])
        self.assertGreater(q["newLightCoast"]["worstPhase"]["coastMm"],q["oldLightCoast"]["worstPhase"]["coastMm"])
        self.assertLess(q["screwDragAt18to21"]["(0, 0.2)"]["toleranceWindowNm"],0)
        self.assertGreater(q["conditionalStructure"]["ineffectiveRail43mmPlateStrip"]["nominalRootStressMPa"],276)

    def test_panel_mesh_resolution_and_effect(self):
        result=data(HERE/"panel.json")
        for mesh in result.values():
            for row in mesh:
                self.assertLess(row["sideY"]["relativeForceResidual"],1e-7)
        fine=result["windowed"][-1]
        self.assertGreater(fine["sideY"]["meanPadDisplacementAlongLoadMm"],
                           6*result["blank"][-1]["sideY"]["meanPadDisplacementAlongLoadMm"])
        expected_area=data(HERE/"native.json")["poses"]["place"]["volumeMm3"]/5
        self.assertLess(abs(fine["analyzedAreaMm2"]-expected_area)/expected_area,.01)
