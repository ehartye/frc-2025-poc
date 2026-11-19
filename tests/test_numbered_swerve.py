from collections import Counter
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/"cad/full-swerve-r3"
NUMBERED=ROOT/"cad/full-swerve-r4"

def read(path):
    return json.loads(path.read_bytes())

class NumberedSwerve(unittest.TestCase):
    def test_numbers_preserve_the_source_robot_and_public_boundary(self):
        before=read(BASE/"appearances.json")["solids"]
        result=read(NUMBERED/"appearances.json")
        self.assertEqual(result["sourceSha256"],hashlib.sha256((NUMBERED/"full-robot.step").read_bytes()).hexdigest())
        current={v["id"]:v for v in result["solids"]}
        self.assertEqual(len(current),len(before)+16)
        for old in before:
            new=current[old["id"]]
            self.assertEqual((old["role"],old["representation"]),(new["role"],new["representation"]))
            self.assertLess(max(abs(a-b) for a,b in zip(old["boundsMm"],new["boundsMm"])),.001)
        self.assertEqual(sum(v["representation"].startswith("public-") for v in current.values()),27)

    def test_each_outward_1780_has_full_native_cloth_backing(self):
        ink=read(NUMBERED/"lettering.json")["impressions"]
        rows={r["id"]:r for r in read(NUMBERED/"appearances.json")["solids"]}
        for site in ("front","rear","left","right"):
            for i,digit in enumerate("1780",1):
                key=f"bumper-lettering-{site}-{i}-{digit}"
                row,witness=rows[key],ink[key]
                self.assertEqual("bumper-lettering",row["role"])
                self.assertGreaterEqual(witness["fullAreaBackingFraction"],.9999)
                self.assertLess(witness["nominalHardBodyOverlapMm3"],.0001)
                self.assertEqual([59.5,164.5],[row["boundsMm"][2],row["boundsMm"][5]])
                nx,ny,_=witness["outwardNormal"]
                self.assertEqual([-ny,nx,0],witness["readerRight"])
        self.assertEqual(read(ROOT/"cad/component-palette.json")["colors"]["bumper-lettering"],"#FFFFFF")

    def test_independently_reopened_fusion_tracks_the_issued_sources(self):
        receipt = read(NUMBERED / "native-reopen.json")
        register = read(NUMBERED / "appearances.json")
        self.assertEqual(receipt["configuration"], register["configuration"])
        self.assertEqual(receipt["source"]["publicStepSha256"], register["sourceSha256"])
        register_hash = hashlib.sha256((NUMBERED / "appearances.json").read_bytes()).hexdigest()
        self.assertEqual(receipt["source"]["publicRegisterSha256"], register_hash)

        originals = [row for row in register["solids"] if not row["representation"].startswith("public-")]
        counts = Counter(row["role"] for row in originals)
        measured = receipt["originalGeometry"]
        actual = {row["role"]: (row["count"], row["rgb"]) for row in measured["roles"]}
        self.assertEqual({role: result[0] for role, result in actual.items()}, counts)
        self.assertEqual(actual["bumper-lettering"], (16, "#FFFFFF"))
        self.assertEqual(actual["bumper-red"], (2, "#D71920"))
        self.assertEqual(measured["registeredSingleBodyOccurrences"], len(originals))
        self.assertEqual(measured["visible"], len(originals))
        self.assertLess(measured["maximumPlacedBoundErrorMm"], 0.000014)
        for key in ("missing", "duplicates", "discrepancies"):
            self.assertEqual(measured[key], [])

        supplier = receipt["supplierModules"]
        self.assertEqual(supplier["referenceBodies"], 4 * 274)
        self.assertEqual(supplier["referenceBodies"], supplier["reopenedBodies"])
        self.assertEqual(supplier["referenceWheelCentersMm"], supplier["issuedWheelCentersMm"])
        self.assertEqual(set(supplier["issuedWheelCentersMm"]), {"FL", "FR", "RL", "RR"})
        self.assertEqual(supplier["changedOrMissing"], [])

