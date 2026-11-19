"""Portable geometry-contract and archived native-result checks."""
import hashlib
import json
import math
from pathlib import Path
import unittest

HERE=Path(__file__).resolve().parent


def read(name):
    return json.loads((HERE/name).read_bytes())


def sha(name):
    return hashlib.sha256((HERE/name).read_bytes()).hexdigest()


class Packaging(unittest.TestCase):
    def test_module_datums_and_axes(self):
        c=read("module.json")
        self.assertEqual(c["order"],["FL","FR","RL","RR"])
        self.assertEqual(c["driveMotorTurnsPerWheelTurn"],45*22/(12*15))
        self.assertAlmostEqual(c["azimuthMotorTurnsPerModuleTurn"],9424/203)
        receipt=read("student-archive.json")
        self.assertEqual(receipt["frame"]["stepSha256"],sha("swerve-frame.step"))
        for value in receipt["wheelChecks"]:
            corner=value["path"].split(" ")[0]
            s=c["modules"][corner]
            e=math.dist(value["centerMm"],[1000*s["xM"],1000*s["yM"],38.1])
            self.assertLess(e,.004)
            self.assertEqual(receipt["corners"][corner]["nested"],74)
            self.assertGreaterEqual(receipt["corners"][corner]["bodyCount"],72)

    def test_complete_continuous_envelopes(self):
        c=read("envelopes.json")
        self.assertEqual(c["frameStepSha256"],sha("swerve-frame.step"))
        for k in ("FL","FR","RL","RR"):
            rows=[b for name,b in c["cornerBoundsMm"].items() if name.startswith(k+"/")]
            self.assertEqual(len(rows),75)
            for x,y,z,x1,y1,z1 in rows:
                self.assertLessEqual(max(abs(x),abs(x1)),330)
                self.assertLessEqual(max(abs(y),abs(y1)),325)
                self.assertGreaterEqual(z,-.001)
                self.assertLess(z1,172)
        self.assertLessEqual(max(c["movingRadiusMm"].values()),47.4)

    def test_student_feature_history(self):
        r=read("saddle-fusion.json")
        self.assertEqual(sha("tab-saddle.f3d"),r["sha256"])
        self.assertEqual(r["nativeExtrudes"],4)
        self.assertEqual(r["nativeSketches"],4)
        self.assertAlmostEqual(r["volumeMm3"],r["correspondingOcctFrameMm3"],delta=.01)

    def test_baseline_hits_and_scope(self):
        r=read("fit.json")
        self.assertEqual(r["kitSha256"],sha("swerve-frame.step"))
        self.assertEqual(len(r["initialMaterialHitsMm3"]),26)
        self.assertEqual(len(r["installedMaterialHitsMm3"]),2)
        self.assertAlmostEqual(sum(r["installedMaterialHitsMm3"].values()),579.1604,places=3)
        self.assertIsNone(read("mount-budget.json")["mass"]["physicalScaleKg"])
        self.assertEqual(len(read("retirement.json")["relocatedInnerEarSet"]),40)

    def test_rear_host_uses_native_material_and_continuous_service_bounds(self):
        rear=read("rear-electronics.json")
        self.assertEqual(rear["stepSha256"],sha("rear-electronics.step"))
        self.assertEqual(rear["materialHitsWithRearMotorsMm3"],{})
        self.assertLessEqual(rear["noChipUpperKg"],.158)
        self.assertGreater(min(rear["existingDeckBearingPressMm3AtPointTwo"].values()),9.5)
        self.assertEqual(len(rear["boundsMm"]),30)
        self.assertGreater(rear["originalHardwareConflicts"]["RR"]["hardwareNative"]["host-post-19--245"][0],77)
        self.assertGreater(rear["originalHardwareConflicts"]["RR"]["hardwareNative"]["base-guide-rib-1"][0],257)
        service=read("service.json")
        self.assertEqual([r["corner"] for r in service],["FL","FR","RL","RR"])
        for r in service:
            self.assertEqual(r["positiveConservativeBoxesMm3"],{})
            self.assertEqual(r["screenedBodyBoxes"],85)
            self.assertEqual(len(r["matchingTemporaryDeckAnchorRemoval"]),1)
            self.assertEqual(r["rearCorrection"],rear["stepSha256"])


if __name__=="__main__":
    unittest.main()
