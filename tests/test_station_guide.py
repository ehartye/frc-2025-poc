import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"station-guide"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class StationGuideTests(unittest.TestCase):
    def test_face_material_and_observer_endpoints(self):
        long=read("fit.json")
        self.assertEqual({r["robot"] for r in long["interferences"]},{"roof-top-tube"})
        for r in long["interferences"]:
            self.assertGreater(r["largest"][1],83.)
        for name,step in (("fit-short.json","short-face-profile.step"),("fit-narrow.json","narrow-face.step")):
            record=read(name)
            self.assertEqual(record["stepSha256"],hashlib.sha256((STUDY/step).read_bytes()).hexdigest())
            self.assertFalse(record["interferences"])
            self.assertFalse(record["unresolvedVariableGeometry"])
        self.assertEqual(read("optics.json")["trials"]["leftSmall"]["status"],"stale registered stereo")
        data=read("optics-narrow.json")
        self.assertEqual(data["profileStepSha256"],read("fit-narrow.json")["stepSha256"])
        self.assertEqual(data["registeredReceiverDepthSha256"],read("background.json")["backgroundSha256"]["stowed-robot-fixed"])
        for r in data["trials"].values():
            self.assertEqual(r["status"],"observed stationary")
            self.assertLess(r["wallS"],21.1)
            self.assertGreater(min(r["fieldMinimumMm"].values()),11.7)

    def test_evaluated_incoming_poses_drive_actual_java_and_stop_controls(self):
        ports=read("seat-narrow.json");self.assertEqual(len(ports),24)
        paths=read("optics-narrow.json")["trials"]
        for r in ports:
            case="nominal" if r["case"]=="shiftedHumanSource" else r["case"]
            n,y,h=paths[case]["finalStationMmDeg"]
            self.assertEqual(r["attainedRobotMmDeg"],[-n,-y,h])
            self.assertEqual(r["lastState"],"IN_BAND")
            self.assertEqual(r["lastTickMs"],5460)
            self.assertIsNone(r["stop"])
            self.assertLess(r["peakLocalDeflectionMm"],3.)
        for case in ("leftSmall","rightSmall"):
            review=read("review.json")["cases"][case]
            self.assertEqual((review["status"],review["acceptance"]),("unknown","blocked"))
            self.assertGreater(next(r for r in review["runs"] if r["run"]=="dt-4")["firstStop"][2],1.4)
        self.assertEqual(read("review.json")["cases"]["nominal"]["status"],"not-converged")

    def test_shared_mass_is_charged_once_for_each_alternative(self):
        mass=read("mass.json")
        self.assertIsNone(mass["mountingKg"])
        self.assertIsNone(mass["measuredMassKg"])
        self.assertAlmostEqual(mass["backingKgAt2700KgM3"],.03888)
        row=mass["hypotheticalLinerDensityKgM3"]["500"]
        self.assertAlmostEqual(row["guideFaceKg"],.06588)
        options=row["alternatives"]
        self.assertGreater(options["guideOnly"]["representedMarginKg"],0)
        for name in ("guideAndCovers","guideAndHarness","guideHarnessAndCovers"):
            self.assertLess(options[name]["representedMarginKg"],0)
        self.assertAlmostEqual(options["guideHarnessAndCovers"]["representedUpperKgBeforeAttachments"],
                               mass["baseRobotUpperKg"]+row["guideFaceKg"]+
                               mass["optionalHarnessUpperKg"]+mass["optionalCoverUpperKg"])
