"""Reviewable receipt checks without loading supplier models or rendering new images."""

import hashlib
import json
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def read(name):
    return json.loads((HERE / name).read_bytes())


class SelectedOpticsTests(unittest.TestCase):
    def test_supplier_witness_and_corrected_mount(self):
        mount = read("mount.json")
        self.assertEqual(
            mount["stepSha256"],
            hashlib.sha256((HERE / "fixed-datum.step").read_bytes()).hexdigest(),
        )
        self.assertEqual(len(mount["reopenedParts"]), 5)
        self.assertEqual(mount["interferenceAbove001Mm3"], {})
        self.assertIsNone(mount["measuredOpticalPupils"])
        pos = mount["registrations"]
        self.assertGreater(pos["fixed-usb-down"]["usbCorridorCradleOverlapMm3"], 90)
        self.assertGreater(pos["fixed-usb-up"]["cradleCaseOverlapMm3"], 28)
        self.assertEqual(pos["fixed-datum-up"]["cradleCaseOverlapMm3"], 0)
        self.assertEqual(pos["roof"]["assumedRectifiedOriginMm"], [48.539, 0.0, 846.28])
        iface = read("interfaces.json")
        self.assertGreater(min(v["cornerMarginMm"] for v in iface["aperture"]), 0.15)
        self.assertGreater(
            iface["fit"]["fixedPlugGaugesBoundsMm"][
                "power-with-transferred-straight-tail"
            ][5],
            1059,
        )

    def test_issued_hardware_and_adverse_images(self):
        out = read("observations.json")
        self.assertEqual(
            out["sources"]["selected-match/receiver-clearance-kit.step"],
            hashlib.sha256(
                (ROOT / "selected-match/receiver-clearance-kit.step").read_bytes()
            ).hexdigest(),
        )
        self.assertEqual(out["selectedScene"]["selectedPartsIncludingWire"], 985)
        self.assertEqual(
            out["pairedHandoff"]["coral-B"]["joinedCapturesMs"],
            [26180, 26197, 26214, 26231],
        )
        self.assertGreater(out["adverse"]["staleRoofTruthOutsideAt26231Mm"], 0)
        self.assertFalse(out["adverse"]["lostEyeSuppliedBounds"])
        self.assertFalse(out["adverse"]["swappedEyesSuppliedBounds"])
        marked = out["adverse"]["markedRegistration"]
        self.assertTrue(marked["sameWitnessCoordinatesAndSigns"])
        self.assertTrue(all(n >= 300 for n in marked["witnessSamples"]))
        self.assertEqual(
            out["adverse"]["unmarkedRegistration"]["witnessSamples"], [0] * 4
        )
        self.assertTrue(
            all(v["maxTruthOutsideMm"] == 0 for v in out["completeViews"].values())
        )
        field = read("roof-field.json")
        endpoints = [
            pose for phases in field["cases"].values() for pose in phases.values()
        ]
        self.assertEqual(sum(len(pose["frames"]) for pose in endpoints), 55)
        self.assertTrue(
            all(pose["excludedRobotBackingPixels"] == [0, 0] for pose in endpoints)
        )
        controller = read("controller.json")
        self.assertEqual(
            controller["roofFieldCheckSha256"],
            hashlib.sha256((HERE / "roof-field.json").read_bytes()).hexdigest(),
        )
        self.assertEqual(len(controller["cases"]), 28)
        self.assertTrue(controller["sameEventsAsReference"])
        mass = read("mass.json")
        self.assertEqual(len(mass["addedInstalled"]), 4)
        self.assertFalse(any(mass["coverage"].values()))
        self.assertIsNone(mass["physicalMarginKg"])

    def test_survey_rejects_stale_origin(self):
        sys.path.insert(0, str(HERE))
        import survey
        import numpy as np

        for head in ("roof", "fixed"):
            model = survey.metrology.example()
            model.update(head=head, stroke_mm=370)
            for obs in model["eyes"].values():
                for plane, points in zip(obs["planes"], survey.targets(head, 370)):
                    plane["robot_dots_xyz_mm"] = points
            result = survey.measure(head, 370, model)
            self.assertTrue(result["within_project_pose_screen"])
            self.assertFalse(result["physical_measurement_eligible"])
            basis, _ = survey.fixture(head, 370)
            for obs in model["eyes"].values():
                for plane in obs["planes"]:
                    plane["robot_dots_xyz_mm"] = (
                        np.array(plane["robot_dots_xyz_mm"]) + 2 * basis[:, 0]
                    ).tolist()
            self.assertFalse(
                survey.measure(head, 370, model)["within_project_pose_screen"]
            )


if __name__ == "__main__":
    unittest.main()
