import copy
import hashlib
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "guide-setting"
spec = importlib.util.spec_from_file_location("guide_setting_survey", HERE / "survey.py")
survey = importlib.util.module_from_spec(spec)
spec.loader.exec_module(survey)


class GuideSettingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = survey.load_layout()

    def test_tool_and_probe_paths(self):
        fit = survey.read(HERE / "fit.json")
        self.assertEqual(len(fit["changedInstalled"]), 6)
        self.assertEqual(len(fit["pitOnly"]), 4)
        self.assertEqual(set(fit["volumesMm3"]), set(fit["changedInstalled"] + fit["pitOnly"]))
        self.assertEqual(fit["stepSha256"], hashlib.sha256((HERE / "setting.step").read_bytes()).hexdigest())
        self.assertEqual(len(fit["tools"]), 16)
        for path in fit["tools"].values():
            self.assertFalse(path["materialOverlapMm3"] or path["cableIntrusionMm"])
        for name, joint in fit["interfaces"].items():
            self.assertGreater(joint["outboardShiftOverlapMm3"], 3)
            self.assertEqual(joint["verticalShiftOverlapMm3"] > 1, "-687-" in name)
            for socket in joint["frontSockets"]:
                self.assertGreaterEqual(socket["workingClearanceMm"], 1.25 - 1e-6)
                self.assertGreater(socket["originalLocationOverlapMm3"], 16)
        contacts = list(self.probe["mastDatums"].values())
        self.assertEqual(len(contacts), 14)
        for row in self.probe["stations"].values():
            contacts.extend(row["sides"].values())
        self.assertEqual(len(contacts), 34)
        for p in contacts:
            self.assertAlmostEqual(p["intentionalPenetrationMm3"], 0.04608, delta=.001)
        blocked = self.probe["rejectedRightMastApproaches"]
        for height, kind in (("845","data-tail-10"), ("820","power-tail-11")):
            self.assertGreater(blocked[height]["materialOverlapMm3"]["wire/" + kind], 37)

    def test_assembly_cases(self):
        data = survey.read(HERE / "accounting.json")
        self.assertEqual(data["chipCreditKg"], 0.)
        self.assertEqual(len(data["cases"]), 2)
        for policies in data["cases"].values():
            self.assertEqual(len(policies), 2)
            for case in policies.values():
                self.assertEqual(case["selectedCount"], 1189)
                self.assertIsNone(case["physicalMarginKg"])
            self.assertGreater(policies["completeGroupDeltaKg"]["marginKg"][0], .08)
            self.assertLess(policies["unknownGroupDeltaKg"]["marginKg"][0], 0.)

    def test_three_examples_and_renderer(self):
        reports = {name: survey.assess(survey.example(self.probe, name), self.probe)
                   for name in ("nominal", "sideways", "both-outboard")}
        bundled = (HERE / "examples.js").read_text(encoding="utf8").strip()
        published = json.loads(bundled.removeprefix("const cases = ").removesuffix(";"))
        for name, report in reports.items():
            expected = .220 if name == "nominal" else -.095
            self.assertAlmostEqual(report["minimumLowerOverlapMm"], expected, places=7)
            self.assertAlmostEqual(published[name]["minimumLowerOverlapMm"], expected, places=7)
            self.assertEqual(published[name]["settingStepSha256"], report["settingStepSha256"])
            self.assertIsNone(report["scoringAccepted"])
        shifts = survey.compare(reports["nominal"], reports["sideways"])
        for row in shifts["stationChanges"].values():
            self.assertAlmostEqual(row["widthChangeMm"], 0.)
            self.assertAlmostEqual(row["centerShiftMm"], .315)
            self.assertAlmostEqual(row["faceChangeMm"]["L"][0], .215)
        with self.assertRaises(ValueError):
            survey.compare(reports["nominal"], reports["nominal"])

    def test_uncertainty_and_reference(self):
        base = survey.example(self.probe, "nominal")
        missing = dict(base, frameProjectionErrorMm=None)
        self.assertIsNone(survey.assess(missing, self.probe)["minimumLowerOverlapMm"])
        with self.assertRaises(ValueError):
            survey.assess(dict(base, guideStepSha256="wrong"), self.probe)
        for failure in ("rank", "side", "residual", "station"):
            trial = copy.deepcopy(base)
            points = trial["mastPointsMm"]
            if failure == "rank":
                for key in points:
                    if "-fore-" in key:
                        points[key] = [1, 2, 3]
            elif failure == "side":
                trial["mastPointsMm"] = {k: points[("R" if k[0]=="L" else "L")+k[1:]] for k in points}
            elif failure == "residual":
                points["L-fore-750"][0] += 1
            else:
                trial["linerPointsMm"]["628/front"]["R"][0] += 1
            if failure in ("rank", "side"):
                with self.assertRaises(ValueError):
                    survey.assess(trial, self.probe)
            else:
                report = survey.assess(trial, self.probe)
                self.assertTrue(report["unresolved"])
                self.assertIsNone(report["bothEndsContactAtAllLowerStations"])
