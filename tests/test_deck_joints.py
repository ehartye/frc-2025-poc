import hashlib
import json
from pathlib import Path
import unittest
from xml.etree import ElementTree

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"deck-joints"


def read(name):
    return json.loads((HERE/name).read_bytes())


class DeckJoints(unittest.TestCase):
    def test_six_positive_joint_paths_and_full_tool_strokes(self):
        fit=read("fit.json")
        self.assertEqual(fit["stepSha256"],hashlib.sha256((HERE/"deck-through-ties.step").read_bytes()).hexdigest())
        self.assertEqual(len(fit["volumeMm3"]),21)
        self.assertEqual(set(map(tuple,fit["axesMm"])),{(-339,-225),(-339,72),(-339,225),(339,-225),(339,-72),(339,225)})
        self.assertEqual(fit["counts"]["originalOpticalAndReceiver"],991)
        self.assertGreater(fit["counts"]["movingSweepBoxes"],300)
        for key,sha in fit["sources"].items():
            source=list(ROOT.glob("*/"+key))
            self.assertEqual(len(source),1,key)
            self.assertEqual(hashlib.sha256(source[0].read_bytes()).hexdigest(),sha,key)
        for station in fit["stations"].values():
            self.assertEqual(len(station["parts"]),3)
            self.assertEqual(set(station["service"]),{"socketEntry","topSocket","handle","anchorWithdrawal","bottomHoldingFork"})
            for stage in ("parts","service"):
                self.assertFalse(any(row["overlapMm3"] for row in station[stage].values()))
            witnesses=station["pressedCommonMm3"]
            self.assertGreater(witnesses["footToCrossrail"],75)
            self.assertGreater(min(witnesses.values()),1)
            self.assertEqual(witnesses["keyAt-2.0Deg"],witnesses["keyAt2.0Deg"])
        self.assertGreater(fit["rejectedRearMinus72NutRacewayMm3"],50)
        self.assertEqual(set(fit["misplacedSleeve"]["overlapMm3"]),{"bumper-screw-front-end-L","bumper-sleeve-front-end-L"})
        ElementTree.parse(HERE/"shop.svg")

    def test_mass_removal_and_conditional_budget(self):
        cases=read("comparison.json")
        fit=read("fit.json")
        selected=cases["conditionalMatchedArticleDeltaKg"]
        mass=selected["hardware"]
        self.assertEqual(mass["stepSha256"],fit["stepSha256"])
        self.assertEqual(mass["removedStockMassCreditKg"],0)
        self.assertIsNone(mass["weighedFinishedKitKg"])
        self.assertEqual(set(mass["classKg"]),{"anchor","washer","locknut"})
        self.assertAlmostEqual(sum(r[1] for r in mass["classKg"].values()),mass["estimatedKg"][1])
        self.assertTrue(.110<mass["estimatedKg"][1]<.115)
        self.assertTrue(0<selected["representedMarginKg"][0]<.010)
        for alt in ("harness","restrictedNoMotorCreditKg"):
            self.assertLess(cases[alt]["representedMarginKg"][0],-.330)
        for case in cases.values():
            self.assertIsNone(case["physicalMarginKg"])
            self.assertEqual(case["choices"][-1],"deck-through-ties")
        steel=read("steel-control.json")
        self.assertEqual(len(steel["volumeMm3"]),27)
        self.assertTrue(all(not row["overlapMm3"] for s in steel["stations"].values()
                            for stage in ("parts","service") for row in s[stage].values()))

    def test_load_assumptions_and_adverse_case(self):
        data=read("load.json")
        a,b=data["integralAllSix"],data["integralMissingOne"]
        self.assertTrue(260<a["stressAtTrialNotchFactor1_6MPa"]<270)
        self.assertGreater(b["stressAtTrialNotchFactor1_6MPa"],a["stressAtTrialNotchFactor1_6MPa"])
        self.assertGreater(data["oneTieUnavailable"]["maximumTieTensionN"],data["fullSection"]["jointSeparationNAt850NModelPreload"])
        for side in data["contact"].values():
            self.assertLess(side["normalMaxN"],0)
            self.assertTrue(abs(side["lineAtDeckMm"][1])<45)
        self.assertIsNone(a["physicalYieldFatigueAndSeparation"])
