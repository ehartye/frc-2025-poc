"""Retain geometric, electrical and observational reversals in the selected fixed harness."""

import hashlib
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "fixed-camera-route"


def read(name):
    return json.loads((STUDY / name).read_bytes())


class FixedCameraRouteTests(unittest.TestCase):
    def test_installed_and_withdrawn_fit(self):
        fit, install = read("fit.json"), read("installation.json")
        self.assertEqual(fit["nativeBodies"], 991)
        self.assertGreater(fit["straightOldUpperMm"], 1059)
        self.assertGreater(1059 - fit["installedHighestMm"], 15)
        for row in fit["routeToHardware"].values():
            self.assertGreater(row["lowerMm"], 0)
            self.assertFalse(row["concerns"])
        self.assertGreater(min(fit["betweenUnrelatedRoutesMm"].values()), 0)
        for seat in install["bearingSeats"].values():
            self.assertLess(seat["gapMm"], 0.0001)
            self.assertGreater(min(seat["pressedContactMm3"]), 0)
        for groups in (install["withdrawalToOtherRoutesMm"], install["continuousCarrier"]):
            self.assertTrue(all(min(values.values()) > 0 for values in groups.values()))
        self.assertAlmostEqual(fit["fittingsAndWithdrawal"]["host-A-fixed"]["lowerMm"], 0.25)

    def test_native_sources_and_power_counterexamples(self):
        issue, fit, q = read("guides.json"), read("fit.json"), read("qualification.json")
        self.assertEqual(
            hashlib.sha256((STUDY / "harness-guides.step").read_bytes()).hexdigest(),
            issue["stepSha256"],
        )
        for p, h in fit["snapshotSources"].items():
            self.assertEqual(hashlib.sha256((ROOT / p).read_bytes()).hexdigest(), h)
        self.assertEqual(
            set(issue["volumeMm3"]), set(issue["supportBlanks"] + issue["removableTroughs"])
        )
        self.assertTrue(all(v > 0 for v in issue["volumeMm3"].values()))
        lengths = q["tipOrDrawingLengthMm"]
        self.assertGreater(lengths["fixedCustom"], lengths["fixedStockMaximum"])
        self.assertLess(lengths["roofStockInterval"][0], lengths["roof2m"][0])
        self.assertGreater(lengths["roofStockInterval"][1], lengths["roof2m"][1])
        self.assertGreater(q["minRadiusMm"]["roof-host"], 12 * 6.3)
        self.assertEqual(
            [k for k, v in q["voltageAt1p2A"].items() if v["withinWindow"]], ["selected"]
        )
        self.assertTrue(
            all(v["nativeInterferenceMm3"] < 0.001 for v in q["fitToRelievedAdapter"].values())
        )

    def test_each_optical_clock_and_receiving_limit(self):
        s, m = read("sightlines.json"), read("mass.json")
        self.assertTrue(s["roofRegistration"]["sameWitnessCoordinatesAndSigns"])
        self.assertTrue(
            all(v["bothRectanglesUnique"] and v["maxShiftPx"] == 0 for v in s["station"].values())
        )
        self.assertEqual(sum(v["frames"] for v in s["captures"].values()), 3496)
        self.assertTrue(
            all(
                v["wholePipeConeUnresolved"] == 0 and v["outsideModelTruthMm"] == 0
                for v in s["captures"].values()
            )
        )
        self.assertEqual(s["joined"]["coral-B"]["overlapMs"], [26180, 26197, 26214, 26231])
        self.assertEqual(s["joined"]["coral-B"]["wholeFrontTimeMs"], 26401)
        self.assertGreater(m["netAddedKg"][1], 0.18)
        self.assertTrue(all(not group for group in m["coverage"].values()))
        self.assertLess(m["representedMarginKg"][0], 0)
        self.assertIsNone(m["physicalMarginKg"])
        self.assertTrue(all(v["finite"] == 0 for v in s["adverseControls"].values()))
        replay = read("controller.json")
        self.assertEqual(len(replay["cases"]), 28)
        self.assertTrue(all(v["sameEventsAsReference"] for v in replay["cases"].values()))
