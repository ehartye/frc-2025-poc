import hashlib
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "grip-motion"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class GripperMotionWitness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = json.loads((STUDY / "qualification.json").read_bytes())
        cls.fit = json.loads((STUDY / "fit.json").read_bytes())

    def test_selected_geometry_and_electrical_sources(self):
        for relative, pin in self.fit["sources"].items():
            self.assertEqual(digest(ROOT / relative), pin)
        self.assertEqual(self.fit["sceneBodyCount"], 991)
        self.assertEqual(self.fit["sourceBodyCount"], 144)
        for side, fit in self.fit["grips"].items():
            self.assertEqual(fit["magnetToBoardEnvelopeMm"], 1)
            self.assertEqual(fit["standoffToBoardMm"], 0)
            self.assertEqual(fit["shaftToMagnetMm"], 0)
            self.assertEqual(fit["shaftToPulleyMm"], 0)
            name = f"grip-{side}-idler-angle-board"
            self.assertEqual(fit["inventory"][name]["category"], "sensor-reserve")
            box = fit["inventory"][name]["stowedMm"]
            self.assertEqual((box[0], box[2], box[3], box[5]), (89.5, 577, 101.5, 581))
        self.assertFalse(self.fit["verifiedShaftToPulleyAntirotation"])
        self.assertIsNone(self.fit["installedDieGapMm"])

    def test_actual_signed_clock_keeps_good_controls_and_fails_earlier_on_stall(self):
        runs = self.report["joinedSource"]
        for name, pin in self.report["watchSources"].items():
            self.assertEqual(digest(STUDY / name), pin)
        for name, pin in self.report["registeredInputs"].items():
            self.assertEqual(digest(ROOT / name), pin)
        for label, r in runs.items():
            self.assertEqual(int(r["probe_ms"]), 580)
            if label.startswith("candidate"):
                self.assertEqual(r["state_or_command_differences"], "0")
                self.assertEqual(r["fault_ms"], "-1")
                self.assertEqual(int(r["band_ms"]), r["originalFirstBandMs"])
                self.assertEqual(r["confirmed_ms"], "640")
                self.assertEqual(r["originalLastJavaMs"], 5460)
            elif label.startswith("locked"):
                self.assertEqual(r["confirmed_ms"], "-1")
                self.assertEqual(r["originalLastJavaMs"] - int(r["fault_ms"]), 700)
                self.assertEqual(float(r["last_command"]), 0)
            else:
                self.assertEqual(r["state_or_command_differences"], "0")
                self.assertEqual(r["confirmed_ms"], "640")
                self.assertEqual(r["originalLastJavaMs"], int(r["fault_ms"]))
        self.assertEqual(len(runs), 8)

    def test_grid_and_continuous_bounds(self):
        for state, kind in (("PROBE", "healthy"), ("FAULT", "locked")):
            row = self.report["onsetGrid"][kind]
            self.assertEqual(row["outcomes"], {state: 960})
            self.assertEqual(row["millisecondsAfterProbe"], [60, 60])
        b = self.report["guaranteedBounds"]
        uncertainty = 2 * (
            b["angleErrorDegEach"] / 360 * 60 + 0.5 * 60 / 4095 + 0.5 * 60 / 1650
        )
        self.assertAlmostEqual(uncertainty, b["quantized4095SensorAnd1650AnalogPairMm"])
        self.assertLess(uncertainty, b["thresholdMm"])
        self.assertGreater(b["minHealthyStationaryStartReadMm"], b["thresholdMm"])
        late = self.report["laterPersistentStall"]
        self.assertEqual(late["cases"], 480)
        self.assertEqual(late["sampledDelayMs"], [39, 61])
        self.assertLess(max(late["sampledDelayMs"]), late["conservativeContinuousMs"])
        self.assertEqual(len(self.report["sweepSha256"]), 64)

    def test_missing_data_motion_and_counterfeit_custody(self):
        cases = self.report["digitalControls"]
        self.assertEqual(len(cases), 14)
        empty = cases["empty with good port and angle replay"]
        self.assertEqual((empty["outcome"], empty["band_ms"]), ("IN_BAND", "1340"))
        self.assertEqual(
            cases["replayed angle with jammed ports"]["reason"], "probe-timeout"
        )
        for name, r in cases.items():
            if name != "empty with good port and angle replay":
                self.assertEqual(r["outcome"], "FAULT", name)
        for k in ("disabled", "lost dock", "stale port"):
            self.assertEqual(cases[k]["reason"], "unqualified-or-stale-frame")
        for k in ("single locked idler", "wrong right polarity"):
            self.assertEqual(cases[k]["fault_ms"], "640")
        self.assertIsNone(self.report["installedCalibration"])
        self.assertFalse(self.report["chassisAuthority"])
