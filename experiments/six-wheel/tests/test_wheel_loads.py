"""Numerical and provenance controls for the straight-guide wheel-load study."""

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "wheel-loads"


def read(path):
    return json.loads(path.read_bytes())


def procedure():
    spec = importlib.util.spec_from_file_location(
        "straight_guide_wheel_contact", STUDY / "contact.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class WheelLoading(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.proc = procedure()
        cls.wheels = read(ROOT / "selected-drive/wheels.json")["wheels"]
        cls.contact = read(STUDY / "contact.json")

    def solve(self, x=0, y=0, ratio=1, gap=0):
        stiffness = {
            w: 100 * (ratio if w.startswith("center") else 1) for w in self.wheels
        }
        gaps = {w: gap if w.startswith("center") else 0 for w in self.wheels}
        return self.proc.contacts(
            self.wheels, 600 / self.proc.G, (x, y), stiffness, gaps
        )

    def test_solver_matches_force_and_moments(self):
        case = self.solve(x=25, y=-30, ratio=0.8, gap=0.2)
        forces = case["wheelN"]
        self.assertTrue(all(v > 0 for v in forces.values()))
        self.assertAlmostEqual(sum(forces.values()), 600)
        for axis, expected in enumerate((25, -30)):
            self.assertAlmostEqual(
                sum(
                    f * self.wheels[w]["contactCenterMm"][axis]
                    for w, f in forces.items()
                )
                / 600,
                expected,
            )
        self.assertAlmostEqual(case["lightSideFraction"], 0.5 - 30 / 580)
        self.assertAlmostEqual(self.solve(ratio=0.8, gap=0.1875)["outerFraction"], 0.75)

    def test_unilateral_controls(self):
        ends = self.solve(gap=2)
        self.assertEqual(set(ends["zeroLoad"]), {"center-L", "center-R"})
        self.assertAlmostEqual(ends["outerFraction"], 1)
        center = self.solve(gap=-4)
        self.assertAlmostEqual(center["outerFraction"], 0)
        with self.assertRaises(ValueError):
            self.solve(x=300)
        with self.assertRaises(ValueError):
            self.solve(ratio=-1)

    def test_configuration_and_mass_phases(self):
        cases = self.proc.simulate()
        self.assertEqual(json.loads(json.dumps(cases)), self.contact)
        mass = cases["mass"]["noCutCreditKg"]
        self.assertAlmostEqual(mass["R103Kg"][1], 51.793931, 6)
        self.assertAlmostEqual(mass["emptyFieldKg"][1], 65.463931, 6)
        self.assertAlmostEqual(mass["loadedFieldKg"][1], 66.280397, 6)
        self.assertAlmostEqual(
            mass["oldReferenceEmptyMarginKg"], mass["oldReferenceLoadedMarginKg"]
        )
        self.assertGreater(mass["R408MarginUpperKg"], 1.94)
        self.assertIsNone(cases["physicalWheelLoadsN"])

    def test_located_mass_and_residual(self):
        m = read(STUDY / "moments.json")
        rows = m["fixedParcelRows"]
        self.assertEqual(len(rows), 48)
        self.assertAlmostEqual(
            sum(a["kg"][0] for a in rows.values()) + m["unlocatedUpperRemainderKg"],
            m["fieldMassUpperKg"],
        )
        self.assertEqual(m["deck"]["maxAddedAndMissingMm3Each"], 40000)
        self.assertEqual(
            m["deck"]["stepSha256"],
            hashlib.sha256(
                (ROOT / "mast-attachment/slotted-ribbed-deck.step").read_bytes()
            ).hexdigest(),
        )
        self.assertIsNone(m["actualSelectedCgMm"])
        budget = read(STUDY / "balance.json")["stationaryAndOpening"]
        self.assertEqual(
            budget["0"]["permittedRemainderYmm"],
            m["permittedOtherCentroidYmmForStatic44pctAtUpperMass"],
        )
        for y in ("-20", "0", "20"):
            self.assertGreater(
                budget["450"]["placementControls"][y]["minimumSideFraction"], 0.44
            )
        for y in ("-40", "40"):
            self.assertLess(
                budget["450"]["placementControls"][y]["minimumSideFraction"], 0.44
            )

    def test_signed_clocks_have_distinct_scope_and_failure_modes(self):
        old = read(ROOT / "conditioned-match/slow-power.json")
        issued = read(STUDY / "power.json")
        revised = read(STUDY / "opening.json")
        for name, digest in issued["sources"].items():
            self.assertEqual(
                hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), digest
            )
        self.assertIsNone(issued["fullNewMatchScoring"])
        self.assertEqual(revised["cargoKg"], 0)
        for sign in ("leftSmall", "rightSmall"):
            for field in (
                "peakMotorPhaseA",
                "minTerminalV",
                "bodyKg",
                "onePipeKg",
                "peakFrictionRequired",
            ):
                self.assertAlmostEqual(
                    issued["signed"][sign]["cases"]["issued"][field],
                    old[sign]["screened"][field],
                    7,
                )
            source = revised["signed"][sign]
            self.assertEqual(source["endS"], 21.1)
            for key, last, first in (("0.027", 0.925, 0.95), ("0.032", 0.775, 0.8)):
                grid = source["grid"][key]
                self.assertEqual(
                    (grid["lastPassingTestedShare"], grid["firstFailingTestedShare"]),
                    (last, first),
                )
                self.assertEqual(len(grid["trials"]), 13)
                fail = next(r for r in grid["trials"] if r["ends"] == first)
                if key == "0.027":
                    self.assertGreater(fail["peakMotorPhaseA"], 38)
                    self.assertGreater(fail["minTerminalV"], 9.5)
                else:
                    self.assertLess(fail["peakMotorPhaseA"], 38)
                    self.assertLess(fail["minTerminalV"], 9.5)
                self.assertLess(fail["firstReferenceInhibitWallS"], 21.1)
