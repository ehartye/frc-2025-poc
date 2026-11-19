"""Controls for parcel accounting, attainable load-line failures and unilateral contact witnesses."""

import importlib.util
import itertools
import json
import math
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "lateral-balance"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def read(p):
    return json.loads(p.read_bytes())


class LateralBalance(unittest.TestCase):
    def test_mass_inventory_and_physical_limit(self):
        p = read(STUDY / "parcels.json")
        k = read(STUDY / "kits.json")
        assigned = [
            b
            for r in list(p["parcels"].values()) + list(k["parcels"].values())
            for b in r.get("bodies", [r.get("body")])
        ]
        self.assertEqual(len(assigned), len(set(assigned)))
        self.assertEqual((p["registeredBodies"], k["installed"]), (991, 1189))
        self.assertNotIn("lift", p["parcels"])
        self.assertNotIn("guide-L-687-setting-pin", k["parcels"])
        self.assertIn("match-drilled/guide-L-machined-mast", k["parcels"])
        d = read(STUDY / "transfer.json")
        self.assertEqual(d["parcelCount"], 441)
        self.assertEqual(
            d["fieldMassKg"],
            read(ROOT / "wheel-loads/contact.json")["mass"]["noCutCreditKg"][
                "emptyFieldKg"
            ],
        )
        self.assertIsNone(d["actualWheelLoadN"])

    def test_ratio_extrema_match_brute_force(self):
        t = load("isolated_lateral_transfer", STUDY / "transfer.py")
        p = {
            "a": {"kg": [1, 2], "positionMm": [-2, -4, 7, 1, 2, 8]},
            "b": {"kg": [2, 3], "positionMm": [1, 3, 4, 2, 5, 7]},
        }
        m = (8, 15)
        r = [-4, -3, 0, 3, 3, 10]
        numeric = t.cg_box(p, m, r)
        for i, coord in enumerate(numeric):
            allvals = [
                (
                    a * p["a"]["positionMm"][i]
                    + b * p["b"]["positionMm"][i]
                    + (total - a - b) * r[i]
                )
                / total
                for a, b, total in itertools.product((1, 2), (2, 3), m)
            ]
            self.assertAlmostEqual(coord, (min if i < 3 else max)(allvals))

    def test_slab_and_clock_controls(self):
        t = load("isolated_lateral_transfer", STUDY / "transfer.py")
        p = t.components()
        b = "NP18-12 battery"
        d = read(STUDY / "transfer.json")
        treated = t.pack_fill(p, 12)[b]
        self.assertAlmostEqual(
            treated["positionMm"][1] - p[b]["positionMm"][1], 21.853478715, 7
        )
        self.assertAlmostEqual(treated["positionMm"][1] + treated["positionMm"][4], -29)
        original = d["cases"]["centered"]
        witness = original["commonMassWitness"]
        self.assertLess(original["minimumSideFraction"], witness["minimumSideFraction"])
        self.assertLess(witness["minimumSideFraction"], 0.44)
        self.assertAlmostEqual(
            witness["knownMassKg"] + witness["residualMassKg"], witness["massKg"]
        )
        self.assertTrue(
            0.442 < d["cases"]["paired-and-packed"]["minimumSideFraction"] < 0.443
        )
        self.assertLess(
            d["cases"]["paired-and-packed-adverse"]["commonMassWitness"][
                "minimumSideFraction"
            ],
            0.44,
        )
        a, b = d["limitingResidualCentroidYmmAtSpecifiedXandZ"]
        self.assertTrue(a < -15 and b > 18)
        self.assertEqual(d["cases"]["gross"]["twoSideSupportBound"], False)
        self.assertAlmostEqual(
            d["allLegacyStockAtNamedSites"]["explicitlyPartitionedKg"],
            read(ROOT / "winchless-bill/stock.json")["stockAndFastenerUpperKg"],
        )

    def test_contact_optimization_reconstructs_on_independent_solver(self):
        engine = load("independent_wheel_contact", ROOT / "wheel-loads/contact.py")
        wheels = read(ROOT / "selected-drive/wheels.json")["wheels"]
        cases = read(STUDY / "order.json")["cases"]
        for name, r in cases.items():
            witness = r["witness"]
            ids = r["wheelOrder"]
            sol = engine.contacts(
                wheels,
                6 / engine.G,
                witness["loadLineMm"],
                dict(zip(ids, witness["stiffnessRatios"])),
                dict(zip(ids, witness["gaps"])),
            )
            self.assertAlmostEqual(sol["outerFraction"], r["maximumEndFraction"])
            for key, v in zip(ids, witness["normalFractions"]):
                self.assertAlmostEqual(sol["wheelN"][key] / 6, v)
            self.assertLess(r["minimumActiveSetSize"], 6)
        self.assertTrue(
            cases["centers-largest"]["maximumEndFraction"]
            < 0.75
            < cases["unsorted"]["maximumEndFraction"]
        )
        self.assertGreater(cases["wide-rates"]["maximumEndFraction"], 0.75)
        self.assertEqual(cases["wide-rates"]["minimumActiveSetSize"], 4)

    def test_joint_assignment_and_incompatible_set(self):
        w = load("wheel_assignment", STUDY / "assign.py")
        trial = w.arrange(w.EXAMPLE)
        self.assertEqual(
            (trial["assignment"]["center-L"], trial["assignment"]["center-R"]),
            ("E", "F"),
        )
        self.assertAlmostEqual(trial["maxAxleMismatchKg"], 0.007)
        bad = [dict(r) for r in w.EXAMPLE]
        bad[1]["radiusErrorMm"], bad[5]["radiusErrorMm"] = (
            bad[5]["radiusErrorMm"],
            bad[1]["radiusErrorMm"],
        )
        self.assertFalse(w.arrange(bad)["assignable"])
        with self.assertRaises(ValueError):
            w.arrange(bad[:-1] + [bad[0]])
