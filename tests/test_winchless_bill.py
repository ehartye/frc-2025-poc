import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
BILL = ROOT / "winchless-bill"


def load(name):
    return json.loads((BILL / name).read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def module(name):
    m = importlib.util.spec_from_file_location("bill_" + name, BILL / (name + ".py"))
    s = importlib.util.module_from_spec(m)
    m.loader.exec_module(s)
    return s


class WinchlessMassBill(unittest.TestCase):
    def test_rebuilt_same_row_across_all_archived_sources(self):
        source = load("original.json")
        self.assertEqual(source["sourceRows"], 331)
        for p, h in source["sourceSha256"].items():
            self.assertEqual(sha(ROOT / p), h)
        for i, v in enumerate(source["originalGroupKg"]):
            self.assertAlmostEqual(
                source["summedBaselineKg"][i] - v,
                source["baselineWithoutPrimaryLiftKg"][i],
            )
        self.assertEqual(source["keptExamplesKg"]["unmodeled-allocation"], [2, 5])

    def test_native_joint_limits_and_counterexample(self):
        stock = load("stock.json")
        for p, h in stock["sources"].items():
            self.assertEqual(sha(ROOT / p), h)
        self.assertEqual(len(stock["originalRoutingIds"]), 34)
        self.assertEqual(len(stock["retainedStock"]), 7)
        self.assertEqual(len(stock["machinedParentIds"]), 4)
        self.assertEqual(
            sum(
                r["materialSharedWithParentMm3"]
                for r in stock["retainedStock"].values()
            ),
            0,
        )
        self.assertEqual(
            stock["throughSet"]["screw"]["sourceOutsideTwoCylinderGaugeMm3"], 0
        )
        self.assertAlmostEqual(
            stock["stockAndFastenerUpperKg"],
            sum(r["finishedStockUpperKg"] for r in stock["retainedStock"].values())
            + 14 * sum(r["upperKg"] for r in stock["throughSet"].values()),
        )
        self.assertGreater(stock["opaqueSportSolidBoxSensitivityKg"], 3)
        self.assertIsNone(stock["installedStockMeasuredKg"])

    def test_conservative_configurations(self):
        for source, expected in (
            ("straight-guide.json", 0.36919150),
            ("returned-guide.json", 0.338957),
        ):
            report = load(source)
            for shelf, r in report.items():
                self.assertEqual(r["instances"], 1189)
                self.assertEqual(r["originalGroupArticles"], 24)
                self.assertIn("spark-lift", r["kept"])
                self.assertIn("observer-camera-proxy", r["kept"])
                self.assertEqual(len(r["rejectedCompositionControls"]), 5)
                self.assertIn(
                    "zero-credit",
                    r["rejectedCompositionControls"]["prior credit: winchless"],
                )
                self.assertEqual(
                    r["fixedGyro"]["cameraSource"],
                    sha(ROOT / "station-landmark/README.md"),
                )
                self.assertFalse(r["fixedGyro"]["kickstarterProvidesGyro"])
                self.assertEqual(
                    r["fixedGyro"]["straightApproachRateModel"],
                    sha(ROOT / "reef-inertial-observer/rate.py"),
                )
                self.assertLess(r["zeroCreditMarginKg"], 0)
                out = r["outcomes"]
                self.assertLess(out["opaqueReserveAlsoPresentKg"]["upperMarginKg"], 0)
                self.assertGreater(
                    out["allStocksAndFourteenSetsKg"]["upperMarginKg"], 0
                )
                self.assertAlmostEqual(
                    out["issuedDrawingKg"]["upperMarginKg"] - r["zeroCreditMarginKg"],
                    3.9,
                )
                self.assertIsNone(r["unboundedResidualMarginKg"])
                self.assertIsNone(r["physicalScaleMassKg"])
            self.assertAlmostEqual(
                report["noCutCreditKg"]["outcomes"]["allStocksAndFourteenSetsKg"][
                    "upperMarginKg"
                ],
                expected,
                places=5,
            )

    def test_model_schedule_requires_complete_finite_quantities(self):
        screen = module("screen")
        stock = load("stock.json")
        start = screen.blank(stock)
        r = load("straight-guide.json")["noCutCreditKg"]

        def evaluate(data):
            return screen.evaluate(data, stock, r["upperResidualBudgetKg"])

        self.assertEqual(evaluate(start)["proposedResidualUpperKg"], 0)
        for site, row in start["sites"].items():
            row["residualSizeMm"] = stock["retainedStock"][site]["maxSizeMm"]
            row["throughSets"] = 2
        self.assertAlmostEqual(
            evaluate(start)["estimatedR103UpperMarginKg"],
            r["outcomes"]["allStocksAndFourteenSetsKg"]["upperMarginKg"],
        )
        for key, v in (
            ("residualSizeMm", [45, 19, 7]),
            ("residualSizeMm", [float("nan"), 19, 7]),
            ("maxDensityKgM3", 9000),
            ("throughSets", 3),
            ("throughSets", False),
        ):
            bad = copy.deepcopy(start)
            bad["sites"]["anchor-tab-L"][key] = v
            with self.assertRaises(ValueError):
                evaluate(bad)
        for remove in ("anchor-tab-L", "mount-riser"):
            bad = copy.deepcopy(start)
            del bad["sites"][remove]
            with self.assertRaises(ValueError):
                evaluate(bad)
        start["allOtherPrimaryHardware"] = "unknown"
        with self.assertRaises(ValueError):
            evaluate(start)
