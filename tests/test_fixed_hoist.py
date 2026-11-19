"""Coupled receipts for removal identity, stops and the adjacent mast and shelf variants."""
import hashlib
import json
from pathlib import Path
import unittest


PARTS = Path(__file__).resolve().parents[1]/"fixed-hoist"


def read(name):
    return json.loads((PARTS/name).read_bytes())


def sha(name):
    return hashlib.sha256((PARTS/name).read_bytes()).hexdigest()


class FixedHoistTests(unittest.TestCase):
    def test_both_reopened_stops_react_on_fixed_arms(self):
        fit = read("fit.json")
        self.assertEqual(fit["stepSha256"],sha("stationary-shaft.step"))
        self.assertEqual(len(fit["volumeMm3"]),9)
        for side in ("L","R"):
            witnesses = fit["axialContactsAt0p6mm"][side]
            self.assertEqual(witnesses["spacerToBearingMm3"],0)
            for key in ("spacerToArmMm3","drumInToSleeveMm3","drumOutToStopMm3","stopToPinMm3","pinToShaftMm3"):
                self.assertGreater(witnesses[key],1.)
            self.assertEqual(fit["shaftPull750mm"][side]["overlapMm3"],{})
            self.assertGreater(fit["shaftPull750mm"][side]["installedPinInterferenceMm3"],150.)
        for check in [*fit["installedChecks"].values(),*fit["serviceChecks"].values()]:
            self.assertEqual(check["overlapMm3"],{})

    def test_only_retired_stock_offsets_new_pieces(self):
        for name in ("budget.json","mast-budget.json"):
            record=read(name)
            evidence=record["evidence"]
            self.assertEqual(record["fitSha256"],sha("fit.json"))
            self.assertEqual(len(evidence["removedOccurrencesKg"]),48)
            self.assertEqual(len(evidence["retiredArticles"]),36)
            self.assertFalse(any("carriage" in k or "service" in k for k in evidence["retiredArticles"]))
            self.assertAlmostEqual(evidence["grossRecoveryKg"],sum(evidence["removedOccurrencesKg"].values())+evidence["trimCreditKg"])
            self.assertAlmostEqual(evidence["trimCreditKg"],.8*evidence["oldBookedExtensionKg"])
            self.assertEqual(record["installedAfter"],record["installedBefore"]-37+9)
            self.assertAlmostEqual(record["representedMarginKg"][0]-record["priorUpperMarginKg"],.8260060744)
            self.assertIsNone(record["physicalMarginKg"])

    def test_joined_mast_service_and_unmatched_shelf_control(self):
        fit=read("mast-fit.json")
        self.assertEqual(fit["shaftStepSha256"],sha("stationary-shaft.step"))
        for side in ("L","R"):
            self.assertGreater(fit["service"][side+"-splice"]["nearestMm"],49.)
            self.assertEqual(fit["service"][side+"-shaftPull"]["overlapMm3"],{})
        combined=read("combined.json")
        self.assertEqual(combined["selectedChanges"][-2:],["mast-foot-splices","fixed-secondary-haul"])
        self.assertGreater(combined["cases"]["densityMatchedRemovalKg"]["exampleFaceOnlyUpperMarginKg"],.27)
        self.assertLess(combined["cases"]["noCutCreditKg"]["exampleFaceOnlyUpperMarginKg"],-.64)
        self.assertIsNone(combined["physicalInspectionMarginKg"])

    def test_signed_impact_exposes_existing_carriage_lock(self):
        transfer=read("lock-transfer.json")
        self.assertAlmostEqual(transfer["lockSpacingM"],.445)
        for case in transfer["cases"].values():
            self.assertEqual(case["sampleCount"],4801)
            self.assertGreater(case["singleSidePeakPinBendingMPa"],840.)
            heavy=max(row["nominalPinBendingMPa"] for row in case["bilateralVerticalCouple"].values())
            self.assertGreater(heavy,430.)
            self.assertGreater(case["totalForeAftCoupleAtFullShoeHeightN"],8200.)
        self.assertIsNone(transfer["actualContactShare"])
