import hashlib
import json
from pathlib import Path
import unittest


PARTS = Path(__file__).resolve().parents[1] / "ribbed-deck"


def read(name):
    return json.loads((PARTS / name).read_bytes())


class RibbedDeckTests(unittest.TestCase):
    def test_native_panel_matches_the_joined_frame(self):
        geometry = read("geometry.json")
        fit = read("fit.json")
        sha = hashlib.sha256((PARTS / "ribbed-deck.step").read_bytes()).hexdigest()
        self.assertEqual(sha, geometry["stepSha256"])
        self.assertEqual(sha, fit["candidateSha256"])
        self.assertEqual(geometry["solidCount"], 1)
        self.assertEqual(len(geometry["stationsMm"]), 13)
        self.assertGreaterEqual(fit["installedOrReservedBodies"], 1200)
        self.assertEqual(fit["installedRibCheck"]["overlapMm3"], {})
        self.assertEqual(fit["undersideBoxAdversePairs"], {})
        self.assertGreaterEqual(fit["installedRibCheck"]["nearestMm"], 2.99)
        self.assertEqual(len(fit["upperContactLostMm3"]), 14)
        self.assertFalse(any(fit["upperContactLostMm3"].values()))
        self.assertGreater(min(fit["serviceMm"].values()), 0)

    def test_mass_exchange_keeps_motor_credit_separate(self):
        data = read("budget.json")
        cases = data["cases"]
        for case in cases.values():
            self.assertEqual(case["sourceChoices"].count("deck-through-ties"), 1)
            self.assertEqual(case["sourceChoices"].count("integral-deck"), 1)
            self.assertIsNone(case["physicalMarginKg"])
            self.assertAlmostEqual(case["representedMarginKg"][0] - case["priorUpperMarginKg"],
                                   -data["material"]["deltaKg"][1])
        strict = cases["restrictedNoMotorCreditKg"]["representedMarginKg"][0]
        self.assertGreater(strict, .075)
        self.assertAlmostEqual(cases["conditionalMatchedArticleDeltaKg"]["representedMarginKg"][0] - strict, .4)

    def test_structural_and_milling_limits_remain_visible(self):
        loads = read("loads.json")
        nominal = loads["cases"]
        self.assertEqual(loads["deckSha256"], read("geometry.json")["stepSha256"])
        self.assertLess(nominal["battery"]["nominal"]["seriesTravelMm"], 2.2)
        self.assertLess(nominal["service"]["nominal"]["seriesTravelMm"], 1.7)
        self.assertGreater(loads["controls"]["touchdownOneSideSensitivity"]["nominalRibPeakMPa"], 276)
        mill = read("machining.json")
        self.assertEqual(mill["geometrySha256"], loads["deckSha256"])
        reserved = [r for r in mill["trials"] if r["placementReserveMm"] > 0]
        self.assertGreater(reserved[0]["uncutStockMm3"], mill["finishStockVolumeReserveMm3"])
        self.assertLess(reserved[-1]["uncutStockMm3"], mill["finishStockVolumeReserveMm3"])
        self.assertIsNone(mill["actualMachinedVolumeMm3"])
