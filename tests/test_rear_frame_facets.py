"""Check the public corner recommendation and its source geometry controls."""

import hashlib
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "cad/rear-frame-facets"


class RearFrameFacets(unittest.TestCase):
    def test_published_geometry_and_service_controls(self):
        design = json.loads((STUDY / "construction.json").read_bytes())
        fit = json.loads((STUDY / "fit.json").read_bytes())
        self.assertEqual(
            hashlib.sha256(
                (STUDY / "rear-facet-construction.step").read_bytes()
            ).hexdigest(),
            design["stepSha256"],
        )
        archive = (
            ROOT
            / "cad/assembly-engineering/receiver-service/issue/engineering-reopen.json"
        )
        self.assertEqual(
            hashlib.sha256(archive.read_bytes()).hexdigest(),
            fit["sourceNativeObservations"],
        )
        self.assertEqual(design["parentArchive"], fit["sourceArchive"])
        self.assertEqual(len(design["coldPartVolumesMm3"]), 5)
        self.assertEqual(len(design["pairInterferenceMm3"]), 10)
        self.assertLess(max(design["pairInterferenceMm3"].values()), 0.001)
        self.assertEqual(design["frameSetbackMm"], 19.05)
        self.assertGreater(design["minimumExtraRawCrossRailReachEachEndMm"], 5.9)
        self.assertEqual(design["rearAnchorBoreCount"], 4)
        for corner in ("RL", "RR"):
            with self.subTest(corner=corner):
                row = fit["controls"][corner + "-19.05"]
                self.assertEqual(row["registeredSupplierGroups"], 75)
                self.assertGreater(row["allAzimuth"]["gapMm"], 4.68)
                self.assertGreater(row["fullCartridgeWithdrawal"]["gapMm"], 4.68)
                self.assertGreater(row["detachableHardwareMinimumBoxGapMm"], 8.69)
                for compare in fit["controls"][corner + "-25.4"][
                    "actualFixedPlates"
                ].values():
                    self.assertLess(compare["gapMm"], 0.20)
                    self.assertGreater(compare["inward025CommonMm3"], 0.55)
                for compare in fit["controls"][corner + "-50.8"][
                    "actualFixedPlates"
                ].values():
                    self.assertGreater(compare["nominalCommonMm3"], 610)
        self.assertGreater(fit["originalCarrier"]["diagonalProtectionMarginMm"], 40)
        self.assertEqual(fit["battery"]["verticalGapToBumperWhenExitingMm"], 223.5)


if __name__ == "__main__":
    unittest.main()
