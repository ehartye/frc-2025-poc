"""Check the registered stop fixture, counterexamples, source coverage and signed mass."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"design/terminal-mounts"
def read(p):
    return json.loads(p.read_bytes())


class TerminalMountTest(unittest.TestCase):
    def test_all_native_sources_and_negative_controls(self):
        native=read(HERE/"native.json")
        for color in ("red","blue"):
            directory=ROOT/"design/competition-package/shop-duplex"/color
            for pose,review in read(directory/"review.json").items():
                if pose not in ("transport","receive","place","stowed"):
                    continue
                r=native[color+"-"+pose]
                self.assertEqual(r["nativeSha256"],review["export_sha256"])
                for name in ("installedIntersectionsMm3","kitInternalIntersectionsMm3","toolApproachIntersectionsMm3"):
                    self.assertEqual(r[name],{})
                self.assertEqual(r["nativeIds"],545)
                self.assertTrue(all(v>177 for v in r["sleeveOverlapInUndrilledMastMm3"].values()))
        negative=native["original"]
        self.assertGreater(negative["kitInternalIntersectionsMm3"]["L-bracket-reserve"]["L-lower-screw"],67)
        self.assertGreater(negative["toolApproachIntersectionsMm3"]["R-lower-key"]["R-lower-KM20"],31)
        mounted=native["red-place"]["mountedExport"]
        self.assertEqual(32,mounted["count"])
        self.assertEqual(mounted["sha256"],hashlib.sha256((HERE/"mounted-place.step").read_bytes()).hexdigest())
        self.assertLess(mounted["maximumReopenErrorMm3"],1e-6)
        compat=read(HERE/"compatibility.json")
        self.assertEqual((compat["mountedSolids"],compat["lightSolids"]),(32,86))
        self.assertTrue(all(not v for v in compat["bridgeIntersectionsMm3"].values()))

    def test_mass_and_failure_cannot_be_promoted_to_inspection(self):
        study=read(HERE/"study.json")
        low,high=study["addedKg"]
        self.assertLess(low,study["nominalDeltaKgIfOneKM20IncludedPerDamper"])
        self.assertGreater(high,study["nominalDeltaKgIfOneKM20IncludedPerDamper"])
        cases=study["terminalCases"]
        self.assertFalse(cases["single"]["fullStrokeCatalogScreen"])
        self.assertFalse(cases["slowBalanced"]["fullStrokeCatalogScreen"])
        self.assertTrue(cases["uneven"]["fullStrokeCatalogScreen"])
        a=read(ROOT/"design/carriage-bridge/review.json")["R103HighMarginKg"]
        c=read(ROOT/"design/installed-procurement/clamp-review.json")["deltaKg"]
        for record in read(HERE/"accounting.json")["configurations"].values():
            self.assertAlmostEqual(record["representedMarginKg"][0],a-c-high)
            self.assertTrue(record["overlappingStopRejected"])
            self.assertEqual("source-pinned",record["limitBasis"])
            self.assertIsNone(record["configurationMarginKg"])
            self.assertFalse(record["orderQuantitiesKnown"])
