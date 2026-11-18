import hashlib
import json
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"design/receiver-lift"
sys.path.insert(0,str(HERE))
from delivery_load import audit


class DeliveryRetentionTest(unittest.TestCase):
    def test_mass_sources_account_for_every_moving_article(self):
        saved=json.loads((HERE/"delivery-retention.json").read_bytes())
        live=audit()
        self.assertEqual(saved["registeredMoving"], json.loads(json.dumps(live)))
        self.assertEqual((401,174),(live["counts"]["all"],len(live["bodyMassKg"])))
        self.assertAlmostEqual(live["loadedKg"][1],11.608626266,places=7)
        self.assertIsNone(live["weighedMassKg"])

    def test_native_layer_reopened_against_each_complete_pose(self):
        fit=json.loads((HERE/"duplex-fit.json").read_bytes())
        for pose,r in fit.items():
            self.assertFalse(r["externalOverlapMm3"] or r["internalOverlapMm3"] or r.get("installedServicePinOverlaps"))
            self.assertEqual(hashlib.sha256((ROOT/"design/competition-package/poses"/(pose+".step")).read_bytes()).hexdigest(),r["sourceSha256"])
        self.assertEqual(fit["place"]["uniqueReopenedLayerBodies"],fit["place"]["addedBodies"])
        self.assertEqual(fit["place"]["layerStepSha256"],hashlib.sha256((HERE/"duplex-place.step").read_bytes()).hexdigest())

    def test_compliant_and_mixed_winders_fail_the_same_guide_limit(self):
        from retention import catch
        saved=json.loads((HERE/"delivery-retention.json").read_bytes())
        for row in saved["oneSideCatch"] + [saved["mixedCoreControl"],saved["compliantLineControl"]]:
            actual=catch(saved["candidateLoadedUpperKg"],row["slackMm"],row["effectiveStiffnessNPerMm"],.253125)
            self.assertAlmostEqual(actual["peakLineN"],row["peakLineN"],places=7)
            self.assertEqual(actual["withinReserved0p8mm"],row["withinReserved0p8mm"])
        self.assertTrue(all(r["withinReserved0p8mm"] for r in saved["oneSideCatch"]))
        self.assertFalse(saved["mixedCoreControl"]["withinNominal1mmShoeGap"])
        self.assertFalse(saved["compliantLineControl"]["withinNominal1mmShoeGap"])
        self.assertIsNone(saved["componentScreen"]["commonShaftFreeRotationRemainingDropBoundMm"])
        self.assertFalse(saved["physicalQualification"])
