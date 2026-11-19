"""Selected axle and bearing evidence stays bound to its native geometry."""
import hashlib
import json
import math
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1] / "axle-retention"
WHEELS = {f"{position}-{side}" for position in ("front", "center", "rear") for side in "LR"}


def receipt(name):
    return json.loads((PACKAGE / name).read_bytes())


def sha(name):
    return hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest()


class AxleRetentionTests(unittest.TestCase):
    def test_native_inputs_match_downstream_evidence(self):
        kit = receipt("fit.json")
        keepers = receipt("keepers.json")
        self.assertEqual((kit["reopenedBodies"], keepers["count"]), (32, 72))
        self.assertEqual(kit["stepSha256"], sha("axle-kit.step"))
        self.assertEqual(keepers["stepSha256"], sha("bearing-keepers.step"))
        self.assertTrue(set(kit["volumeMm3"]).isdisjoint(keepers["volumeMm3"]))
        self.assertEqual(len(kit["volumeMm3"]) + len(keepers["volumeMm3"]), 104)
        for row in (receipt("bearing.json"), receipt("service.json")):
            self.assertEqual(row["kitSha256"], kit["stepSha256"])
            self.assertEqual(row["keeperSha256"], keepers["stepSha256"])
        for item in (kit, keepers):
            self.assertLess(item["maxReopenVolumeErrorMm3"], 1e-6)

    def test_each_bearing_has_two_positive_stops_and_shield_clearance(self):
        source = receipt("bearing.json")
        self.assertEqual(len(source["checks"]), 12)
        for result in source["checks"].values():
            self.assertEqual(result["beforeStopCommonMm3"], 0)
            for volume in result["eccentricContactRangeMm3"]:
                self.assertAlmostEqual(volume, source["areaMm2"] * .05, places=4)
        expected = {f"{wheel.split('-')[0]}-{face}-{wheel[-1]}" for wheel in WHEELS for face in ("outer", "inner")}
        self.assertEqual(set(source["keeperChecks"]), expected)
        keepers = receipt("keepers.json")["locations"]
        self.assertEqual(set(keepers), expected)
        for location, result in keepers.items():
            self.assertEqual(result["unretainedCarrierCommonAtHalfMm3"], 0)
            self.assertGreater(result["oppositeCarrierCommonAt005Mm3"], 0)
            self.assertGreater(result["keeperCommonAt011Mm3"], 0)
            self.assertGreater(min(source["keeperChecks"][location]["eccentricPastStopVolumeRangeMm3"]), 0)
            self.assertGreater(result["nearestOther"]["clearanceMm"], 0)
            if not location.startswith("center"):
                self.assertGreater(result["fixedHardwareFullTakeupLowerMm"], 0)
        self.assertGreater(min(source["remainingInnerOuterMarginMm"]), 0)
        self.assertGreater(min(source["keeperAt025RunoutMarginsMm"].values()), 0)

    def test_stopped_center_shafts_remain_serviceable(self):
        old = receipt("original.json")["centerOutwardExtraction"]
        service = receipt("service.json")
        self.assertEqual(set(service["wheels"]), WHEELS)
        for location, result in service["wheels"].items():
            self.assertEqual(result["outwardTravelMm"], 160)
            self.assertEqual(result["sweptMaterialOverlapMm3"], {})
            self.assertGreater(result["finalBareRailClearanceMm"], 20)
        for side in "LR":
            self.assertEqual(old[side]["outwardTranslationMm"], 10)
            self.assertEqual(old[side]["sweptMaterialOverlapMm3"], {})
            result = service["centerRings"][side]
            self.assertGreater(result["nominalSocketEngagementAtStopMm"], 9.7)
            self.assertTrue(result["shortBladeControlCommonMm3"])
            self.assertFalse(result["toolCorridor"]["overlapMm3"])
            self.assertGreater(result["toolCorridor"]["clearanceMm"], 0)

    def test_selected_loads_and_mass_keep_their_limits(self):
        demand = receipt("load-demand.json")
        self.assertEqual([row["axialN"] for row in demand["loads"]], [250, 500])
        for row in demand["loads"]:
            self.assertAlmostEqual(row["axialN"] / demand["minimumGrooveFaceAreaMm2"], row["clipToGrooveMeanUpperMPa"])
            self.assertAlmostEqual(row["axialN"] / (math.pi * (8.35**2 - 7.50**2)), row["thrustFaceMeanMPa"])
        mass = receipt("mass-input.json")
        self.assertEqual(mass["fitSha256"], sha("fit.json"))
        self.assertEqual(mass["keeperSha256"], sha("keepers.json"))
        self.assertLess(mass["groupsKg"]["old_six_shafts_and_two_spacers"][1], 0)
        self.assertEqual(mass["groupsKg"]["twelve_drilled_carriers"][1], 0)
        for index, total in enumerate(mass["selectedDeltaKg"]):
            self.assertAlmostEqual(total, sum(pair[index] for pair in mass["groupsKg"].values()))
        account = receipt("accounting.json")
        self.assertEqual((account["addedInstalled"], account["retainedReworked"]), (84, 20))
        self.assertFalse(any(account["coverage"].values()))
        self.assertIsNone(account["physicalMarginKg"])
        self.assertAlmostEqual(52.16312255 - account["representedMassKg"][1], account["representedMarginKg"][0])
        self.assertLess(account["representedMarginKg"][0] - 6 * (.55 - .30), 0)
