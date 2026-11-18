"""Check physical identities and independently recompute represented upper mass headroom."""
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/"design/assembly-accounting"


class AssemblyAccountingTest(unittest.TestCase):
    def test_pose_coverage_and_distinct_articles_in_each_candidate(self):
        review=json.loads((HERE/"review.json").read_bytes())
        for color,cases in review["configurations"].items():
            self.assertEqual(set(cases),{"baseline","light","steel","stop","light+stop","steel+stop"})
            for case in cases.values():
                self.assertEqual(case["configuration"],"shop-duplex-545-"+color)
                self.assertEqual(case["covered"],545*4)
                self.assertEqual(case["roles"],{"installed":2168,"included":4,"service":8,"external":0})
                self.assertEqual({side:len(segments) for side,segments in case["secondaryLines"].items()},{"L":3,"R":3})
                self.assertEqual((case["primaryLines"],case["transducerUnits"],case["includedDrawWirePerPose"]),(2,1,1))
                self.assertEqual(case["knownLooseWheelScrews"],36)
                self.assertEqual(case["bearings"]["requiredUnits"],12)
                self.assertEqual(case["bearings"]["unitsPerPack"],4)
                self.assertIsNone(case["bearings"]["packsToCoverKnownShortfall"])
                self.assertFalse(case["orderQuantitiesKnown"])
                self.assertFalse(case["inventoryComplete"])
                self.assertIsNone(case["configurationMarginKg"])
            self.assertEqual(cases["baseline"]["representedInstances"],556)

    def test_mass_against_source_and_rejected_controls(self):
        read=lambda name:json.loads((ROOT/name).read_bytes())
        cases=read("design/assembly-accounting/review.json")
        headroom=read("design/competition-package/shop-duplex/mass.json")["poses"]["stowed"]["R103_margin_at_high_kg"]
        stop=read("design/lift-arrest/study.json")["bracketFit"]["place"]["export"]["massWithoutFastenersKg"]
        for material,file in (("light","clamp-review.json"),("steel","steel-review.json")):
            candidate=headroom-stop-read("design/installed-procurement/"+file)["deltaKg"]
            for color in ("red","blue"):
                self.assertAlmostEqual(cases["configurations"][color][material+"+stop"]["representedMarginKg"][0],candidate)
        self.assertTrue(all(control["rejected"] for control in cases["controls"].values()))


if __name__ == "__main__":
    unittest.main()

