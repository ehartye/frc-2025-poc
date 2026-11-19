import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/"design/signal-light"


class SignalLight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.study=json.loads((FOLDER/"review.json").read_bytes())
        cls.cases=cls.study["cases"]

    def test_wrong_product_envelope_and_keyed_panel(self):
        self.assertEqual(self.study["lock_register_sha256"],
                         hashlib.sha256((ROOT/"design/fixed-height/native.json").read_bytes()).hexdigest())
        for row in self.cases.values():
            self.assertAlmostEqual(row["mast_trial_top_mm"]-1066.8,5.9,places=2)
            self.assertGreater(row["mast_trial_interference_mm3"]["backup-L-drilled-mast"],500)
            self.assertGreater(row["round_hole_key_interference_mm3"],3)
            self.assertGreater(min(row["bearing_witness_mm3"].values()),90)
            self.assertFalse(row["hardware_interference_mm3"])
            self.assertFalse(row["spacer_interference_mm3"])
            self.assertFalse(row["lead_reserve_interference_mm3"])

    def test_starting_footprint_and_extension(self):
        for key, height_margin, bodies in [("high",28.3,584),("low-receive",46.8,578)]:
            row=self.cases[key]
            self.assertEqual(row["installed_model_bodies"],bodies)
            self.assertAlmostEqual(row["height_margin_mm"],height_margin)
            self.assertEqual(row["bumper_off_xy_mm"],[-350,-350,350,350])
            self.assertGreater(min(row["perimeter_corner_witness_mm3"]),.48)
        self.assertEqual(self.cases["low-place"]["bumper_off_xy_mm"],[-350,-350,535,350])

    def test_full_stroke_view_and_occluded_alternative(self):
        sweep=self.cases["low-receive"]["low_slide_sweep"]
        self.assertEqual(sweep["moving_ids"],157)
        self.assertFalse(sweep["lamp_envelope_pairs"])
        for view in [sweep["right"],*[r["right_view"] for r in self.cases.values()]]:
            self.assertEqual(len(view),9)
            self.assertEqual([r["visible_of_nine"] for r in view],[9]*9)
        self.assertEqual([r["visible_of_nine"] for r in self.cases["low-place"]["inboard_front_trial"]],[0]*9)

    def test_mass_allowance_preserves_measurement_gate(self):
        review=json.loads((FOLDER/"accounting.json").read_bytes())
        for key in ("low","high"):
            row=review["cases"][key]
            self.assertEqual(row["missing"],[])
            self.assertGreater(row["representedMarginKg"][0],.49)
            self.assertIsNone(row["configurationMarginKg"])
