import itertools
import json
import sys
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]/"design/stability"
sys.path.insert(0,str(HERE))
from stability import Load,extremum
from weigh import compare


class Stability(unittest.TestCase):
    def test_uncertain_center_extrema_match_independent_corner_enumeration(self):
        a=Load("a",(2,5),(-20,0,0,-10,0,0),(0,)*6)
        b=Load("b",(1,4),(50,0,0,70,0,0),(0,)*6)
        values=[(m*x+n*y)/(m+n) for m,n,x,y in itertools.product(a.mass,b.mass,(-20,-10),(50,70))]
        self.assertAlmostEqual(extremum([a,b],(1,0,0),True),max(values))
        self.assertAlmostEqual(extremum([a,b],(1,0,0),False),min(values))
        self.assertAlmostEqual(extremum([a,b],(-1,0,0),True),-min(values))

    def setUp(self):
        self.review=json.loads((HERE/"review.json").read_text())
        self.sample={"pose":"transport","load":"seated","stepSha256":self.review["poses"]["transport"]["step_sha256"],
            "source":"invented fixture","basis":"synthetic","units":"kg","absoluteErrorKg":.05,
            "scales":{f"wheel-{axle}-{side}":9. for axle in ("front","center","rear") for side in "LR"}}

    def test_symmetric_scale_fixture_and_conserved_mass_side_shift(self):
        result=compare(self.sample,self.review)
        self.assertEqual(result["static_balance"],"within targets")
        self.assertEqual(result["basis"],"synthetic")
        self.assertLess(result["turn_with_unrestricted_outer_axle_loading"]["frictionRequired"],.9)
        for axle in ("front","center","rear"):
            self.sample["scales"][f"wheel-{axle}-L"]+=3
            self.sample["scales"][f"wheel-{axle}-R"]-=3
        shifted=compare(self.sample,self.review)
        self.assertEqual(shifted["static_balance"],"outside targets")
        self.assertEqual(result["mass_kg"],shifted["mass_kg"])
        self.assertFalse(shifted["checks"]["radial_CG"])
        self.assertGreater(shifted["turn_with_unrestricted_outer_axle_loading"]["frictionRequired"],
                           result["turn_with_unrestricted_outer_axle_loading"]["frictionRequired"])

    def test_rejects_wrong_robot_or_unloaded_scale(self):
        self.sample["stepSha256"]="b"*64
        with self.assertRaises(ValueError):
            compare(self.sample,self.review)
        self.sample["stepSha256"]=self.review["poses"]["transport"]["step_sha256"]
        self.sample["scales"]["wheel-center-L"]=0
        with self.assertRaises(ValueError):
            compare(self.sample,self.review)
