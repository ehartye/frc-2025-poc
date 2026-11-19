"""Guard the intermediate obstruction that four clear endpoint poses failed to reveal."""
from pathlib import Path
import sys
import unittest

PACKAGE=Path(__file__).resolve().parents[1]/"design"/"competition-package"
sys.path.insert(0,str(PACKAGE))
from build import electrical, native, pose_layers, rotation_bounds
from travel import swept_prism


class CombinedRobotReview(unittest.TestCase):
    def test_harness_endpoints_and_two_bad_routes(self):
        start=pose_layers(560,0)[0]["Receiver drive"]
        # The bracket moves at one-half shelf extension; model everything in its frame.
        bracket=start["slide-sync-bracket-79"]
        original=native.brep(["box",[20,115,470,16,10,15]])
        for stroke,overlap in ((0,0),(90,240),(100,320),(110,320),(120,120),(370,0)):
            anchor=native.translated(original,(stroke/2,0,0))
            self.assertAlmostEqual(native.intersection(anchor,bracket),overlap,places=5)
        second=native.brep(["box",[20,-40,470,16,10,15]])
        motor=start["slide-power-NEO550"]
        self.assertGreater(native.intersection(swept_prism(second,370),motor),1500)
        revised=electrical.electrical_parts({"height":560,"stroke":0})["shelf-harness-moving-anchor"]
        self.assertLess(native.intersection(swept_prism(revised,370),motor),0.001)
        self.assertLess(native.intersection(swept_prism(revised,185),bracket),0.001)

    def test_intermediate_register_and_variable_geometry(self):
        start,_,_,frames=pose_layers(560,0)
        middle=pose_layers(700,123)[0]
        rope="return-line-L"
        a=native.bounds(start["Lift"][rope])
        b=native.bounds(middle["Lift"][rope])
        self.assertAlmostEqual(b[2]-a[2],140)
        self.assertAlmostEqual(b[5],a[5])
        [wire]=[name for name,frame in frames.items() if frame=="variable-draw-wire"]
        a=native.bounds(start["Receiver drive"][wire])
        b=native.bounds(middle["Receiver drive"][wire])
        self.assertAlmostEqual(b[0],a[0])
        self.assertAlmostEqual(b[3]-a[3],123)
        for name,entry in rotation_bounds().items():
            offset=native.offset({"frame":entry["frame"]},{"height":700,"stroke":123})
            bounds=native.bounds(middle["Receiver drive"][name])
            envelope=[v+offset[i%3] for i,v in enumerate(entry["bounds_mm"])]
            self.assertTrue(all(bounds[i]>=envelope[i]-1e-6 and bounds[i+3]<=envelope[i+3]+1e-6 for i in range(3)))
