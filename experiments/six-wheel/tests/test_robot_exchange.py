from pathlib import Path
import hashlib
import json
import subprocess
import sys
import unittest

FOLDER=Path(__file__).resolve().parents[1]/"design/fixed-no-air"


class RobotExchange(unittest.TestCase):
    def test_export_preserves_named_and_independent_solids(self):
        r=json.loads((FOLDER/"exchange.json").read_bytes())
        self.assertEqual(r["native_manifest_sha256"],hashlib.sha256((FOLDER/"whole-robot.json").read_bytes()).hexdigest())
        self.assertEqual(r["reopened_named_solids"],593)
        self.assertEqual(r["independent_root_solids"],593)
        self.assertLess(r["max_independent_volume_error_mm3"],.01)
        self.assertEqual(len(r["changed_or_optical_material_difference_mm3"]),24)
        lamp=r["source_lamp_surface_audit"]
        self.assertEqual(lamp["matched_faces"],1002)
        self.assertGreater(lamp["bidirectional_samples"],10000)
        self.assertGreater(lamp["faces_with_interior_samples"],980)

    def test_shifted_equal_volume_body_is_rejected(self):
        code="""
from export_robot import cad,lamp_surfaces
a=cad.brep(["box",[0,0,0,6,3,4]])
assert lamp_surfaces(a,a)["matched_faces"]==6
try: lamp_surfaces(a,cad.translated(a,[0,.01,0]))
except ValueError: pass
else: raise AssertionError("Undetected surface registration change")
"""
        subprocess.run([sys.executable,"-c",code],cwd=FOLDER,check=True)
