"""Verify the published student configuration and its native/private boundary."""
import hashlib
import json
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/"cad/full-swerve-r3"

def read(path):
    return json.loads(path.read_bytes())

class FullSwerveR3(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.appearances=read(DESIGN/"appearances.json")
        cls.rows={r["id"]:r for r in cls.appearances["solids"]}
        cls.colors=read(ROOT/"cad/component-palette.json")["colors"]

    def test_ordered_public_inventory_is_bound_to_its_own_exchange(self):
        self.assertEqual(hashlib.sha256((DESIGN/"full-robot.step").read_bytes()).hexdigest(),self.appearances["sourceSha256"])
        self.assertEqual(len(self.rows),len(self.appearances["solids"]))
        for row in self.rows.values():
            self.assertRegex(self.colors[row["role"]],r"^#[0-9A-F]{6}$")
            self.assertTrue((ROOT/row["source"]).is_file(),row["id"])
            self.assertTrue(all(a<b for a,b in zip(row["boundsMm"][:3],row["boundsMm"][3:])))

    def test_stowed_geometry_and_semantic_covers(self):
        for end in ("front","rear"):
            cover=self.rows[f"bumper-cover-{end}-red"]
            self.assertEqual("#D71920",self.colors[cover["role"]])
            self.assertNotEqual(self.rows[f"bumper-foam-{end}"]["role"],cover["role"])
        self.assertAlmostEqual(500,self.rows["r6-polymer-facing"]["boundsMm"][5],places=3)
        for side in ("L","R"):
            self.assertAlmostEqual(1020,self.rows[f"guide-{side}-machined-mast"]["boundsMm"][5],places=3)
        self.assertEqual(sum(k.startswith("relocated-") for k in self.rows),48)
        self.assertFalse({"deck-main","motor-L","motor-R","gearbox-L","gearbox-R","lift-crossbar-drilled"} & self.rows.keys())
        for k in self.rows:
            self.assertFalse(k.startswith(("fixed-head/","roof-head/","REV-21-3004","wheel-front-","wheel-rear-","wheel-center-")))

    def test_four_public_substitutes_register_to_the_native_wheels(self):
        modules=read(ROOT/"experiments/swerve-drive/packaging/module.json")["modules"]
        for corner,site in modules.items():
            row=self.rows[corner+"-display-tread"]
            self.assertEqual("public-module-representation",row["representation"])
            bb=row["boundsMm"]
            self.assertEqual([(bb[i]+bb[i+3])/2 for i in range(3)],
                             [round(site["xM"]*1000,4),round(site["yM"]*1000,4),38.1])
        self.assertEqual(sum(r["representation"]=="public-supplier-envelope" for r in self.rows.values()),3)

