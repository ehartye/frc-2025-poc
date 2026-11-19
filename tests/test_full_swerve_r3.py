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

    def test_component_override_survives_resolution_and_step_writes(self):
        import importlib.util
        import os
        import tempfile
        plugin = os.environ.get("AGENT_FRC_PLUGIN_ROOT")
        if not plugin or not (Path(plugin)/"scripts/component_palette.py").is_file():
            self.skipTest("Set AGENT_FRC_PLUGIN_ROOT to run the installed palette regression")
        spec = importlib.util.spec_from_file_location("full_swerve_builder", DESIGN/"build.py")
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        with tempfile.TemporaryDirectory() as scratch:
            root = Path(scratch)
            (root/"cad").mkdir()
            override = root/"cad/component-palette.json"
            for colors,expected in [({"electrical":"#123456"},"#123456"),({},"#8263A9")]:
                override.write_text(json.dumps({"schema":"agent-frc/component-palette@1","colors":colors}))
                before = override.read_bytes()
                resolved = builder.resolve_project_palette(root,Path(plugin))["colors"]
                self.assertEqual(expected,resolved["electrical"])
                original = builder.block([0,0,0,1,2,3])
                parts = {"controller-fixture":(original,"fixture","project-cad",False)}
                rows = builder.write_step(root/"fixture.step",parts,resolved,{})
                self.assertEqual(rows[0]["role"],"electrical")
                self.assertEqual(before,override.read_bytes())


    def test_native_and_public_registration_sources_match(self):
        register = read(DESIGN/"source-register.json")
        for source, expected in register["projectSources"].items():
            self.assertEqual(hashlib.sha256((ROOT/source).read_bytes()).hexdigest(),expected,source)
        snapshot=(json.dumps(register["historicalPaletteSnapshot"],indent=2)+"\n").encode("utf8")
        self.assertEqual(hashlib.sha256(snapshot).hexdigest(),register["historicalPaletteSha256"])
        self.assertEqual(register["r3UsedRoles"],{r["role"]:self.colors[r["role"]] for r in self.rows.values()})
        self.assertEqual(register["historicalBumper"]["sRGB"],self.colors["bumper-red"])
        self.assertEqual(register["representedPublicSolids"]["public-module-representation"],24)
