import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/"design/fixed-no-air"


def read(name):
    return json.loads((HERE/name).read_bytes())


class FixedNoAir(unittest.TestCase):
    def test_optical_successor_keeps_moving_surfaces(self):
        manifest=read("whole-robot.json")
        for path,digest in manifest["source_sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest)
        for pose,x in (("receive",425),("transport",425),("place",535)):
            r=manifest["cases"][pose]
            self.assertEqual((r["source_bodies"],r["match_bodies"]),(598,593))
            self.assertEqual(len(r["identical_brep_sha256"]),590)
            self.assertIn("receiver-R6-drilled-for-camera",r["identical_brep_sha256"])
            self.assertEqual(sum(k.startswith("roof-") for k in r["identical_brep_sha256"]),21)
            self.assertEqual(r["source_front"],r["match_front"])
            self.assertAlmostEqual(r["match_front"]["x_mm"],x)
            self.assertLess(r["new_fixed_change_front"]["x_mm"],425)

    def test_service_access_and_positive_contact(self):
        native = read("native.json")
        self.assertEqual(native["pit_step_sha256"],hashlib.sha256((HERE/"pit-pins.step").read_bytes()).hexdigest())
        for name, case in native["cases"].items():
            self.assertEqual(case["match_bodies"], 578 if name=="high" else 593 if "camera" in name else 572)
            for side,row in case["pit_support"].items():
                self.assertTrue(all(v=={} for v in row["nominal_hardware_hits_mm3"].values()))
                for key in ("pin_sweep_hits_mm3","keeper_sweep_hits_mm3","nut_tool_hits_mm3","head_tool_hits_mm3"):
                    self.assertEqual(row[key],{})
                self.assertEqual(row["keeper_sweep_pin_overlap_mm3"],0)
                self.assertTrue(row["prior_tool_hits_mm3"])
                if side=="L":
                    self.assertIn("backup-L-running-line",row["lateral_tool_hits_mm3"])
                self.assertAlmostEqual(row["gap_to_carriage_mm"],1)
                self.assertAlmostEqual(row["adverse_plate_engagement_mm"],6,places=5)
                for key in ("bearing_at_2mm_down_mm3","adverse_bearing_at_2mm_down_mm3","seated_shank_at_half_mm_mm3",
                            "head_at_1mm_forward_mm3","keeper_at_3mm_withdrawal_mm3"):
                    self.assertGreater(row[key],1)
                self.assertGreater(row["wrong_height_carriage_gap_mm"],350)
        bending = native["cases"]["high"]["pit_support"]["L"]["bending"]
        selected = bending["selected"]["sections"]
        self.assertGreater(selected["keeper_bore"]["nominal_stress_MPa"],selected["bushing_exit"]["nominal_stress_MPa"])
        self.assertLess(selected["keeper_bore"]["nominal_stress_MPa"],.5*bending["long_control"]["sections"]["keeper_bore"]["nominal_stress_MPa"])

    def test_separate_mass_intervals_and_camera(self):
        deletion = read("deletion.json")
        for path,digest in deletion["source_sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest)
        self.assertEqual(len(deletion["removed"]),6)
        self.assertNotIn("rev-ph",deletion["removed"])
        result = read("inspection.json")["cases"]
        for mode in ("high","low"):
            self.assertEqual(result[mode+"-red"],result[mode+"-blue"])
        cam = result["low-camera-red"]
        self.assertEqual(cam["represented_items"],result["low-red"]["represented_items"]+21)
        self.assertAlmostEqual(cam["checker_upper_headroom_kg"],1.81114,places=4)
        self.assertAlmostEqual(cam["paired_upper_headroom_kg"],2.61114,places=4)
        self.assertIsNone(cam["finished_margin_kg"])
        for mode, data in read("balance.json").items():
            before, after = data["with_air"],data["no_air"]
            self.assertAlmostEqual(before["running_mass_kg"][0]-after["running_mass_kg"][0],1.8)
            self.assertAlmostEqual(before["running_mass_kg"][1]-after["running_mass_kg"][1],2.6)
            self.assertGreater(after["straight_load_line_margin_mm"],50)
            if mode=="high":
                self.assertTrue(after["straight_park"]["passes_endpoints"])

    def test_partial_deletion_and_added_pneumatic_consumer(self):
        # Separate interpreter preserves the installed CAD project's module paths.
        code = """
from account import qualify
from copy import deepcopy
m={'exports':[{'id':'pose'}], 'assignments':[{'export':'pose','role':'installed','instance':key,'occurrence':key}
    for key in ('tank-vessel','viair-compressor')]}
c={'sensorChannels':{'PHswitch':'sensor','PH0':'M4V210 valve'}}
parts={'tank-vessel','viair-compressor'}
qualify(m,parts,c,'low')
new=deepcopy(c)
new['sensorChannels']['PH1']='new-actuator'
for ids,channels,mode in [(parts-{'tank-vessel'},c,'low'),(parts,new,'low'),(parts,c,'moving')]:
    try: qualify(m,ids,channels,mode)
    except ValueError: continue
    raise AssertionError('Unreviewed removal authorized')
"""
        subprocess.run([sys.executable,"-c",code],cwd=HERE,check=True)
