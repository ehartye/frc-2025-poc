import hashlib
import importlib.util
import json
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"stereo-custody"
sys.path.insert(0,str(HERE))
from imaging import W,H,exposure,np
from observe import read_pair

DATA=json.loads((HERE/"results.json").read_bytes())
NATIVE=json.loads((HERE/"native.json").read_bytes())


class IndependentPipePosition(unittest.TestCase):
    def test_measured_contour_comes_from_pixels(self):
        empty=np.full((H,W),40,dtype=np.uint8)
        background=[(empty,np.full((H,W),3000.)) for _ in range(2)]
        pics=exposure(background,[315.67,0,937.15])
        result=read_pair(pics,[empty,empty],940,240)
        self.assertEqual("position-witness",result["status"])
        self.assertTrue(result["u_interval_mm"][0]<=75.67<=result["u_interval_mm"][1])
        self.assertNotEqual("position-witness",read_pair([pics[0],empty],[empty,empty],940,240)["status"])
        self.assertNotEqual("position-witness",read_pair([empty,empty],[empty,empty],940,240)["status"])

    def test_false_port_picture_and_stale_camera_do_not_prove_custody(self):
        cases=DATA["schedule"]
        self.assertEqual(1380,cases["nominal"]["first_corroborated_ms"])
        self.assertEqual(17,len(DATA["all_integer_stereo_phases"]))
        self.assertEqual({1380},{r["first_corroborated_ms"] for r in DATA["all_integer_stereo_phases"]})
        for name in ("replayed-ports-empty-shelf","replayed-ports-weak-grip","replayed-ports-pipe-at-rear-edge",
                     "true-ports-weak-grip","uncalibrated","old-attempt","different-pose","repeated-camera-capture"):
            self.assertEqual(0,cases[name]["positive_ticks"],name)
        for name in ("moving","camera-disconnected"):
            self.assertEqual(1420,cases[name]["first_revoked_ms"],name)
        for name in ("wrong-forward","behind-seat","airborne","lateral","missing-eye","flat-clutter","contrast-lost",
                     "loose-errors","yaw-8"):
            self.assertNotEqual("position-witness",DATA["frames"][name]["status"],name)
        self.assertFalse(DATA["physical_load_measured"])
        self.assertFalse(DATA["carry_permission"])

    def test_pose_identity_and_negative_load_alternatives(self):
        self.assertEqual(0,NATIVE["added_mass_kg"])
        self.assertEqual(6,len(NATIVE["poses"]))
        for pose in NATIVE["poses"]:
            self.assertEqual((545,120,0,{}),(pose["bodies"],pose["rays"],pose["out_of_frame"],pose["positive_obstacles"]))
            if pose["step_sha256"]: self.assertEqual(pose["step_sha256"],DATA["pose_sha256"][pose["name"]])
        self.assertGreater(NATIVE["min_gap_disk_to_seated_pipe_projection_mm"],6)
        hal=json.loads((HERE/"hal.json").read_bytes())
        for data in (DATA,hal):
            for path,sha in data["source_sha256"].items():
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),sha,path)
        self.assertGreater(hal["scheduler_ticks"],2000)
        self.assertGreater(hal["camera_frames"],800)
