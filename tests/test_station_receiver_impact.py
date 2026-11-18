import json
import math
import unittest

from scripts import station_receiver_impact as study


class FirstImpactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = json.loads(study.DESIGN.read_text())

    def test_square_control_recovers_previous_study(self):
        for speed in self.design["entry_speeds_mps"]:
            for offset in self.design["extra_standoff_mm"]:
                new = study.first_impact(speed,offset,220,0,self.design)
                previous = study.station.stand_off_screen(speed,offset)
                self.assertAlmostEqual(previous["flight_time_s"],new["time_s"],places=8)
                if previous["surface"] == "clear-through-center-height":
                    self.assertIsNone(new["post_x_mps"])

    def test_impulse_conserves_tangent_and_dissipates_normal_energy(self):
        for result in study.study(self.design):
            if result["post_x_mps"] is None:
                continue
            vx,vz = result["velocity_in_mps"]
            nx,nz = result["normal_xz"]
            self.assertAlmostEqual(math.hypot(nx,nz),1)
            for e,post_x in zip(self.design["restitution_samples"],result["post_x_mps"]):
                post_z = vz + (1+e)*result["normal_approach_mps"]*nz
                self.assertAlmostEqual(nx*post_x+nz*post_z,e*result["normal_approach_mps"])
                self.assertAlmostEqual(nz*post_x-nx*post_z,nz*vx-nx*vz)
                self.assertLessEqual(post_x**2+post_z**2,vx**2+vz**2+1e-12)

    def test_slow_feed_changes_direction_with_reach_and_standoff(self):
        for radius in (0,6):
            self.assertLess(max(study.first_impact(.5,40,220,radius,self.design)["post_x_mps"]),0)
            self.assertGreater(min(study.first_impact(.5,40,240,radius,self.design)["post_x_mps"]),0)
            self.assertLess(max(study.first_impact(.5,60,240,radius,self.design)["post_x_mps"]),0)
            case = study.first_impact(.5,220,220,radius,self.design)
            self.assertEqual(case["surface"],"clear-through-cutoff")

    def test_rejects_incompatible_geometry(self):
        for extension,radius in ((220,16),(371,0),(270,6)):
            with self.assertRaises(ValueError):
                study.first_impact(.5,0,extension,radius,self.design)
