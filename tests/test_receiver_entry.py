"""Independent transformed tube points and registered rigid-body witnesses."""
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import unittest

SUPPLY = Path(__file__).resolve().parents[1] / "station-supply"
spec = importlib.util.spec_from_file_location("entry_envelope",SUPPLY/"entry_envelope.py")
envelope = importlib.util.module_from_spec(spec)
spec.loader.exec_module(envelope)


class ReceiverEntryTest(unittest.TestCase):
    def test_rotated_translated_rim_and_ballistic_bow_stay_inside(self):
        p,q = (32,998),(42,990)
        radius,half_length = 57.15,150.8125
        profile,half = envelope.projected_section(p,q,99,5,.5,.25,.6,.048,radius,half_length)
        edges = list(zip(profile,profile[1:]+profile[:1]))
        for dx,dy,sign,t in itertools.product((-5,5),(-.5,.5),(-1,1),(0,.25,.5,1)):
            angle = math.radians(sign*.25)
            for i,end in itertools.product(range(32),(-half_length,half_length)):
                phase = i*math.tau/32
                x = p[0]+t*(q[0]-p[0])+radius*math.cos(phase)-99-dx
                z = p[1]+t*(q[1]-p[1])+4*t*(1-t)*.048+radius*math.sin(phase)
                y = end-dy
                xx = 99+x*math.cos(angle)+y*math.sin(angle)
                yy = -x*math.sin(angle)+y*math.cos(angle)
                self.assertLessEqual(abs(yy)+.6,half+1e-8)
                self.assertGreaterEqual(min((bx-ax)*(z-az)-(bz-az)*(xx-ax)
                                            for (ax,az),(bx,bz) in edges),-1e-8)

    def test_relocated_hardware_and_failure_witnesses(self):
        data=json.loads((SUPPLY/"entry-window.json").read_bytes())
        self.assertEqual(data["reopened_stop_step"],hashlib.sha256((SUPPLY/"receiver-stops.step").read_bytes()).hexdigest())
        self.assertEqual(data["compensated_pose"],hashlib.sha256((SUPPLY/"compensated-dock.json").read_bytes()).hexdigest())
        self.assertEqual(data["body_count"],545)
        for side in data["stop_checks"].values():
            self.assertAlmostEqual(side["added_mm3"],0)
            self.assertAlmostEqual(side["supported_footprint_mm2"],8)
            self.assertEqual(side["unresolved_other_body_boxes"],{})
            self.assertAlmostEqual(side["lug_positions"]["-0.5"]["distance_mm"],0)
            self.assertAlmostEqual(side["lug_positions"]["-0.75"]["overlap_mm3"],7)
        _,working,wide,_=data["revised"]
        self.assertGreater(working["hardware_lower_mm"],.113)
        self.assertFalse(working["unresolved_material"])
        self.assertGreater(wide["max_belt_excursion_mm"],data["outward_travel_nominal_mm"])
        self.assertTrue(wide["unresolved_material"])
        witness=data["at_half_mm_side_zero_yaw"]
        self.assertGreater(witness["issued_inward_1_5"]["material_mm3"]["grip-L-upper-cheek"],.19)
        self.assertIsNone(witness["revised_inward_0_6"])
