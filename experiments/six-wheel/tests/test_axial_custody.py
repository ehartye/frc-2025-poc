import hashlib
import json
import math
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"axial-spin"
sys.path.insert(0,str(HERE))
from pipe_axial_contact import Contact,G,I,K,M,R,patches,ramp
from pipe_spin_route import DATA,acquire
from spin_cases import CANDIDATE

STUDY=json.loads((HERE/"study.json").read_bytes())


class AxialCustody(unittest.TestCase):
    def test_chute_rolling_and_frictionless_slip_have_different_momentum(self):
        source=DATA["feed"]
        no_friction=ramp(source,0)
        energy=(source["fall_speed_mps"]*math.sin(math.radians(35)))**2+2*G*math.sin(math.radians(35))*source["run_m"]
        self.assertEqual("sliding",no_friction["mode"])
        self.assertAlmostEqual(no_friction["speed"]**2,energy)
        self.assertEqual(no_friction["spin"],0)
        dry=ramp(source,.4)
        self.assertEqual(dry["mode"],"rolling")
        self.assertAlmostEqual(dry["spin"]*R,dry["speed"])
        self.assertAlmostEqual(dry["speed"],source["lip_mps"])
        self.assertEqual(patches(.3,0,CANDIDATE),[])
        self.assertTrue(all(p[0]<0 for p in patches(.08,0,CANDIDATE)))

    def test_impulse_and_sources_follow_each_identified_pipe(self):
        self.assertAlmostEqual(STUDY["unforced_reset"]["residual_Nms"],-I*STUDY["unforced_reset"]["before_rad_s"])
        for path,sha in STUDY["model_sources"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),sha)
        for pipe,run in STUDY["cases"]["nominal"].items():
            self.assertEqual(pipe,run["id"])
            witness=run["pickup"]["impact"]
            after=run["phase_entry_spin_rad_s"]["capture"][1]
            self.assertAlmostEqual(I*(after-witness["spin_before_rad_s"]),R*witness["floor_tangent_impulse_Ns"])
            self.assertAlmostEqual(run["release_spin_rad_s"],run["target"]["input_spin_rad_s"])
            self.assertLess(abs(run["angular_momentum_residual_Nms"]),1e-10)
            self.assertLessEqual(run["force_fraction"],1+1e-12)
            self.assertGreater(run["min_shelf_normal_N"],0)
            self.assertTrue(run["belt_motor_envelope"]["feasible"])
            self.assertEqual("sustained-paired-floor-and-lip",run["target"]["buzzer_assessment"]["status"])

    def test_changed_feeds_and_release_are_kept_as_counterexamples(self):
        got,_,_=acquire(Contact(),.0005)
        self.assertEqual("hold-unconfirmed-seat",got["status"])
        self.assertLess(got["seat_u_m"],.072)
        for run in STUDY["cases"]["force-following-instant-command"].values():
            self.assertLess(run["pickup"]["capture_travel_m"][0],-.1)
        for case,stop in (("velocity-command-high","inner-riser-bound"),("velocity-command-low","receiver-recontact"),
                          ("low-belt-preload","hold-unconfirmed-seat")):
            self.assertEqual({stop},{r["status"] for r in STUDY["cases"][case].values()})
        for case in ("coarse-time","fine-time","fine-belt-height"):
            for piece,run in STUDY["cases"][case].items():
                base=STUDY["cases"]["nominal"][piece]
                self.assertEqual(base["status"],run["status"])
                self.assertLess(abs(run["target"]["end_teleop_s"]-base["target"]["end_teleop_s"]),.002)

    def test_native_and_discrete_coverage(self):
        native=json.loads((HERE/"geometry.json").read_bytes())
        timing=json.loads((HERE/"field-timing.json").read_bytes())
        self.assertEqual(hashlib.sha256((HERE/"study.json").read_bytes()).hexdigest(),timing["study_sha256"])
        self.assertEqual(STUDY["model_sources"]["axial-spin/pipe_axial_contact.py"],native["contact_source_sha256"])
        self.assertEqual(native["intake"]["unexpected_envelope_names"],[])
        self.assertEqual(native["intake"]["supported_nominal_enclosure_mm3"],{})
        for key,departure in native["departures"].items():
            self.assertEqual(departure["tight_17_phases_clear"],17)
            for row in departure["native_material_samples"]:
                self.assertEqual(row["forward_endpoint_inward_grip_overlap_mm3"],{})
                self.assertTrue(all(v["overlap_mm3"]==0 for v in row["bodies"].values()))
            self.assertGreater(departure["min_tail_gap_mm"],32)
            data=timing["cases"][key]
            self.assertTrue(all(v["lower_bound_m"]>0 for v in data["loaded_bounds_m"].values()))
            onsets=data["tight_pixel_integer_ms_settling"]
            self.assertEqual([r[0] for r in onsets],list(range(onsets[0][0],onsets[-1][0]+1)))
            self.assertTrue(all(r[1]=="sustained-paired-floor-and-lip" for r in onsets))
        self.assertEqual(["coral-A","coral-B"],[p["id"] for p in native["issued_human_schedule"]["qualified_supports"]])
