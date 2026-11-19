from hashlib import sha256
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
OBSERVER=ROOT/"release-observer"
sys.path.insert(0,str(OBSERVER))
from stereo import LENGTH,clearance,image_pairs
from witness import support_gap_upper,witness


class ReleaseObserverTests(unittest.TestCase):
    def setUp(self):
        common=dict(attempt=7,piece="station-attempt-7",calibration="synthetic-test-only",
                    track_unambiguous=True,track_epoch=3,shelf_registered=True)
        self.before=dict(**common,source="lip-contact",capture_ms=100,sequence=1,
                         pad_id="center",healthy_loaded_edge=True,isolated_pipe_load=True)
        self.after=dict(**common,source="independent-stereo",capture_ms=150,sequence=2,
                        end_marks=image_pairs([(632,i*LENGTH/2,540) for i in (-1,1)]),
                        front_mm=535,top_mm=500)
        # Fake certification exercises input gates only.
        self.cal=dict(id="synthetic-test-only",physical_calibration=True,pixel_error=.1,clock_bound_ms=1,
                      shelf_registration_mm=1,pipe_size_bound_mm=.5,surface_pair_bound_mm=.1,
                      pads=["center"],installed_pad_max_mm=.8)

    def check(self):
        return witness(self.before,self.after,self.cal,7,"station-attempt-7",170)

    def test_independent_contact_and_clearance_bracket(self):
        result=self.check()
        self.assertEqual((99,151),(result["earliestCrossingMs"],result["latestCrossingMs"]))
        self.assertEqual("shelf-only",result["scope"])
        self.assertGreater(result["lowerGapMm"],1)

    def test_two_clear_ports_cannot_create_witness(self):
        self.before["source"]="seat-and-entry"
        with self.assertRaises(ValueError): self.check()

    def test_lost_identity_reset_clock_and_uncalibrated_cases(self):
        for path,field,value in ((self.before,"piece","other"),(self.after,"track_epoch",4),
                                 (self.after,"capture_ms",200),(self.after,"track_unambiguous",False),
                                 (self.cal,"physical_calibration",False),(self.before,"isolated_pipe_load",False),
                                 (self.before,"healthy_loaded_edge",False),(self.cal,"installed_pad_max_mm",1.1)):
            with self.subTest(field=field):
                old=path[field];path[field]=value
                with self.assertRaises(ValueError): self.check()
                path[field]=old

    def test_high_uncertainty_and_swapped_matches_fail(self):
        self.after["end_marks"]=image_pairs([(538,i*LENGTH/2,557.15) for i in (-1,1)])
        with self.assertRaises(ValueError): self.check()
        marks=image_pairs([(632,i*LENGTH/2,540) for i in (-1,1)])
        marks[1]=marks[0]
        with self.assertRaises(ValueError): clearance(marks,535,500,.1,1,.5)

    def test_even_perfectly_coincident_measurements_cannot_establish_contact(self):
        m=image_pairs([(535,0,500)])[0]
        self.assertGreater(support_gap_upper(m,m,.1,0),12)
        self.before.update(source="independent-stereo",pipe_material=m,shelf_material=m)
        with self.assertRaises(ValueError): self.check()

    def test_reopened_robot_package_and_geometry_digest(self):
        doc=json.loads((OBSERVER/"mount-check.json").read_bytes())
        self.assertEqual(sha256((OBSERVER/"stereo-mount.step").read_bytes()).hexdigest(),doc["mount_sha256"])
        for pose,info in doc["poses"].items():
            source=ROOT/"design/station-l1"/(pose+".step")
            self.assertEqual(sha256(source.read_bytes()).hexdigest(),info["source_sha256"])
            self.assertEqual(96,info["solids"])
            self.assertEqual([],info["interferences_mm3"])
        self.assertEqual(24,len(doc["poses"]["place"]["optical_rays"]))
        self.assertFalse(any(r["blockers"] for r in doc["poses"]["place"]["optical_rays"]))
        self.assertFalse(any(r["provisional_guide_blockers"] for r in doc["poses"]["place"]["optical_rays"]))
        self.assertEqual(14,len(doc["part_order"]))

    def test_persisted_replay_exercises_slow_release_and_missing_phases(self):
        reports=json.loads((OBSERVER/"replay.json").read_bytes())["reports"]
        self.assertTrue(all(r["eligible_phases"]==0 for r in reports if r["case"]=="slow-release"))
        for approach in ("centered-bench","field-registered"):
            rows=[r for r in reports if r["case"]=="reference" and r["approach"]==approach]
            self.assertEqual(4,len(rows))
            self.assertEqual(17,rows[0]["eligible_phases"])
            self.assertEqual(0,rows[-1]["eligible_phases"])
            self.assertEqual(0,rows[1]["bounded_timing_phases"])
