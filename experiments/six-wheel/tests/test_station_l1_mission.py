import json
import math
import unittest
from scripts import station_l1_mission as sim, station_l1_phase as phase


class StationMissionTest(unittest.TestCase):
    def test_identity_and_continuity(self):
        run = sim.run()
        self.assertEqual(run["outcome"],"paired-trough-first-contact")
        self.assertTrue(all(r["piece_id"]==run["piece_id"] for r in run["events"]+run["trace"]))
        for a,b in zip(run["trace"],run["trace"][1:]):
            dt = b["time_s"]-a["time_s"]
            self.assertGreaterEqual(dt,0)
            self.assertLessEqual(math.dist(a["xyz_m"],b["xyz_m"]),2.5*dt+1e-6)
        self.assertIsNone(run["score"])
        names = [e["event"] for e in run["events"]]
        self.assertLess(names.index("retract"),names.index("drive-and-turn"))
        self.assertLess(names.index("left-powered-shelf-with-same-piece"),
                        names.index("finite-pipe-envelope-clear-of-shelf-by-1mm"))
        self.assertAlmostEqual(run["events"][0]["xyz_m"][2],.88+sim.R,places=8)

    def test_gates_keep_failure_boundaries_separate(self):
        expected={"weak-grip":"recover-rear-edge-departure","slow-release":"receiver-recontact",
                  "occupied-target":"hold-and-recover-target-occupied",
                  "sensor-timeout":"recover-at-station-unconfirmed-seat"}
        for case,end in expected.items():
            with self.subTest(case=case):
                run = sim.run(case)
                self.assertEqual(run["outcome"],end)
                self.assertFalse(any("clear-of-shelf" in e["event"] for e in run["events"]))

    def test_paired_region_rejects_unregistered_support(self):
        self.assertFalse(sim.panel_support(.6,0))
        run=sim.run(target_y=1.0)
        self.assertEqual(run["outcome"],"floor-support-unresolved")

    def test_steps_converge_to_same_kind_at_both_mass_limits(self):
        for case in ("reference","light-pipe"):
            coarser=sim.run(case,.002)
            fine=sim.run(case,.0005)
            self.assertEqual(coarser["outcome"],fine["outcome"])
            self.assertLess(abs(coarser["events"][-1]["time_s"]-fine["events"][-1]["time_s"]),.01)

    def test_finite_clock_and_independent_clear_confirmation(self):
        self.assertEqual(phase.rehearse()["stop"],"await-empty-robot-and-separate-target-observations")
        interrupted=phase.rehearse(15,True,"AUTO")
        self.assertEqual(interrupted["elapsed_s"],15)
        self.assertNotIn("first_contact_time_s",interrupted["pieces"]["coral-A"])
        repeated=phase.rehearse(witnessed_clear=True)
        self.assertEqual(list(repeated["pieces"]),["coral-A","coral-B"])
        self.assertEqual(repeated["pieces"]["coral-B"]["release_time_s"],35)
        for edition in repeated["assessments"].values():
            self.assertIsNone(edition["officialEarnedPoints"])
            self.assertTrue(all(p["l1CriteriaAtInstant"]=="unknown" for r in edition["samples"] for p in r["pieces"]))
        blocked=phase.rehearse(witnessed_clear=True,targets=(0,.05))
        self.assertEqual(blocked["stop"],"target-overlaps-previous-piece-envelope")
        self.assertEqual(len(blocked["pieces"]),1)
