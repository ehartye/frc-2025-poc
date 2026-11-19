import unittest

from scripts import station_l1_phase as phase, station_l1_settle as model


class TroughContinuationTest(unittest.TestCase):
    def test_fixed_and_late_withdrawals_recontact_the_shelf(self):
        a=model.continue_contact()
        b=model.continue_contact(withdrawal="late")
        self.assertEqual(a["stop"],"receiver-recontact")
        self.assertEqual(a["elapsed_s"],b["elapsed_s"])
        self.assertLess(a["min_shelf_clearance_mm"],0)
        self.assertTrue(.5<a["elapsed_s"]<.7)

    def test_retraction_and_counterexamples_survive_numerical_refinement(self):
        for step in (.0002,.00005):
            good=model.continue_contact(withdrawal="after-separation",dt=step)
            no_friction=model.continue_contact(withdrawal="after-separation",mu=0,dt=step)
            backspin=model.continue_contact(withdrawal="after-separation",initial_roll=-10,dt=step)
            self.assertEqual(good["stop"],"sustained-paired-floor-and-lip")
            self.assertGreater(good["ending_still_double_contact_s"],1)
            self.assertGreater(good["min_shelf_clearance_mm"],40)
            for r in (good,no_friction,backspin):
                self.assertLess(r["peak_total_energy_gain_J"],.001)
            self.assertEqual(no_friction["stop"],"inner-riser-bound")
            self.assertEqual(backspin["stop"],"inner-riser-bound")

    def test_upstream_id_and_supported_contact_required(self):
        with self.assertRaises(ValueError):
            model.continue_contact(case="slow-release")
        wrong=model.mission.run(piece_id="wrong",target_y=-.18)
        with self.assertRaises(ValueError):
            model.continue_contact(upstream=wrong)

    def test_clock_cutoff_while_waiting_does_not_create_a_witness(self):
        interrupted=phase.rehearse(20,trough_continuation=True)
        self.assertEqual(interrupted["elapsed_s"],20)
        self.assertNotIn("post_contact",interrupted["pieces"]["coral-A"])
        complete=phase.rehearse(witnessed_clear=True,trough_continuation=True)
        self.assertTrue(all(v["post_contact"]=="sustained-paired-floor-and-lip" for v in complete["pieces"].values()))
        self.assertTrue(all(e["officialEarnedPoints"] is None for e in complete["assessments"].values()))
