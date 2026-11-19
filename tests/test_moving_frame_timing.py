import hashlib
import json
from pathlib import Path
import unittest


STUDY = Path(__file__).resolve().parents[1] / "moving-frame-timing"


class CameraTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.record = json.loads((STUDY / "results.json").read_text())

    def test_inputs_and_complete_trial_coverage(self):
        for name, expected in self.record["source_sha256"].items():
            self.assertEqual(hashlib.sha256((STUDY / name).read_bytes()).hexdigest(), expected, name)
        self.assertEqual(self.record["counts"], {"HAL": 1428, "same-consumer": 59364})

    def test_nominal_reference_and_first_missed_phase(self):
        for piece in ("coral-A", "coral-B"):
            nominal = self.record["hal"]["20ms/constant"][piece]
            self.assertEqual(nominal["accepted_phase_ticks"], nominal["historical_accepted_phase_ticks"])
            profile = self.record["profiles"]["5ms/constant/" + piece]
            for delay in range(7, 19):
                self.assertEqual(profile[str(delay)], [85, 85])
            self.assertEqual(profile["24"], [0, 85])
        self.assertEqual(self.record["profiles"]["5ms/constant/coral-A"]["19"], [84, 85])

    def test_faults_including_same_batch_and_between_poll_expiry(self):
        for cadence in ("5ms", "20ms"):
            for piece in ("coral-A", "coral-B"):
                for fault in ("asymmetric", "one-late", "duplicate-clear", "drop", "silence",
                              "wrong-revision", "old-epoch", "future-stamp", "unattributed-pad"):
                    self.assertEqual(self.record["hal"][cadence + "/" + fault][piece]["accepted_phase_ticks"], {})
