import collections
import csv
import hashlib
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STUDY=ROOT/"low-observation"


class LowObservationEvidence(unittest.TestCase):
    def test_bound_inputs_and_scheduler(self):
        result=json.loads((STUDY/"results.json").read_bytes())
        hal=json.loads((STUDY/"hal.json").read_bytes())
        for base,inputs in ((ROOT,result["inputs"]),(STUDY,hal["input_sha256"])):
            for name,digest in inputs.items():
                self.assertEqual(hashlib.sha256((base/name).read_bytes()).hexdigest(),digest,name)
        self.assertEqual("LOW_WINDOWS=68 TICKS=1496 ELIGIBLE=23",hal["results"]["LowWindowTest"])
        self.assertEqual("CUSTODY_TICKS=1038 FRAMES=1192 POSITIVE=22",hal["results"]["CustodyClockTest"])

    def test_missing_track_holds_and_immediate_is_conditional(self):
        primary=json.loads((STUDY/"results.json").read_bytes())
        immediate=json.loads((STUDY/"immediate.json").read_bytes())
        contacts=json.loads((STUDY/"contacts.json").read_bytes())
        for pose in primary["poses"].values():
            self.assertEqual((577,6,32),(pose["bodies"],len(pose["machined"]),len(pose["added"])))
        for key,total in (("coral-A",11),("coral-B",12)):
            study=primary["pieces"][key]
            self.assertTrue(all(not phase["timely"] for phase in study["phases"]))
            self.assertEqual(study["station"]["nominal"]["first_corroborated_ms"],
                             study["truth_ms"]["resume-from-modeled-java-in-band"]+100)
            for cause in ("wrong-pose","wrong-epoch","empty","uncalibrated"):
                self.assertEqual(0,study["station"][cause]["positive_ticks"])
            self.assertIsNotNone(study["station"]["stale"]["first_revoked_ms"])
            trial=immediate["pieces"][key]
            self.assertFalse(trial["continuous_track_established"])
            self.assertFalse(trial["physically_calibrated"])
            self.assertEqual(total,sum(p["time_window_only"] for p in trial["phases"]))
            onsets={p["tick_ms"]+lag for p in trial["phases"] if p["time_window_only"] for lag in range(11)}
            self.assertEqual(onsets,set(map(int,trial["onsets_ms"])))
            for outcome in trial["onsets_ms"].values():
                self.assertGreater(outcome["minimum_whole_front_gap_mm"],32.97)
                self.assertEqual("sustained-paired-floor-and-lip",outcome["outcome"])
            self.assertEqual("receiver-recontact",contacts[key]["slide-held"]["stop"])
            self.assertIsNone(contacts[key]["slide-held"]["official_score"])
