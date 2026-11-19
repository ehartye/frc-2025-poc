import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT/"station-feed-margin"


def read(name):
    return json.loads((STUDY/name).read_bytes())


class FeedMarginTests(unittest.TestCase):
    def test_trace_bound_ports_and_latest_observable_warning(self):
        source = ROOT/"station-landmark/feed.json"
        issued = json.loads(source.read_bytes())
        witness = read("port-witness.json")
        self.assertEqual(witness["issuedFeedSha256"], hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual(witness["programSha256"], issued["programSha256"])
        controls = {k for k,r in issued["outcomes"].items() if r["lastPortState"] == "IN_BAND"}
        for name,row in witness["trials"].items():
            self.assertEqual(row["traceSha256"], issued["outcomes"][name]["trajectorySHA256"])
        for name in ("shiftedHumanSource", "badRegistration"):
            row = witness["trials"][name]
            self.assertEqual(set(row["indistinguishableBeforeStop"]), controls)
            self.assertEqual(row["stopAtControllerMs"], 302)
        for name in ("leftSmall", "rightSmall"):
            row = witness["trials"][name]
            self.assertEqual(row["controllerEdges"][2][:3], [340,0,0])
            self.assertLess(row["stopAtControllerMs"]-340, 28)

    def test_actuator_ramp_and_contact_domain_block_both_rescues(self):
        trials = read("motion.json")
        self.assertEqual(len(trials), 18)
        for name in ("leftSmall", "rightSmall"):
            for command in (0., -.65):
                row = next(r for r in trials if r["case"]==name and r["challengeMps"]==command and r["dtS"]==.00025)
                self.assertEqual(row["firstChangedDemandMs"], 340)
                self.assertGreater(row["stopTravelMm"], 1.4)
                self.assertIsNone(row["firstPortBandMs"])
        self.assertTrue(any(r["case"]=="rightSmall" and r["firstChangedDemandMs"] is None
                            and r["stopAtControllerMs"] is not None for r in trials))

    def test_incomplete_history_cannot_promote_a_belt_prediction(self):
        doc = read("refinement.json")
        for name,row in doc["cases"].items():
            self.assertEqual(row["engineeringAcceptance"], "blocked")
            self.assertEqual(len(row["runs"]), 9)
            timestep = row["runs"][:6]
            self.assertEqual([r["dtSeconds"] for r in timestep], [.0005/2**i for i in range(6)])
            self.assertEqual([r["strips"] for r in row["runs"][6:]], [5,10,20])
            if name == "nominal":
                self.assertEqual(row["status"], "within-declared-bounds")
            else:
                self.assertEqual((row["status"],row["checkerExit"]), ("unknown",2))
                self.assertEqual({bool(r["firstStop"]) for r in timestep}, {True,False})
                self.assertTrue(all(r["firstStop"] for r in row["runs"][6:]))
