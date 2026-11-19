"""Guard the scoped cradle evidence and the failures that prevent issuing it."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "cradle-receiver"

def data(name):
    return json.loads((HERE / (name + ".json")).read_bytes())

class CradleReceiver(unittest.TestCase):
    def test_piece_and_contact(self):
        result = data("study")
        for path, digest in result["inputs"].items():
            self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), digest, path)
        rows = {row["id"]: row for row in result["rows"]}
        self.assertGreater(result["priorLongCutTravelMm"], 3 * result["priorAvailableMm"])
        self.assertFalse(result["cutLengthQualification"])
        for name in ("backstop-A","backstop-B","zero-rolling-loss","high-rolling-loss","fine","finer"):
            self.assertTrue(rows[name]["conditionalSequence"], name)
        self.assertEqual(rows["flat"]["outcome"], "escaped-before-dump")
        self.assertFalse(rows["V-medium-loss"]["conditionalSequence"])
        self.assertEqual(rows["no-withdrawal"]["outcome"], "recontact-after-cradle-clear")
        self.assertEqual(rows["overtilt"]["outcome"], "inner-riser-envelope")
        grid = [row for key,row in rows.items() if key.startswith("factor-")]
        self.assertEqual(sum(r["conditionalSequence"] for r in grid),44)
        self.assertEqual(sum(r["seatAtExistingTime"] is True for r in grid),56)
        self.assertTrue(all(r["support"]["bothRailCoverageMm"] > 31 for r in grid))

    def test_replay_centered_contact(self):
        command = "from study import BASE,one; import json; print(json.dumps(one(('backstop-A', BASE))))"
        result=json.loads(subprocess.check_output([sys.executable,"-B","-c",command],cwd=HERE))
        original=next(r for r in data("study")["rows"] if r["id"]=="backstop-A")
        self.assertEqual(result, original)

    def test_native_and_station(self):
        native, installed = data("native"), data("installed")
        self.assertEqual(hashlib.sha256((HERE/"cradle.step").read_bytes()).hexdigest(),native["nativeSha256"])
        self.assertEqual(native["nativeSha256"],installed["nativeSha256"])
        self.assertEqual((len(native["retiredIds"]),native["bodyCount"]),(96,16))
        self.assertTrue(all(not p["internalOverlapMm3"] and not p["originalBodyHitsMm3"] for p in native["poses"]))
        self.assertGreater(installed["fieldSweep"]["gap_lower_bound_mm"],15)
        cases=installed["receiverSamples"]
        self.assertEqual(cases["nominal"]["otherContacts"],{})
        for name,side in (("left-long","L"),("right-short","R")):
            self.assertGreater(cases[name]["otherContacts"]["rail-"+side]["maxMm3"],330)

    def test_load_and_mass_rejections(self):
        proc=subprocess.check_output([sys.executable,"-B",str(HERE/"engineering.py")])
        e=json.loads(proc)
        self.assertEqual(e,data("engineering"))
        self.assertLess(e["remainingRepresentedMarginKg"],0)
        self.assertLess(e["captureSignedTorqueNm"][0],-e["vendorStageFailureReferenceNm"])
        self.assertGreater(e["captureSignedTorqueNm"][1],7)
        self.assertFalse(e["regulatedCandidateDrive"]["retainedSingle"]["feasible"])
        self.assertTrue(e["regulatedCandidateDrive"]["candidateThree"]["feasible"])
        self.assertFalse(e["qualifiedForHardware"])
        self.assertTrue(all(0<r["minimumFrontRearShare"]<.1 and r["minimumSufficientFriction"]<.031 for r in data("route")))

if __name__=="__main__":
    unittest.main()
