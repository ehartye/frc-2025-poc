import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
DIR=ROOT/"od-gate"

def load(name):
    return json.loads((DIR/name).read_bytes())

class SplitODReceiver(unittest.TestCase):
    def test_native_identity_and_all_registered_parts(self):
        native=load("package.json")
        digest=hashlib.sha256((DIR/"split-gate.step").read_bytes()).hexdigest()
        self.assertEqual(native["nativeSha256"],digest)
        self.assertEqual(load("installed.json")["splitGateNativeSha256"],digest)
        self.assertEqual(load("observer.json")["sourceNative"],digest)
        self.assertEqual(load("engineering.json")["nativeSha256"],digest)
        self.assertEqual(native["bodyCount"],len(native["volumeMm3"]))
        self.assertEqual(set(native["poses"]),{"station","receive","clamp","place","half","open"})
        self.assertTrue(all(not hits for row in native["poses"].values() for hits in row.values()))
        move=load("moving.json")
        self.assertEqual(move["nativeSha256"],digest)
        self.assertEqual(len(move["samples"]),28)
        self.assertTrue(all(not row["interiorMm3"] for row in move["samples"]))
        old=load("intermediate-guide-control.json")
        self.assertGreater(max(max(row["interiorMm3"].values()) for row in old["samples"]),2900)

    def test_fixed_gate_and_legacy_counterexamples(self):
        new=load("installed.json")
        self.assertEqual(new["stationCount"],47)
        self.assertGreater(new["wholeDockBound"]["gap_lower_bound_mm"],8)
        self.assertTrue(new["nativeDock"][-1]["intersections_mm3"])
        for name,case in new["pipe"].items():
            with self.subTest(case=name):
                self.assertFalse(case["unexpected"])
                gap=case["allBodyDeparture"]
                self.assertEqual(gap["bodyCount"],new["registeredRobotCount"])
                self.assertGreater(gap["minimumSubsequentMm"],1)
                self.assertGreater(gap["firstHeldOneMmClearS"],case["geometricSurfaceExitS"])
        self.assertGreater(new["pipe"]["long-yawed"]["allBodyDeparture"]["firstHeldOneMmClearS"],4)
        old=load("rigid-control.json")["data"]["pipe"]["center-both-gates"]["unexpected"]
        self.assertGreater(old["L-nose-pulley"]["peakMm3"],600)
        self.assertGreater(old["guide-130-1"]["peakMm3"],4000)

    def test_source_bound_grid_and_replay(self):
        study=load("study.json")
        for path,digest in study["sourceSha256"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest)
        rows={r["id"]:r for r in study["rows"]}
        grid=[r for k,r in rows.items() if k.startswith("grid-")]
        self.assertEqual(len(grid),64)
        self.assertEqual(sum(r["conditionalSequence"] for r in grid),52)
        for name in ("reference-A","reference-B","finest"):
            self.assertTrue(rows[name]["conditionalSequence"])
        for name in ("single-side-late","gate-stalled","near-lip","early-gate","reduced-power","long-offset"):
            self.assertFalse(rows[name]["conditionalSequence"])
        code="from motion import Case,run;import json;print(json.dumps(run(Case(withdraw=0))))"
        result=json.loads(subprocess.check_output([sys.executable,"-B","-c",code],cwd=DIR))
        self.assertEqual(result["eventsS"],rows["reference-A"]["eventsS"])
        self.assertEqual(result["seatedS"],rows["reference-A"]["seatedS"])

    def test_custody_unobservable_controls_and_budget(self):
        witness=load("observer.json")
        self.assertTrue(all(row["shortestEmptyRayMm"]>=99 for row in witness["clearRayProbes"].values()))
        for case in witness["stationCases"].values():
            self.assertEqual({k for k,v in case["controls"].items() if v},{"valid"})
        eng=load("engineering.json")
        self.assertLess(eng["mass"]["cases"]["red-low-light-and-shared-masts"]["afterUpperReplacementKg"],0)
        self.assertIsNone(eng["mass"]["finishedConfigurationMarginKg"])
        budget=eng["mass"]["cases"]["red-low-light-dedicated-masts-physically-deleted-pneumatics"]
        self.assertAlmostEqual(budget["afterUpperReplacementKg"],.568,places=3)
        self.assertEqual(len(eng["mass"]["noAir"]["removedIds"]),6)
        for k,name in (("inspectionSha256","inspection.json"),("deletionSha256","deletion.json")):
            self.assertEqual(eng["mass"]["noAir"][k],hashlib.sha256((ROOT/"design/fixed-no-air"/name).read_bytes()).hexdigest())
        self.assertTrue(eng["opening"]["withinAllocations"])
        self.assertTrue(eng["hood"]["withinAllocations"])
        self.assertGreater(eng["guide"]["singleGuideBendingMPa"],300)
        self.assertFalse(eng["physicalAuthorization"])
