import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"end-grip-length"

def read(stem):
    return json.loads((HERE/(stem+".json")).read_bytes())

class EndGripLength(unittest.TestCase):
    def test_optical_and_mechanical_distinctions(self):
        rows=read("replay")
        by_name={r["case"]["name"]:r for r in rows}
        self.assertEqual(by_name["issued"]["optical"],json.loads((ROOT/"design/fixed-height/ports-065.json").read_bytes())["cases"]["0ms-0.0005"])
        for prefix,want in (("cassette-",24),("indexed-box-",0),("timed-",0)):
            self.assertEqual(sum(r["optical"]["outcome"]=="IN_BAND" for n,r in by_name.items() if n.startswith(prefix)),want)
        for length in ("-6.35","0","6.35"):
            for setting in ("-6.35","0","6.35"):
                row=by_name[f"gauge-{length}-{setting}-0"]
                self.assertEqual(row["optical"]["outcome"]=="IN_BAND",length==setting)
        self.assertEqual(by_name["preload-7.0"]["optical"]["outcome"],"physical-custody-lost")
        self.assertEqual(by_name["preload-11.699"]["optical"]["outcome"],"neutral-fault")

    def test_sources_solid_endpoints_and_weight(self):
        pack,gauges=read("package"),read("gauges")
        for path,digest in pack["sources"].items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest,path)
        for name,reference in (("gauge-stops.step",gauges),("fixed-center-cassette.step",pack)):
            self.assertEqual(hashlib.sha256((HERE/name).read_bytes()).hexdigest(),reference["nativeSha256"])
        self.assertTrue(all(n==1 for n in pack["solidCounts"].values()))
        self.assertEqual(pack["newRigidOverlapMm3"],{})
        self.assertEqual(gauges["reopenedSolidCount"],12)
        for variant in gauges["sets"].values():
            for surface in variant["stopFaces"].values():
                self.assertLess(surface["firstTouchGapMm"],.001)
                self.assertLess(surface["endpointOverlapMm3"],.001)
                self.assertGreater(surface["supportedFootprintMm2"],12)
        self.assertGreater(pack["conditionalMassHeadroomKg"]-gauges["maximumFourSpacerMassKgSteel"],.43)
        self.assertIsNone(pack["completeInspectionMarginKg"])

    def test_failed_paths_survive(self):
        r=read("witness")
        self.assertGreater(r["long"][1]["positive"]["grip-L-idler-20T"]["peakMm3"],700)
        for item in ("index-short","index-center","index-long"):
            self.assertEqual(r[item][0]["positive"],{})
        self.assertGreater(r["index-long-side"][0]["positive"]["grip-R-rear-riser"]["peakMm3"],2000)
        self.assertTrue(all(not any(row.values()) for row in read("mechanics")["prototype"].values()))
        cases=read("delivery")
        self.assertEqual({r["result"]["sourceId"] for r in cases},{"coral-A","coral-B"})
        self.assertTrue(all(r["result"]["firstContactOutcome"]=="receiver-recontact" for r in cases))

    def test_reversing_belt_geometry(self):
        program = """import json
from shoe import routing,load,Design
models=[routing(i) for i in (-6,0,6)]
assert all(abs(m['pitchLength']-420)<1e-8 and m['driveWrapTeeth']>8 for m in models)
assert models[-1]['idlerX']-models[0]['idlerX']>3.9
assert load(-6,Design())<5 < load(0,Design())
"""
        subprocess.run([sys.executable,"-B","-c",program],cwd=HERE,check=True)

if __name__=="__main__":
    unittest.main()
