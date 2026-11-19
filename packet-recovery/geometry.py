"""Repeat the source-fed physical cases and both registered rack/park geometry runs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
BASE=HERE.parent/"relieved-match"
sys.path.insert(0,str(BASE))
import first_feed
import field_trial
import landing

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def run(raw,cache,jdk):
    cases={}
    for piece,label in (("coral-A","first"),("coral-B","return")):
        first_feed.run(cache,jdk,piece,cache)
        name=label+"-feed.json"
        if sha(cache/name)!=sha(BASE/name):raise ValueError("Acquired states changed: "+piece)
    for mode in ("bridge26","bridge30"):
        dest=cache/mode;dest.mkdir(exist_ok=True)
        landing.run(cache,jdk,cache/("path-"+mode+"-coral-A.csv"),HERE/"events.csv",scenario=mode,
                    registration=HERE,output=dest)
        field_trial.run(dest,modes=(mode,),registration=HERE,landing=dest,output=dest,motion_cache=raw)
        trial=json.loads((dest/"match.json").read_bytes())
        ref=json.loads((BASE/"match.json").read_bytes())
        case=trial["cases"][mode]
        if case!=ref["cases"]["nominal"] or trial["sourceToFinal"]!=ref["sourceToFinal"]:
            raise ValueError("Recovery changed the field or support result")
        with (dest/"landing-observed.csv").open() as f:rows=list(csv.DictReader(f))
        if any(row["outcome"]!="supported" for row in rows):raise ValueError("Trough support failed")
        cases[mode]={"landingSHA256":sha(dest/"landing-observed.csv"),"points":case["points"],
                     "scoredPipes":case["scoredPieces"],"parkWallS":case["parkWallS"],"parkTapeOverlapMm":case["parkTapeOverlapMm"],
                     "sixRobotMarginMm":case["allRobotLowerMm"],"stockMarginMm":case["stowedStockLowerMm"],
                     "pipeToPipeMm":trial["successiveTubeLowerMm"],"minimumPostContactFrontMm":1000*min(float(r["min_front_m"]) for r in rows),
                     "contactIntegrations":len(rows)}
    result={"cases":cases,"sources":{p.name:sha(p) for p in [BASE/n for n in ("first-feed.json","return-feed.json","navigation.json","native-register.json","landing-observed.csv","free-flight-clearance.json","camera-registration.json")]+[HERE/"hal.json",HERE/"events.csv"]}}
    (HERE/"geometry.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    ap=argparse.ArgumentParser(description=__doc__)
    for key in ("raw","cache","jdk"):ap.add_argument(key,type=Path)
    a=ap.parse_args()
    run(a.raw,a.cache,a.jdk)
