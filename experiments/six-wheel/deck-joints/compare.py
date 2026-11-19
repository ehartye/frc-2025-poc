"""Issue all three source-verified mass cases against the same covered and wired robot."""
import argparse
from pathlib import Path
from account import HERE, read, run, sha, write


def compare(plugin,sources,out):
    results={}
    for scenario in ("harness","restrictedNoMotorCreditKg","conditionalMatchedArticleDeltaKg"):
        incoming=sources/scenario/"supported-covers.json"
        selected=read(incoming)
        if not {"fixed-lead-install","supported-drive-covers","sp1-install"}<=set(selected["selected"]):
            raise ValueError("Wrong source configuration")
        if scenario!="harness":
            mode=next(c for c in selected["changes"] if c["id"]=="fixed-lift-motor-omitted")
            if mode["delta"]["pointer"]!="/"+scenario: raise ValueError("Incorrect motor credit")
        result=run(plugin,incoming,out/scenario)
        result["sourcePins"]={k["id"]:k["sha256"] for k in selected["sources"]}
        results[scenario]=result
    write(HERE/"comparison.json",results)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("plugin","sources","out"): p.add_argument(k,type=lambda v:Path(v).resolve())
    a=p.parse_args()
    compare(a.plugin,a.sources,a.out)
