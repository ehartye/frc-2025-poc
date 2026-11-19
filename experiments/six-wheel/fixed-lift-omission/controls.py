"""Audit the load branch and actual desktop motion events for a permanently locked carriage."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def read(p):
    return json.loads(p.read_bytes())


def run(path):
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    result=read(ROOT/"integrated-slide/result.json")
    if digest!=result["rackEvents"]:
        raise ValueError("Wrong calibrated rack trace")
    with path.open(encoding="utf8") as stream:
        events=list(csv.DictReader(stream))
    counts=Counter(row["event"] for row in events)
    if set(counts)!={"feed","seat","carry","release","withdraw","motion","stow","hold"}:
        raise ValueError("Unreviewed command event")
    power=read(ROOT/"fixed-low-power/results.json")["cases"]
    lift={k:case["branches"]["Lift"]["peakBranchA"] for k,case in power.items()}
    if max(lift.values())!=0:
        raise ValueError("A fixed-role trial requires a powered height axis")
    circuit=next(r for r in read(ROOT/"design/electrical-inspection/circuits.json")["branches"] if r["load"]=="Lift")
    summary={"eventDigest":digest,"releasedRackCases":len(result["rackCases"]),"recordedEvents":dict(counts),
        "fixedRolePeakLiftBranchA":lift,"isolatedPhysicalBranch":circuit,
        "heightRole":"Fixed H560 only. Both retained locks, sleeves, keepers and mast attachments require inspection.",
        "onLostLockOrHeightEvidence":"Stationary inhibit and externally supported recovery; no powered rehome.",
        "oldVariableHeightProfileCompatible":False,"harnessAndFirmwareCommissioned":False}
    (HERE/"controls.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("events",type=Path)
    run(p.parse_args().events)
