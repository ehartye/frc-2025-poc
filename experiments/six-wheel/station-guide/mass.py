"""Calculate the two face allocations and the combined alternatives from their actual source ledgers."""
import argparse
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def calculate(harness):
    fit=HERE/"fit-narrow.json"
    total=sum(json.loads(fit.read_bytes())["volumesMm3"].values())
    back=total*.8/3.8
    liner=total*3/3.8
    base_path=ROOT/"sp1-mount/optics-mass.json"
    guard_path=ROOT/"chain-covers/accounting.json"
    base=json.loads(base_path.read_bytes()); guard=json.loads(guard_path.read_bytes())["repacked-light"]
    air=json.loads(harness.read_bytes())
    if abs(air["priorRepresentedKg"][1]-base["representedKg"][1])>1.e-9:
        raise ValueError("Different base for the optional fixed harness")
    limit=base["representedKg"][1]+base["representedMarginKg"][0]
    aluminum=back*2700/1.e9
    recommendations={}
    for rho in (300,500,700):
        allocated=aluminum+liner*rho/1.e9
        reservations={}
        for name,additional in {"guideOnly":0,"guideAndCovers":guard["conditionalDeltaKg"][1],
                                "guideAndHarness":air["netAddedKg"][1],
                                "guideHarnessAndCovers":guard["conditionalDeltaKg"][1]+air["netAddedKg"][1]}.items():
            subtotal=base["representedKg"][1]+allocated+additional
            reservations[name]={"representedUpperKgBeforeAttachments":subtotal,"representedMarginKg":limit-subtotal}
        recommendations[str(rho)]={"guideFaceKg":allocated,"alternatives":reservations}
    return {"fitSha256":digest(fit),"baseSourceSha256":digest(base_path),"optionalCoverSourceSha256":digest(guard_path),
            "optionalHarnessSourceSha256":digest(harness),"limitKg":limit,"baseRobotUpperKg":base["representedKg"][1],
            "optionalHarnessUpperKg":air["netAddedKg"][1],"optionalCoverUpperKg":guard["conditionalDeltaKg"][1],
            "backingVolumeMm3":back,"linerVolumeMm3":liner,"backingKgAt2700KgM3":aluminum,
            "hypotheticalLinerDensityKgM3":recommendations,"mountingKg":None,"measuredMassKg":None}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("harness",type=Path);a=p.parse_args()
    (HERE/"mass.json").write_text(json.dumps(calculate(a.harness),indent=2)+"\n",encoding="utf8",newline="\n")
