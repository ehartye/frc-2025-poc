"""Summarize paired pixels, every observed release/fault event, and the loaded contact/power results."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def images(prior,study):
    report={}
    for phase in ("marked","fixed-front","fixed-rear"):
        folder=study/phase
        older=prior/phase
        for path in sorted(folder.glob("*.npz")):
            new_src=read(folder/"scenes.json")["backgroundSha256"]
            old_src=read(older/"scenes.json")["backgroundSha256"]
            if path.stem not in new_src:
                if path.stem not in ("barrel-moving-swept","barrel-moving-nearest"):
                    continue
                key="backgroundSha256" if path.stem.endswith("swept") else "nearestPossibleSha256"
                new_hash=read(folder/"sweep.json")[key]
                old_hash=read(older/"sweep.json")[key]
            else:
                new_hash=new_src[path.stem]
                old_hash=old_src[path.stem]
            old_path=older/path.name
            if sha(path)!=new_hash or sha(old_path)!=old_hash:
                raise ValueError("Unregistered optical background: "+path.name)
            eyes=[]
            with np.load(path) as new,np.load(old_path) as old:
                for eye in ("left","right"):
                    delta=np.abs(new[eye+"_z"]-old[eye+"_z"])
                    if not np.isfinite(delta).all():
                        raise ValueError("Nonfinite depth")
                    eyes.append({"changedIntensityPixels":int((new[eye]!=old[eye]).sum()),
                                 "depthChangedOver0_1mm":int((delta>.1).sum()),"maxDepthDifferenceMm":float(delta.max())})
            report[phase+"/"+path.stem]={"issued":old_hash,"selected":new_hash,"leftRight":eyes}
    return report


def run(study,prior):
    stages=read(study/"scenes.json")
    sweep=read(study/"sweep.json")
    frames={}
    for phase in ("marked","fixed-front","fixed-rear"):
        for path in sorted((study/phase).glob("*.csv")):
            old=prior/phase/path.name
            current=read(path.with_suffix(".json"))
            if sha(path)!=current["frameCsvSha256"]:
                raise ValueError("Changed observation "+path.name)
            frames[phase+"/"+path.stem]={"frames":current["frames"],"registerSha256":sha(path),
                    "matchesIssued":sha(path)==sha(old),"statuses":current["statuses"]}
    expected=read(ROOT/"selected-optics/controller.json")["registeredImages"]
    actual=read(study/"controller.json")
    control=read(study/"rack/results.json")
    tests=read(study/"rack/qualification.json")
    masses=read(HERE/"mass.json")
    if tests["massSource"]!=sha(HERE/"mass.json") or control["movingExtraKg"]!=masses["moving"]["extraToIssuedPlantKg"]:
        raise ValueError("Moving or operating mass changed")
    historical=read(ROOT/"rack-sensing/results.json")["cases"]
    c={}; count_changed=0
    for case,paired in control["cases"].items():
        c[case]={}
        for piece,row in paired.items():
            old=historical[case][piece]
            changed=old.get("traceSha256")!=row.get("traceSha256")
            count_changed+=int(changed)
            c[case][piece]={**row,"traceDiffersFromIssued":changed}
    qualified={}
    for case, evidence in tests["cases"].items():
        contacts=evidence["contacts"]
        power=evidence["power"]
        qualified[case]={"outcomes":[r["outcome"] for r in contacts],
             "minimumFrontM":min(float(r["min_front_m"]) for r in contacts),
             "minimumRiserM":min(float(r["min_riser_m"]) for r in contacts),
             "power":{k:{"minimumV":v["minTerminalV"],"firstInhibitS":v["firstReferenceInhibitWallS"],
                         "branch":v["limitingBranch"]} for k,v in power.items()}}
    source={k:sha(ROOT/k) for k in ("selected-optics/controller-register.json","rack-sensing/results.json",
        "rack-sensing/qualification.json","integrated-slide/mass.json","integrated-slide/visibility.json",
        "sp1-mount/fit.json","rack-sensing/rack_trial.py","slide-feedback/Retreat.java",
        "integrated-slide/render.py","integrated-slide/rack.py","integrated-slide/receipt.py")}
    result={"sources":source,"issuedSceneSources":stages["sourceSha256"],"installationSources":stages["sp1Sources"],
        "nativeScene":stages["hardware"],"shelfMarks":read(study/"marked/marks.json")["inkRectsShelfXYmm"],
        "images":images(prior,study),"detectorRuns":frames,"nativeOcclusion":{"sweptGroups":sweep["dynamicEnvelopes"],
            "guardedAndRemaining":sweep["refinedCounts"]},"fieldRoofSha256":sha(study/"roof-field.json"),
        "fieldRoofCases":{k:{stage:{"fieldBodies":v["fieldBodies"],"testedFrames":len(v["frames"]),
                "excludedBacking":v["excludedRobotBackingPixels"]} for stage,v in r.items()}
                for k,r in read(study/"roof-field.json")["cases"].items()},
        "newObservationRegister":sha(study/"controller-register.json"),
        "registeredImageInputsMatch":expected==actual["registeredImages"],
        "packetCases":len(actual["cases"]),"packetEventsMatch":actual["sameEventsAsReference"],
        "packetEvents":actual["selectedEventsSha256"],"rackHal":sha(study/"rack/hal.json"),
        "rackEvents":sha(study/"rack/events.csv"),"rackCases":c,"differentRackPathCount":count_changed,
        "operatingHighWithoutPipeKg":tests["operatingHighWithoutPipeKg"],"contactPower":qualified,
        "contactPowerRawSha256":sha(study/"rack/qualification.json")}
    (HERE/"result.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("study",type=Path)
    p.add_argument("prior",type=Path)
    a=p.parse_args()
    run(a.study,a.prior)
