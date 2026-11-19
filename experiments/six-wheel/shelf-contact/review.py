"""Freeze each contact-resolution experiment, then run and review the acquired pipe through 330 ms."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict,replace
import json
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np
from experiment import CompliantPipe,Shelf,GUIDE,source,ROOT,HERE,feed

sys.path.insert(0,str(ROOT/"station-feed-margin"))
from refinement import sha,artifact,write

DEFAULT={"dt":.000125,"shelfStations":8,"guideAngles":32,"guideRings":2,"beltStrips":5}
LEVELS={"dt":[.0005,.00025,.000125,.0000625,.00003125,.000015625],
        "shelfStations":[4,8,16],"guideAngles":[16,32,64],"guideRings":[2,4,8],"beltStrips":[5,10,20]}
METRICS=[("cartridgeMm","mm","Maximum requested outward belt travel.",.05),
         ("compressionMm","mm","Maximum normal generator compression while the shelf exerts force.",.05),
         ("peakShelfN","N","Peak total force normal to the finite shelf.",25.)]
WINDOW=.330


def plan(case, base, levels_by_axis):
    delta,shift=source(case);pipe=CompliantPipe(delta,shift,GUIDE)
    g=asdict(GUIDE);del g["sectors"],g["radial_bins"]
    f=asdict(feed.cfg);del f["slices"]
    s=asdict(pipe.shelf);del s["stations"]
    law={"guide":g,"belt":f,"shelf":s,"modelFiles":{str(p.relative_to(ROOT).as_posix()):artifact(p) for p in
         (ROOT/"return-station/feed.py",ROOT/"station-guide/guide.py",HERE/"contact.py")}}
    records={"scenario":{"case":case,"robotMmDeg":delta,"releaseShiftMm":shift,"source":artifact(ROOT/"station-guide/optics-narrow.json")},
             "constitutiveLaw":law,"initialState":{"p":pipe.p.tolist(),"v":pipe.v.tolist(),"axis":pipe.a.tolist(),
             "angularMomentum":pipe.l.tolist(),"beltMps":.65},"inputs":{"forwardBeltMps":.65,"window":[0,WINDOW]},
             "runtime":{"numpy":np.__version__,"python":platform.python_version(),"study":artifact(Path(__file__))},
             "outcomeDefinition":"Contact-at-horizon requires continuous normal support above 0.1 N throughout the last 5 ms. Stop on first cartridge hard-limit excursion, loss or incomplete coverage."}
    doc={"schema":"agent-frc/contact-convergence-manifest@1","comparisonPins":{k:sha(v) for k,v in records.items()},
         "windowEndSeconds":WINDOW,"metrics":[{"id":k,"units":u,"definition":d,"absoluteTolerance":t} for k,u,d,t in METRICS],
         "studies":[]}
    jobs=[]
    for key,values in levels_by_axis.items():
        held={k:v for k,v in base.items() if k!=key};levels=[]
        for i,v in enumerate(values):
            cfg={**held,key:v};label=f"{key}-{i+1}";jobs.append((case,label,cfg))
            levels.append({"runId":label,"resolution":[v],"modelSha256":sha(law),"settingsSha256":sha(cfg)})
        doc["studies"].append({"id":key,"axis":"timestep" if key=="dt" else "contact-geometry",
            "resolutionParameters":[key],"fixedSettingsSha256":sha(held),"levels":levels})
    return records,doc,jobs


def simulate(job):
    case,label,cfg=job;delta,side=source(case)
    old=feed.cfg
    try:
        feed.cfg=replace(old,slices=cfg["beltStrips"])
        g=replace(GUIDE,sectors=cfg["guideAngles"],radial_bins=cfg["guideRings"])
        p=CompliantPipe(delta,side,g,Shelf(stations=cfg["shelfStations"]))
        p.belt=.65;last_unloaded=0.
        while WINDOW-p.time>1.e-11 and not p.loss and not p.hard:
            p.moves(.65,min(cfg["dt"],WINDOW-p.time))
            if p.last_normal_n<=.1:last_unloaded=p.time
        return {"case":case,"label":label,"cartridgeMm":p.peak,"compressionMm":1000*p.peak_shelf_compression,
                "peakShelfN":p.peak_shelf_n,"actualWindowS":p.time,"lastUnloadedS":last_unloaded,"stop":p.hard,"loss":p.loss}
    finally:
        feed.cfg=old


def run(plugin,cache,fine=False):
    cache.mkdir(parents=True,exist_ok=True);plans={};jobs=[]
    base,levels=DEFAULT,LEVELS
    if fine:
        base={"dt":.0000625,"shelfStations":16,"guideAngles":128,"guideRings":4,"beltStrips":10}
        levels={"dt":[.0000625,.00003125,.000015625,.0000078125],"shelfStations":[8,16,32],
                "guideAngles":[64,128,256],"guideRings":[4,8,16],"beltStrips":[5,10,20,40]}
    for case in ("leftSmall","rightSmall","nominal"):
        pins,plans[case],local=plan(case,base,levels);jobs.extend(local)
        write(cache/(case+"-parameters.json"),pins);write(cache/(case+"-manifest.json"),plans[case])
    with ProcessPoolExecutor(max_workers=6) as pool:rows=list(pool.map(simulate,jobs))
    summary={}
    for case,doc in plans.items():
        data={r["label"]:r for r in rows if r["case"]==case};runs=[]
        for study in doc["studies"]:
            for level in study["levels"]:
                r=data[level["runId"]];okay=abs(r["actualWindowS"]-WINDOW)<1e-9 and not r["stop"] and not r["loss"]
                runs.append({"id":level["runId"],"comparisonPins":doc["comparisonPins"],"fixedSettingsSha256":study["fixedSettingsSha256"],
                    "modelSha256":level["modelSha256"],"settingsSha256":level["settingsSha256"],"windowEndSeconds":WINDOW,
                    "completion":"complete" if okay else "failed","termination":"horizon" if okay else "physical-limit",
                    "outcome":"contact-at-horizon" if okay and WINDOW-r["lastUnloadedS"]>=.005 else "unresolved",
                    "reportedIntegrityPassed":True,"metrics":[{"id":k,"units":u,"value":r[k] if okay else None} for k,u,_,_ in METRICS]})
        receipt=cache/(case+"-receipt.json");write(receipt,{"schema":"agent-frc/contact-convergence-receipt@1","basis":"simulation-observed","runs":runs})
        manifest=cache/(case+"-manifest.json")
        proc=subprocess.run(["node",str(plugin/"scripts/check-contact-convergence.mjs"),str(manifest),str(receipt)],capture_output=True,text=True)
        verdict=json.loads(proc.stdout);write(cache/(case+"-review.json"),verdict)
        if verdict["status"]=="invalid":raise ValueError(verdict)
        summary[case]={"status":verdict["status"],"acceptance":verdict["engineeringAcceptance"],"exitCode":proc.returncode,
          "manifestSha256":artifact(manifest),"receiptSha256":artifact(receipt),
          "axes":{r["id"]:{"status":r["status"],"issues":r["issues"]} for r in verdict["studies"]},"runs":data}
    write(HERE/("fine-review.json" if fine else "review.json"),{"pluginVersion":json.loads((plugin/"package.json").read_bytes())["version"],"cases":summary})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("plugin",type=Path);p.add_argument("cache",type=Path);p.add_argument("--fine",action="store_true");a=p.parse_args();run(a.plugin,a.cache,a.fine)
