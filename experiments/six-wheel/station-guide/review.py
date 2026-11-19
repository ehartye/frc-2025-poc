"""Use the installed contact checker on the attained poses, with separate guide and belt refinements."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
import json
from pathlib import Path
import subprocess
import sys

from guide import HERE, ROOT, Guide, GuidedPipe, feed
sys.path.insert(0,str(ROOT/"station-feed-margin"))
from refinement import artifact, OUTCOME, WINDOW, report, sha, supported, write


SELECTED = Guide(throat_mm=-.3, stiffness_n_mm=10, damping_n_s_m=200, upper_mm=850, x_width_mm=40, x_offset_mm=20)
RESOLUTIONS = {"dt": [.0005,.00025,.000125,.0000625],
               "sectors": [16,32,64], "radial_bins": [2,4,8], "strips": [5,10,20]}
BASE = {"dt":.00025,"sectors":32,"radial_bins":2,"strips":5}


def plan(case):
    import platform
    import numpy as np
    optics=json.loads((HERE/"optics-narrow.json").read_bytes())
    n,y,h=optics["trials"][case]["finalStationMmDeg"]
    delta=[-n,-y,h]; pipe=GuidedPipe(delta,0,SELECTED)
    fixed={k:v for k,v in asdict(SELECTED).items() if k not in BASE}
    cartridge=asdict(feed.cfg);del cartridge["slices"]
    law={"guide":fixed, "cartridge":cartridge, "feedModel":artifact(ROOT/"return-station/feed.py"),
         "guideModel":artifact(HERE/"guide.py"), "inwardMm":pipe.inward,"outwardMm":pipe.outward,"preloadN":pipe.preload}
    provenance={"scenario":{"case":case,"robotMmDeg":delta,"source":artifact(HERE/"optics-narrow.json")},
                "constitutiveLaw":law,"inputs":{"beltMps":.65,"sourceLateralMm":0},
                "initialState":{"axis":pipe.a.tolist(),"p":pipe.p.tolist(),"v":pipe.v.tolist(),
                                "angularMomentum":pipe.l.tolist(),"beltMps":.65},
                "runtime":{"python":platform.python_version(),"numpy":np.__version__,"review":artifact(Path(__file__))},
                "outcomeDefinition":OUTCOME}
    doc={"schema":"agent-frc/contact-convergence-manifest@1","comparisonPins":{k:sha(v) for k,v in provenance.items()},
         "windowEndSeconds":WINDOW,"metrics":[{"id":"peak-outward-travel","units":"mm","absoluteTolerance":.05,
         "definition":"Maximum positive end-cartridge travel from the chute lip through 330 ms."}],"studies":[]}
    jobs=[]
    for key,values in RESOLUTIONS.items():
        held={k:v for k,v in BASE.items() if k!=key};levels=[]
        for i,num in enumerate(values):
            run=f"{key}-{i+1}";cfg={**held,key:num}
            levels.append({"runId":run,"resolution":[num],"modelSha256":sha(law),"settingsSha256":sha(cfg)})
            jobs.append((case,delta,run,cfg))
        doc["studies"].append({"id":key,"axis":"timestep" if key=="dt" else "contact-geometry",
                               "resolutionParameters":[key],"fixedSettingsSha256":sha(held),"levels":levels})
    return provenance,doc,jobs


def simulate(job):
    case,delta,run,cfg=job
    old=feed.cfg
    try:
        feed.cfg=replace(old,slices=cfg["strips"])
        guide=replace(SELECTED,sectors=cfg["sectors"],radial_bins=cfg["radial_bins"])
        p=GuidedPipe(delta,0,guide);p.belt=.65;last_free=0.
        while WINDOW-p.time>1.e-11 and not p.hard and not p.loss:
            p.moves(.65,min(cfg["dt"],WINDOW-p.time))
            if not supported(p):last_free=p.time
        return {"case":case,"run":run,"endSeconds":p.time,"firstStop":p.hard,"loss":p.loss,
                "observedPeakMm":p.peak,"lastUnsupportedSeconds":last_free,"peakGuideDeflectionMm":p.max_local_compression_mm}
    finally:
        feed.cfg=old


def execute(plugin,cache):
    cache.mkdir(parents=True,exist_ok=True)
    docs={};jobs=[];summary={}
    for case in ("leftSmall","rightSmall","nominal"):
        provenance,doc,cases=plan(case);docs[case]=doc;jobs.extend(cases)
        write(cache/(case+"-settings.json"),provenance);write(cache/(case+"-manifest.json"),doc)
    with ProcessPoolExecutor(max_workers=5) as pool:rows=list(pool.map(simulate,jobs))
    for case,doc in docs.items():
        subset=[r for r in rows if r["case"]==case]
        observed=cache/(case+"-receipt.json");write(observed,report(subset,doc))
        manifest=cache/(case+"-manifest.json")
        c=subprocess.run(["node",str(plugin/"scripts/check-contact-convergence.mjs"),str(manifest),str(observed)],text=True,capture_output=True)
        result=json.loads(c.stdout)
        if result["status"]=="invalid":raise ValueError(result)
        write(cache/(case+"-review.json"),result)
        summary[case]={"checkerExit":c.returncode,"status":result["status"],"acceptance":result["engineeringAcceptance"],
                       "manifestSha256":artifact(manifest),"receiptSha256":artifact(observed),
                       "axes":{r["id"]:r["status"] for r in result["studies"]},"runs":subset}
    write(HERE/"review.json",{"pluginVersion":json.loads((plugin/"package.json").read_bytes())["version"],"cases":summary})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("plugin",type=Path);parser.add_argument("cache",type=Path)
    args=parser.parse_args();execute(args.plugin,args.cache)
