"""Run both signed station acquisitions against the same desktop sensor controller."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
import numpy as np
from model import HERE,ROOT,FlexiblePipe,coupled,read

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):p.write_text(json.dumps(d,indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")

def trial(task):
    approach,trial_name,side,settings,dt,end_ms,jdk,classes,artifacts=task
    cache=artifacts/approach/trial_name/side/f"us-{dt*1e6:g}"
    cache.mkdir(parents=True,exist_ok=True)
    pose=None if approach=="published-drop" else read(ROOT/"conditioned-match/opening.json")[side]["sourceRobotMmDeg"]
    old=coupled.CoupledPipe
    if settings is not None:
        FlexiblePipe.settings=settings
        coupled.CoupledPipe=FlexiblePipe
    job=(side,True,400000.,4.,dt,2300,48,.02,0.,1500.,(-.066,.042),.0001,True,end_ms,pose,jdk,classes,cache)
    try:result=coupled.replay(job)
    finally:coupled.CoupledPipe=old
    result.update(approach=approach,trial=trial_name,settings=settings,deadlineMs=end_ms)
    if settings is not None:
        p=FlexiblePipe.last;m=p.model
        path=cache/"land-reactions.csv"
        np.savetxt(path,np.asarray(p.samples),delimiter=",",fmt="%.11g",
            header="lip_s,FL_m,FR_m,RL_m,RR_m,guide_L_m,guide_R_m,pin_L_N,pin_R_N,bracket_FL_N,bracket_FR_N,bracket_RL_N,bracket_RR_N,damper_R_N,damper_L_N",
            comments="")
        result.update(landHistorySha256=digest(path),stiffnessScale=settings[0],guideStiffnessNPerM=settings[1],
            lashMm=1000*settings[3],pinGapMm=1000*settings[4],pinStiffnessScale=settings[5],
            maxAbsOldPinForceN=p.pin_peak.tolist(),oldPinLoadPeakClockS=p.pin_clocks.tolist(),
            bracketLandRangeMm=(p.site_bounds*1000).tolist(),guidesRelativeRangeMm=(p.guide_bounds*1000).tolist(),
            topStopContactTicks=p.guide_upper_touches.tolist(),individualStageRangeMm=(1000*p.each_guide_bounds).tolist(),damperBaseTravelMm=(p.base_peak*1000).tolist(),
            frequencyHz=(m['omega']/(2*np.pi)).tolist(),participatingMassKg={'plate':m['plateKg'],'guidedRemainder':m['guideKg'],'bridgeRemainder':m['frameKg']},
            modalMassClosure=m['modalClosure'],landInterpolationError=m['cardinalityError'])
    write(cache/"result.json",result)
    return result

DEFAULT=(1.,5e6,1.,.002,.0001,1.)
def jobs(group):
    if group=="source":
        return [(a,k,s,c,31.25e-6,1700) for a in ("published-drop","powered-dock") for s in ("leftSmall","rightSmall") for k,c in (("bracket",DEFAULT),("rigid",None))]
    if group=="refine":
        return [("powered-dock","bracket",s,DEFAULT,d,1700) for d in (15.625e-6,) for s in ("leftSmall","rightSmall")]
    if group=="series":
        return [("powered-dock","two-stages",s,(*DEFAULT,2),dt,1700) for dt in (31.25e-6,15.625e-6) for s in ("leftSmall","rightSmall")]
    if group=="limits":
        configs={"closed-guides":(1.,5e6,1.,0.,.0001,1.),"zero-pin-gap":(1.,5e6,1.,.002,0.,1.),
                 "soft-guides":(1.,1e6,1.,.002,.0001,1.),"rigid-bracket-limit":(40.,5e6,1.,.002,.0001,1.)}
        return [("powered-dock",key,s,c,31.25e-6,1700) for key,c in configs.items() for s in ("leftSmall","rightSmall")]
    raise ValueError(group)

def run(group,jdk,cache,workers):
    registry=read(ROOT/"station-landmark/feed.json")
    with tempfile.TemporaryDirectory() as folder:
        classes=Path(folder);code=classes/"PortAcquisition.java"
        code.write_text(coupled.feed.port_program(True),encoding="utf8",newline="\n")
        if digest(code)!=registry["programSha256"]:raise ValueError("Changed Java sensor program")
        subprocess.run([str(jdk/"bin/javac"),"-d",str(classes),str(code),str(ROOT/"observable-acquisition/PortConsole.java")],check=True)
        items=[(*args,jdk,classes,cache) for args in jobs(group)]
        with ProcessPoolExecutor(max_workers=workers) as pool:outputs=list(pool.map(trial,items))
    write(HERE/(group+".json"),{"registeredProgramSha256":registry["programSha256"],
        "pinnedInputs":{key:digest(ROOT/key) for key in ("carriage-support/carriage-support.step","shelf-structure/coupled.py",
            "shelf-structure/response.py","conditioned-match/opening.json","shelf-structure/coupled-mounts.json","integrated-slide/mass.json")},
        "cases":outputs})
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("group",choices=("source","refine","limits","series"));p.add_argument("jdk",type=Path);p.add_argument("artifacts",type=Path)
    p.add_argument("--workers",type=int,default=2)
    a=p.parse_args();run(a.group,a.jdk.resolve(),a.artifacts.resolve(),a.workers)
