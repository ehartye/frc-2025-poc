"""Bound the two quantified end drives on the actual in-route pipe trajectories."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
from size_drives import UP5, R_GRIP, motor

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def evaluate(assets):
    # The contact cell weights sum to at most one. The complete end therefore
    # transmits at most mu times the greatest possible unclipped local normal.
    speed_limit=.25
    force=.4*(11.5+2*1.4+37.52*speed_limit)+.4
    cases={}
    for path in sorted(HERE.glob("*cargo-nominal-active*.json")):
        rows=[]
        for trial in json.loads(path.read_bytes()):
            trajectory=assets/"observed-reef-a"/(trial["id"]+".npy")
            if digest(trajectory)!=trial["trajectorySha256"]:
                raise ValueError("Custody motion changed")
            q=np.load(trajectory,mmap_mode="r")
            radius=math.hypot(.1508125,.05715)
            normal_speed=np.abs(q[:,8])+radius*np.linalg.norm(q[:,10:13],axis=1)
            rows.append({"case":trial["wheelCase"],"trajectorySha256":trial["trajectorySha256"],
                         "maxSampledClosureBoundMps":float(normal_speed.max()),
                         "peakCartridgeMm":trial["maxCartridgeMm"],
                         "insideAnalyticalInputsAtSamples":bool(normal_speed.max()<=speed_limit and trial["maxCartridgeMm"]<=1.4)})
        cases[path.stem]=rows
    return {"assumedContinuousClosureMaxMps":speed_limit,"assumedRotorInertiaKgM2":7e-6,"upperForceIncludingDragN":force,
            "preloadN":11.5,"travelLimitMm":1.4,"frictionCoefficient":.4,"viscousNsPerM":37.52,"gearRatio":UP5,
            "samplePeriodMs":1,"motorSourceSha256":digest(ROOT/"design/receiver-drives/size_drives.py"),
            "contactSourceSha256":digest(ROOT/"return-station/feed.py"),
            "driveAt006Mps":{str(a):motor(force,.006,R_GRIP,UP5,.80,a,7e-6,8) for a in (2.4,26.25)},
            "trajectories":cases}

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets",type=Path)
    a=p.parse_args()
    (HERE/"effort.json").write_text(json.dumps(evaluate(a.assets),indent=2)+"\n",encoding="utf8",newline="\n")
