"""Keep the safety-calculation and the plant braking limits identical during the station screen."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import numpy as np
from supply import opening,n,electric

def trial(job):
    assets,side,pace,accel=job
    t,q,d,status,wall=opening(assets,side,pace=pace,wheel_accel_mm_s2=accel)
    g=np.arange(4221)*.005
    pose=np.column_stack([np.interp(g,t,x) for x in (q[:,0],q[:,1],np.unwrap(q[:,2]))])
    arcs=n.route.wheel_steps(pose)
    r=electric(g,arcs,np.full(len(arcs),"opening"),{},65.83312255,"conditioned",(11.8,.027),assets)
    row={"case":side,"pace":pace,"wheelAccelMmS2":accel,"poseMmDeg":d,"status":status,"wallS":wall,
         "firstPowerInhibitWallS":r["firstReferenceInhibitWallS"],"peakMotorA":r["peakMotorPhaseA"],"lowestV":r["minTerminalV"]}
    print(side,pace,accel,status,wall,r["firstReferenceInhibitWallS"],flush=True)
    return row

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args()
    jobs=[(a.assets,s,pace,c) for pace in (.90,.94) for c in (220,230,250,270,290,310) for s in ("leftSmall","rightSmall")]
    with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(trial,jobs))
    (n.HERE/"tuning.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf8",newline="\n")
