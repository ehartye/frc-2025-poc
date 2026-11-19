"""Check the physically reached limited station approach, field clock, and loaded electrical supply."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from supply import opening,n,electric
from field import screen,parked_peers

def run(assets):
    stock=n.read(n.ROOT/"full-match/installed-stock.json")["stock"]
    plans=n.read(n.ROOT/"fixed-height-match/placements.json")["placements"]
    f,_,team,_=n.route.route.field.scenario(plans,inboard=True)
    out={}
    for side in ("leftSmall","rightSmall"):
        t,a,delta,status,wall=opening(assets,side,pace=.94,deadline=21.1,wheel_accel_mm_s2=290.)
        grid=np.arange(0,21.105,.005)
        path=np.column_stack([np.interp(grid,t,x) for x in (a[:,0],a[:,1],np.unwrap(a[:,2]))])
        gaps=screen(f,grid,path,stock,team,grid[-1])
        arcs=n.route.wheel_steps(path)
        pack={}
        for key,volts,resistance in (("screened",11.8,.027),("weak",11.8,.032),("fresh",12.4,.025)):
            r=electric(grid,arcs,np.full(len(arcs),"station approach"),{},65.83312255,"conditioned",(volts,resistance),assets)
            pack[key]={k:r[k] for k in ("commonCurrentAtMinimumA","peakMotorPhaseA","peakFrictionRequired","minTerminalV","firstReferenceInhibitWallS","limitingWallS","limitingBranch")}
        center=path[np.searchsorted(grid,15.)];ext=.425*(abs(math.cos(center[2]))+abs(math.sin(center[2])))
        tape=(690.875-144-65.5-88)*f.IN
        path_file=assets/"conditioned-match"/side/"corrected-opening.npz";np.savez_compressed(path_file,t=grid,pose=path)
        disabled=path[(grid>=15)&(grid<=18)]
        out[side]={"stationMmDeg":delta,"sourceRobotMmDeg":[-delta[0],-delta[1],delta[2]],"status":status,"wallS":wall,
          "clearance":gaps,"stationaryPeerTo153Mm":parked_peers(team,path[-1],wall),"pathSha256":n.sha(path_file),"power":pack,
          "leaveFullFootprintAcrossTapeMm":float((center[0]-ext-tape)*1000),
          "disableTranslationMm":float(1000*np.linalg.norm(np.diff(disabled[:,:2],axis=0),axis=1).sum()),
          "disableHeadingChangeDeg":float(np.rad2deg(abs(np.diff(disabled[:,2])).sum()))}
        print(side,status,wall,delta,gaps["minimumBarrierMm"],gaps["minimumStockMm"],gaps["minimumPeerMm"],flush=True)
    (n.HERE/"opening.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);args=p.parse_args()
    run(args.assets)
