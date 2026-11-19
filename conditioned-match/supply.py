"""Replay the achieved station approach, carry and hold against the shared two-camera power model."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from views import load_navigation
n=load_navigation()
from transport import trajectory
from electrical import screen as electric
sys.path.insert(0,str(n.ROOT/"station-guide"))
from optics import depth,referenced,EYES,approach
from inertial import staged
from field import join

def opening(assets, side, pace=1., deadline=21.1, wheel_accel_mm_s2=420.):
    cfg=n.read(n.ROOT/"station-landmark/routes.json")["cases"][side]
    scale=np.asarray(cfg["wheelScale"])
    width=cfg["effectiveTrackMm"]
    q,wall,f,leg,t,actual,_=staged(assets/"relieved-match",scale,width,cfg["gyroBiasDegS"],cfg["gyroScaleError"],
                   cfg["startHeadingErrorDeg"],stage_mm=400,side_mm=0,pace=pace)
    _,_,bias,_=referenced(assets/"selected-optics/issued/marked")
    raw=np.load(assets/"station-guide/selected-background/stowed-robot-fixed.npz")
    bg=[np.minimum(raw[key+"_z"],depth(eye,0,3.8,40,20)) for key,eye in zip(("left","right"),EYES)]
    r=approach(q,wall,scale,width,bg,bias,side_bias=cfg["assumedStationExtrinsicErrorMm"],deadline=deadline,wheel_accel_mm_s2=wheel_accel_mm_s2)
    expected=n.read(n.ROOT/"station-guide/optics-narrow.json")["trials"][side]
    if pace == 1 and wheel_accel_mm_s2 == 420. and not np.allclose(r["finalStationMmDeg"],expected["finalStationMmDeg"],atol=1.e-7,rtol=0):
        raise ValueError("Current station controller produced a different docking pose")
    # The original source labels the integrated end of each 10 ms wheel step with the start time.
    # Put that pose at the end of its physical interval and retain the exact initial zero-speed state.
    rows=[[wall,*q,0.,0.]] + [[row[0]+.01,*row[1:]] for row in r["path"]]
    return (*join(f,leg,t,actual,rows),r["finalStationMmDeg"],r["status"],r["wallS"])

def run(assets,corrected=False,slow=False):
    plans=n.read(n.ROOT/"fixed-height-match/placements.json")["placements"]
    drive=18+plans[0]["drive"]
    results={}
    for side in ("leftSmall","rightSmall"):
        if corrected:
            opening_npz=np.load(assets/"conditioned-match"/side/"corrected-opening.npz")
            ta,pa=opening_npz["t"],opening_npz["pose"]
        else:
            ta,pa,*_=opening(assets,side)
        t1,p1=trajectory(assets,side,"slow" if slow else "corrected" if corrected else "gyro")
        if np.linalg.norm(pa[-1,:2]-p1[0,:2]) > .00001 or abs(n.corrected().short(pa[-1,2]-p1[0,2]))>1e-7:
            raise ValueError("Dock/start discontinuity")
        time=np.r_[ta,drive+t1,156.]
        pose=np.vstack((pa,p1,p1[-1]))
        pose[:,2]=np.unwrap(pose[:,2])
        grid=np.arange(31201)*.005
        actual=np.column_stack([np.interp(grid,time,x) for x in pose.T])
        np.savez_compressed(assets/"conditioned-match"/side/("slow-field-poses.npz" if slow else "corrected-field-poses.npz" if corrected else "field-poses.npz"),t=grid,pose=actual)
        labels=np.select((grid[:-1]<ta[-1],grid[:-1]<drive,grid[:-1]<drive+t1[-1]),("opening approach","station hold","loaded drive"),default="stationary")
        arcs=n.route.wheel_steps(actual)
        ev={"coral-A":{"seat":21.571672635314284+(1.40 if corrected or side=="rightSmall" else 1.42),"feed":21.10,"carry":drive}}
        # R103 limit plus maximum modeled battery and removable bumper masses. Other components share the limit.
        mass=52.16312255+6.17+7.5
        results[side]={}
        for name,pack in (("fresh",(12.4,.025)),("screened",(11.8,.027)),("weak",(11.8,.032))):
            r=electric(grid,arcs,labels,ev,mass,"conditioned",pack,assets)
            results[side][name]={k:r[k] for k in ("bodyKg","onePipeKg","commonCurrentAtMinimumA","minTerminalV","limitingWallS","limitingLeg",
                "peakMotorPhaseA","peakFrictionRequired","firstReferenceInhibitWallS","floorBelowOrUnsolvedS","limitingBranch")}
            print(side,name,results[side][name],flush=True)
    (n.HERE/("slow-power.json" if slow else "carry-power.json" if corrected else "power.json")).write_text(json.dumps(results,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);p.add_argument("--corrected",action="store_true");args=p.parse_args()
    run(args.assets,args.corrected)
