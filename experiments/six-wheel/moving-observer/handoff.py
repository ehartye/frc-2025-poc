"""Joint imaging of the same unmarked tube when the second shelf reaches its fixed placement pose."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from optics import HERE,ROOT,camera
from mount import parts,pose_layers,original,hardware
import sys
sys.path.insert(0,str(HERE))
from replay import load_scene
from observe import read_pair
import optics

def register():
    kit,_,bores=parts()
    layers,group,colors,_=pose_layers(560,370,"red")
    robot=original.flatten(layers)
    machined,added=hardware(robot,height=560)
    robot.update(machined);robot.update(added)
    for name,v in kit.items():
        robot["added-"+name]=original.native.translated(v,(370,0,0))
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    for hole in bores:
        drill=original.native.translated(original.native.brep(hole),(370,0,0))
        robot["receiver-R6-ported"]=BRepAlgoAPI_Cut(robot["receiver-R6-ported"],drill).Shape()
    mesh=[]
    for n,shape in sorted(robot.items()):
        v,t=original.triangulate(shape)
        mesh.append([n,group.get(n,"Mount"),colors.get(n,"#737d85"),v,t])
    return {"parts":mesh}


def run(scratch):
    if not np.isclose(camera.C,math.cos(math.radians(26))):raise ValueError("fixed optics required first")
    prior=json.loads((ROOT/"low-observation/results.json").read_bytes())
    fixed=camera.images_for_pose(register())
    fixed_empty=[g for g,_ in fixed]
    signature=hashlib.sha256((HERE/"moving-bridge.step").read_bytes()).hexdigest()[:16]
    overhead=load_scene(scratch/("moving-custody-"+signature+".npz"))
    dataset={}
    for key,source in prior["pieces"].items():
        trace=np.load(scratch/f"{key}-trajectory.npy")
        release=source["truth_ms"]["place"]
        target=source["truth_ms"]["paired-trough-first-contact"]
        frames={}
        # Evaluate every simulated camera phase independently. All observations precede first target contact.
        for t in range(math.ceil(release),math.floor(target)+1):
            coords=[float(np.interp(t,trace[:,0],col)) for col in trace[:,1:5].T]
            obs=read_pair(camera.exposure(fixed,coords[:3],coords[3]),fixed_empty,560,370)
            frames[t]=[coords,obs]
        dataset[key]=frames
    optics.configure()
    result={}
    for key,frames in dataset.items():
        source=prior["pieces"][key]
        outcomes=[]
        for t,(pos,obs) in frames.items():
            relative=[pos[0]-370,pos[1],pos[2]]
            moving=optics.fit(camera.exposure(overhead,relative,pos[3]),overhead)
            frames[t]=(obs,moving)
        for phase in range(17):
            streak=0
            joined=None
            missed=False
            previous=None
            earliest=None
            for t,(obs,moving) in frames.items():
                if t%17!=phase:continue
                intervals=[obs.get(n) for n in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm")]
                # The shelf camera keeps the phase-zero stream used throughout transport.
                # Pair only a previously captured shelf frame, with no future same-piece information.
                shelf_time=t-t%17
                if shelf_time not in frames:continue
                moving=frames[shelf_time][1]
                mobile=[moving.get(n) for n in ("x_mm","y_mm","bottom_mm")]
                has_fit=all(v is not None for v in intervals)
                overlap=has_fit and all(b is not None and max(a[0],b[0])<=min(a[1],b[1]) for a,b in zip(intervals,mobile))
                if joined is None:
                    streak=streak+1 if overlap and moving["status"]=="inside" else 0
                    if streak>=3:joined=t
                if joined is not None:
                    # Whole optical boxes must admit a continuous track at <= 2 m/s in each axis.
                    if not has_fit or (previous is not None and any(max(0,a[0]-b[1],b[0]-a[1])>34
                                                                for a,b in zip(intervals,previous))):
                        missed=True
                    if has_fit:
                        angle=math.radians(1)
                        radius=(camera.R+.5)*math.cos(angle)+(camera.HALF+.25)*math.sin(angle)
                        if not missed and earliest is None and 370+intervals[0][0]-radius-536>=1:earliest=t
                previous=intervals if has_fit else None
            prior_lower=math.floor(source["ideal_prior_pad_ms"]-3)
            tick=20*math.ceil((earliest+15)/20) if earliest is not None else None
            timely=tick is not None and tick-earliest<=29 and tick-prior_lower+10<=150
            outcomes.append({"phase_ms":phase,"overlap_joined_ms":joined,"fixed_track_lost":missed,
                             "first_forward_ms":earliest,"first_tick_ms":tick,"timely":timely})
        result[key]=outcomes
    (HERE/"handoff.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    run(p.parse_args().scratch)
