"""Replay both issued trajectories at consecutive exposure and application clocks."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import optics
from optics import ROOT,HERE,camera

def digest(f):return hashlib.sha256(f.read_bytes()).hexdigest()

def load_scene(file):
    with np.load(file) as values:
        return [(values[k],values[k+"_z"]) for k in ("left","right")]

def read_scene(background,position):
    pair=camera.exposure(background,position[:3],position[3])
    return optics.fit(pair,background)

def run(scratch):
    source=json.loads((scratch/"source.json").read_bytes())
    primary=json.loads((ROOT/"low-observation/results.json").read_bytes())
    if source["sources"]!=primary["inputs"]:raise ValueError("changed source cache")
    optics.configure()
    bgpath=scratch/("moving-custody-"+digest(HERE/"moving-bridge.step")[:16]+".npz")
    if not bgpath.exists():
        bg=camera.images_for_pose(optics.moving_mesh())
        np.savez_compressed(bgpath,left=bg[0][0],left_z=bg[0][1],right=bg[1][0],right_z=bg[1][1])
    bg=load_scene(bgpath)
    rows=[]
    report={"registration_sha256":digest(bgpath),"cases":{}}
    for key,record in source["pieces"].items():
        tr=np.load(scratch/f"{key}-trajectory.npy")
        if abs(tr[-1,0]-record["truth"]["paired-trough-first-contact"])>.001:raise ValueError("trajectory clock changed")
        start=record["first_in_band_ms"]
        release=record["truth"]["place"]
        counts={}
        misses=[]
        peak_error=0
        bounds=[]
        cache={}
        def at(t):
            x,y,z,yaw,s=(np.interp(t,tr[:,0],a) for a in tr[:,1:].T)
            return (x-s,y,z,yaw)
        for t in range(17*(int(start)//17),int(release)+17,17):
            state=at(t)
            k=tuple(round(v,6) for v in state)
            if k not in cache:cache[k]=read_scene(bg,k)
            fit=cache[k]
            status=fit["status"]
            counts[status]=counts.get(status,0)+1
            if status!="inside":misses.append(t)
            for j,name in enumerate(("x_mm","y_mm","bottom_mm")):
                if name not in fit:continue
                true=state[j]-(500+camera.R if j==2 else 0)
                lo,hi=fit[name]
                peak_error=max(peak_error,lo-true,true-hi)
            rows.append([key,t,t+15,status,*[round(float(x),4) for name in ("x_mm","y_mm","bottom_mm") for x in fit.get(name,[]) ]])
            if status=="inside":
                bounds.append([v for n in ("x_mm","y_mm","bottom_mm") for v in fit[n]])
        report["cases"][key]={"first_ms":17*(int(start)//17),"release_ms":release,"counts":counts,"failed_ms":misses,
            "maximum_truth_outside_inferred_bounds_mm":peak_error,
            "aggregate_minmax_mm":[[float(min(b[i] for b in bounds)),float(max(b[i+1] for b in bounds))] for i in (0,2,4)]}
    trials={}
    for name,position in {"seated":[80,0,557.15,0],"lost-forward":[155,0,557.15,0],"lost-low":[80,0,523,0],
                          "laterally-shifted":[80,15,557.15,0],"yaw-exceeded":[80,0,557.15,5]}.items():
        trials[name]=read_scene(bg,position)
    pair=camera.exposure(bg,None)
    trials["empty"]=optics.fit(pair,bg)
    pair=camera.exposure(bg,[80,0,557.15],0)
    trials["one-camera"]=optics.fit([pair[0],bg[1][0]],bg)
    trials["three-pixel"]=optics.fit(pair,bg,3)
    rectangles=[]
    for image,(empty,_) in zip(pair,bg):
        yy,xx=np.where(np.abs(image.astype(int)-empty.astype(int))>36)
        fake=empty.copy()
        fake[yy.min():yy.max()+1,xx.min():xx.max()+1]=220
        rectangles.append(fake)
    trials["flat-matched-rectangle"]=optics.fit(rectangles,bg)
    report["counterexamples"]=trials
    with (HERE/"frames.csv").open("w",encoding="ascii",newline="") as file:
        w=csv.writer(file,lineterminator="\n")
        w.writerow("piece,captured_ms,delivered_ms,status,xlo_mm,xhi_mm,ylo_mm,yhi_mm,bottom_lo_mm,bottom_hi_mm".split(","))
        w.writerows(rows)
    report["frame_sha256"]=digest(HERE/"frames.csv")
    report["primary_sha256"]=digest(ROOT/"low-observation/results.json")
    (HERE/"replay.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    run(p.parse_args().scratch)
