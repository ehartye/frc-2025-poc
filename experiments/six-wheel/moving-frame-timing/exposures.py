"""Reopen the 598-body two-camera study and retain its numeric release frames for packet replay."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"moving-observer"))
import handoff as optics
from optics import camera

def build(scratch):
    prior=json.loads((ROOT/"low-observation/results.json").read_bytes())
    fixed=camera.images_for_pose(optics.register())
    reference=[g for g,_ in fixed]
    signature=hashlib.sha256((ROOT/"moving-observer/moving-bridge.step").read_bytes()).hexdigest()[:16]
    overhead=optics.load_scene(scratch/("moving-custody-"+signature+".npz"))
    if hashlib.sha256((scratch/("moving-custody-"+signature+".npz")).read_bytes()).hexdigest()!=json.loads((ROOT/"moving-observer/replay.json").read_bytes())["registration_sha256"]:
        raise ValueError("unregistered moving background")
    rows=[]
    mobile=[]
    for key,record in prior["pieces"].items():
        tr=np.load(scratch/f"{key}-trajectory.npy")
        start=math.ceil(record["truth_ms"]["place"])
        end=math.floor(record["truth_ms"]["paired-trough-first-contact"])
        for t in range(start,end+1):
            coords=[float(np.interp(t,tr[:,0],column)) for column in tr[:,1:5].T]
            reading=optics.read_pair(camera.exposure(fixed,coords[:3],coords[3]),reference,560,370)
            rows.append([key,"fixed",t,reading["status"],*[v for k in ("u_interval_mm","y_interval_mm","bottom_relative_to_shelf_mm")
                for v in reading.get(k,["",""])]])
            if t%17==0:mobile.append((key,t,coords))
    optics.optics.configure()
    for key,t,(x,y,z,yaw) in mobile:
        reading=optics.optics.fit(camera.exposure(overhead,[x-370,y,z],yaw),overhead)
        rows.append([key,"moving",t,reading["status"],*[v for k in ("x_mm","y_mm","bottom_mm") for v in reading.get(k,["",""])]])
    with (HERE/"exposures.csv").open("w",encoding="ascii",newline="") as file:
        writer=csv.writer(file,lineterminator="\n")
        writer.writerow("piece,head,capture_ms,status,x_lo,x_hi,y_lo,y_hi,bottom_lo,bottom_hi".split(","))
        writer.writerows(rows)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scratch",type=Path,required=True)
    build(p.parse_args().scratch)
