"""Observe the settled tube at the two attained stations using the fixed shop camera and the finite guides."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"return-station"))
from registration import parts,mount,camera,sha
from images import silhouette
sys.path.insert(0,str(ROOT/"stereo-custody"))
from observe import read_pair


def field_mesh(step):
    out=[]
    for key,shape in sorted(parts(step).items()):
        vs,ts=mount.original.triangulate(shape)
        out.append((key,np.asarray(vs,dtype=float).reshape(-1,3)/10,ts))
    return out


def scene(field,delta,robot):
    dx,dy,angle=delta;c,s=math.cos(math.radians(angle)),math.sin(math.radians(angle));mesh=[]
    for key,vs,ts in field:
        x=444.525-dx-vs[:,0];y=-dy-vs[:,1]
        pts=np.column_stack((c*x+s*y,-s*x+c*y,vs[:,2]))
        mesh.append([key,"station","#808080" if "/GE-" in key else "#ababab",np.rint(pts*10).astype(int).ravel().tolist(),ts])
    # Include both actual guide volumes, in robot coordinates, in the native station render.
    step=ROOT/"station-guide/narrow-face.step"
    ids=[{"id":"guide-positive-y"},{"id":"guide-negative-y"}]
    for key,shape in mount.cad.read_named(step,ids).items():
        vs,ts=mount.original.triangulate(shape)
        mesh.append([key,"guide","#a9a49c",vs,ts])
    fg=camera.images_for_pose({"parts":mesh})
    return [(np.where(fz<rz,gr,g0),np.minimum(fz,rz)) for (gr,fz),(g0,rz) in zip(fg,robot)]


def run(assets,step,fine=False):
    src=json.loads((ROOT/"station-guide/background.json").read_bytes())
    if sha(step)!=src["sourceSha256"]["registered field STEP"]:raise ValueError("Unrecognized field")
    path=assets/"station-guide/selected-background/stowed-robot-fixed.npz"
    if sha(path)!=src["backgroundSha256"]["stowed-robot-fixed"]:raise ValueError("Unregistered shop camera background")
    views=np.load(path);robot=[(views[k],views[k+"_z"]) for k in ("left","right")]
    field=field_mesh(step);out={}
    packet = "fine-ports" if fine else "ports"
    cases = ("leftSmall", "rightSmall") if fine else ("nominal", "leftSmall", "rightSmall", "shiftedHumanSource", "badRegistration")
    step_us = 15.625 if fine else 62.5
    runs = json.loads((HERE / (packet + ".json")).read_bytes())["results"]
    for case in cases:
        r = next(row for row in runs if row["case"] == case and abs(row["dtS"] * 1e6 - step_us) < 1e-8
                 and row["shelf"]["dissipation_s_m"] == 4.)
        tr = assets / "shelf-contact" / packet / f"{case}-400000-4-{step_us:g}.npy"
        if sha(tr)!=r["trajectorySha256"] or r["state"]!="IN_BAND":raise ValueError("Changed or missing port trajectory")
        raw=np.load(tr);bg=scene(field,r["sourceRobotMmDeg"],robot);empty=[g for g,_ in bg]
        frames=[]
        deadline=5460
        latest=17*((deadline-15)//17)
        for tm in range(latest-4*17,latest+1,17):
            xyz=np.array([np.interp(tm,raw[:,0],col) for col in raw[:,1:7].T])
            a=xyz[3:]/np.linalg.norm(xyz[3:])
            fit=read_pair(silhouette(bg,xyz[:3],a),empty,560,0)
            frames.append({"timeMs":tm,"receivedMs":tm+15,"first5msTickMs":5*math.ceil((tm+15)/5),**fit,"sourceCenterMm":xyz[:3].tolist(),"sourceAxis":a.tolist()})
        blank=read_pair(empty,empty,560,0)
        out[case]={"trajectorySha256":sha(tr),"frames":frames,"emptyControl":blank["status"],
                   "backgroundDigests":[hashlib.sha256(g.tobytes()+z.tobytes()).hexdigest() for g,z in bg]}
        print(case,[r["status"] for r in frames],flush=True)
    body={"deliveryAssumptionMs":15,"servicePeriodMs":5,"lastPermittedTickMs":5460,"fieldSha256":sha(step),"guideSha256":sha(ROOT/"station-guide/narrow-face.step"),
          "shopDepthSha256":sha(path),"runs":out}
    (HERE/("fine-custody.json" if fine else "custody.json")).write_text(json.dumps(body,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("assets",type=Path);parser.add_argument("field",type=Path);parser.add_argument("--fine",action="store_true");a=parser.parse_args();run(a.assets,a.field,a.fine)
