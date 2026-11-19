"""Test actual annular wall against actual rigid cartridges at their admitted unloaded positions."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from registration import HERE,ROOT,model,mount,sha
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
from receiver_drives import translates_with_grip
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt


def annulus(pos,axis):
    a=axis/np.linalg.norm(axis);start=pos-150.8125*a
    origin=gp_Ax2(gp_Pnt(*start),gp_Dir(*a))
    outer=BRepPrimAPI_MakeCylinder(origin,57.15,301.625).Shape()
    inner=BRepPrimAPI_MakeCylinder(origin,50.8,301.625).Shape()
    return BRepAlgoAPI_Cut(outer,inner).Shape()


def run(cache,lamp):
    robot=model(lamp)
    feeds=json.loads((HERE/"feed.json").read_bytes())
    possible={k for b in json.loads((HERE/"native.json").read_bytes())["bins"] for k in b["outerEnvelopeOverlapMm3"]}
    out=[]
    for offset in (-1.5,0.,1.5):
        parts={}
        for key in possible:
            shape=robot[key]
            sign=1 if key.startswith("grip-L-") else -1
            if translates_with_grip(key):shape=mount.cad.translated(shape,(0,sign*offset,0))
            parts[key]=(shape,mount.cad.bounds(shape))
        for rec in feeds:
            name=rec["name"]
            if not name.endswith("-250"):continue
            tr=np.load(cache/(name+".npy"))
            first=min(x for x in rec["firstBeltSeconds"] if x is not None)
            cutoff=119+first*1000
            best=None
            for tm in np.r_[np.arange(305,cutoff,1),cutoff-.25]:
                state=np.array([np.interp(tm,tr[:,0],c) for c in tr[:,1:7].T])
                tube=annulus(state[:3],state[3:])
                box=mount.cad.bounds(tube)
                hits={}
                for key,(s,bb) in parts.items():
                    if not mount.cad.occupied(box,bb):continue
                    volume=mount.cad.intersection(tube,s)
                    if volume>.01:hits[key]=round(volume,5)
                if hits:best={"captureMs":tm,"beforeModeledWorkingBeltMs":cutoff-tm,"pipePositionMm":state[:3].tolist(),"pipeAxis":state[3:].tolist(),"materialIntersectionMm3":hits};break
            out.append({"case":name,"unloadedOutwardTravelMm":offset,"earliestSampledHit":best})
            print(name,offset,best and (round(best["captureMs"],2),best["materialIntersectionMm3"]),flush=True)
    (HERE/"rigid.json").write_text(json.dumps({"feed":sha(HERE/"feed.json"),"testedIDs":sorted(possible),"cases":out},indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("lamp",type=Path)
    a=p.parse_args();run(a.cache,a.lamp)
