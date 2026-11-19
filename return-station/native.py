"""Enclose each interpolated corrected feed inside the full fixed-low receiver, with spring travel."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
from registration import HERE,ROOT,model,mount,sha
sys.path.insert(0,str(ROOT/"station-supply"))
from entry_envelope import convex_hull
from dock_tolerance import separation
sys.path.insert(0,str(ROOT/"design/receiver-drives"))
import receiver_drives
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt,gp_Vec


def prism(outline,lo,hi):
    poly=BRepBuilderAPI_MakePolygon()
    for x,z in outline:poly.Add(gp_Pnt(x,lo,z))
    poly.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(poly.Wire()).Face(),gp_Vec(0,hi-lo,0)).Shape()


def run(cache,lamp,relief=False,probe=False,hold=False):
    from receiver_relief import low_relief
    robot=low_relief(lamp) if relief else model(lamp)
    source=HERE/("hold-feed.json" if hold else "probe-feed.json" if probe else "relief-feed.json" if relief else "feed.json")
    skip={"grip-L-420-3M-15","grip-R-420-3M-15","receiver-R6-drilled-for-camera"}
    if len(skip & robot.keys())!=3:raise ValueError("Intended contact inventory changed")
    boxes={k:mount.cad.bounds(s) for k,s in robot.items() if k not in skip}
    feeds=json.loads(source.read_bytes())
    traces=[]
    for row in feeds:
        if "bias-" not in row["name"]:continue
        p=cache/(row["name"]+".npy")
        if sha(p)!=row["trajectorySHA256"]:raise ValueError("Different feed trajectory")
        tr=np.load(p)
        if row["hardStop"] or row["loss"]:raise ValueError("Stop before evaluating support")
        traces.append(tr)
    # A circumscribed 48-gon encloses any tilted radial disk projected into X/Z.
    ring=[(57.15/math.cos(math.pi/48)*math.cos(2*math.pi*(i+.5)/48),57.15/math.cos(math.pi/48)*math.sin(2*math.pi*(i+.5)/48)) for i in range(48)]
    times=[119,*range(130,451,10),*range(475,951,25),1000,1100,1250,1450,1800]
    if hold:times=[1450,1650,2000,5540]
    result=[]
    for start,end in zip(times,times[1:]):
        data=[tr[(tr[:,0]>=start-.51)&(tr[:,0]<=end+.51),1:7] for tr in traces]
        arr=np.concatenate([v for v in data if len(v)])
        x,y,z,ax,ay,az=arr.T
        centers=convex_hull([(float(a),float(b)) for a,b in zip(np.r_[x+150.8125*ax,x-150.8125*ax],np.r_[z+150.8125*az,z-150.8125*az])])
        # Curvature allowance 0.025 mm on each included interpolated segment.
        pxz=convex_hull([(a+u+sx*.025,b+v+sz*.025) for a,b in centers for u,v in ring for sx,sz in ((-1,-1),(-1,1),(1,-1),(1,1))])
        side=150.8125*np.abs(ay)+57.15*np.sqrt(np.maximum(0,1-ay**2))+.025
        yy=[float(min(y-side)),float(max(y+side))]
        tools={}
        inward,outward=(.40,1.40) if relief else (1.5,1.5)
        for side in (-1,0,1):
            offsets=sorted((-side*outward,side*inward))
            shape=prism(pxz,yy[0]+offsets[0],yy[1]+offsets[1])
            tools[side]=(shape,mount.cad.bounds(shape))
        nearest=(3.,None);hits={}
        for k,b in boxes.items():
            mutable=receiver_drives.translates_with_grip(k) or k.endswith("-spring")
            side=(1 if k.startswith("grip-L-") else -1) if mutable else 0
            tool,envelope=tools[side]
            if separation(b,envelope)>nearest[0]+.001:continue
            d=BRepExtrema_DistShapeShape(tool,robot[k])
            if not d.IsDone():raise ValueError(k)
            nearest=min(nearest,(d.Value(),k),key=lambda a:a[0])
            if d.Value()<1e-6:
                volume=mount.cad.intersection(tool,robot[k])
                if volume>.0001:hits[k]=round(volume,5)
        result.append({"fromMs":start,"toMs":end,"nativeRigidLowerMm":nearest[0],"nearest":nearest[1],"outerEnvelopeOverlapMm3":hits})
    (HERE/("hold-native.json" if hold else "probe-native.json" if probe else "relief-native.json" if relief else "native.json")).write_text(json.dumps({"robotBodies":len(robot),"excludedIntended":sorted(skip),"trajectories":len(traces),"independentCartridgeInwardOutwardMm":[inward,outward],"maxTimeStepMs":.5,
        "interpolationPaddingMm":.025,"bins":result,"feedSHA256":sha(source)},indent=2)+"\n",encoding="utf8",newline="\n")
    print([(r["fromMs"],r["outerEnvelopeOverlapMm3"]) for r in result if r["outerEnvelopeOverlapMm3"]],flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("cache",type=Path);p.add_argument("lamp",type=Path)
    p.add_argument("--relief",action="store_true");p.add_argument("--probe",action="store_true");p.add_argument("--hold",action="store_true");args=p.parse_args();run(args.cache,args.lamp,args.relief or args.probe or args.hold,args.probe or args.hold,args.hold)
