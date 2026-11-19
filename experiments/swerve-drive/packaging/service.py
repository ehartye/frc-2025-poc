"""Bound continuous downward service, refining broadphase candidates against native material."""
from pathlib import Path
import sys
import numpy as np
from module_placement import HERE,SITES,read,write
from frame import j,ROOT
from fit import indexed

def xyzbox(b):
    return j.box(b[0],b[3],b[1],b[4],b[2],b[5])

def run(folder,tag):
    original=indexed(folder)
    meta=read(HERE/"frame.json")
    frame=j.cad.read_named(HERE/"swerve-frame.step",[{"id":k} for k in meta["parts"]])
    issued=HERE/"rear-electronics.json"
    rm=read(issued if issued.exists() else folder/"rear-provisional.json")
    if rm["stepSha256"]!=j.digest(HERE/"rear-electronics.step"):
        raise ValueError("Unexpected rear electronics revision")
    rear=j.cad.read_named(HERE/"rear-electronics.step",[{"id":k} for k in rm["partsMm3"]])
    envelope=read(HERE/"envelopes.json")
    offered=read(HERE/"fit.json")["downwardBroadphasePairs"][tag]
    bounds={k.split("/",1)[1]:b for k,b in envelope["cornerBoundsMm"].items() if k.startswith(tag+"/")}
    bounds.update({k:v["boundsMm"] for k,v in meta["parts"].items() if k.startswith(tag+"-")
                   and "rail-screw" not in k and "welded-thread-boss" not in k})
    obstacles={}
    removed_tie=[]
    for k in offered:
        if k.startswith("bumper-"):continue # remove both reversible bumper halves before service
        if k.startswith("deck-tie-") and k.endswith("-anchor"):
            removed_tie.append(k)
            continue
        if k in original:obstacles[k]=original[k]
        if k in frame:obstacles[k]=frame[k]
    obstacles.pop("observer-pi-and-converter-backplate",None)
    obstacles.pop("electrical-backplate",None)
    obstacles.pop("observer-pi4-service-envelope",None)
    obstacles.update(rear)
    obstacles.update({k:s for k,s in frame.items() if "welded-thread-boss" in k})
    controls={}
    radial_witness={}
    radius=envelope["movingRadiusMm"]
    for name,b in bounds.items():
        w=list(b);w[2]-=195
        for k,s in obstacles.items():
            # Tangential faces are bearing contacts; exclude zero-depth bbox intersections.
            t=j.cad.bounds(s)
            if any(min(w[i+3],t[i+3])-max(w[i],t[i])<.00001 for i in range(3)):continue
            if name in radius:
                cx,cy=SITES[tag]
                dist=np.hypot(max(t[0]-cx,cx-t[3],0),max(t[1]-cy,cy-t[4],0))
                if dist>radius[name]+.01:
                    radial_witness[k]=min(float(dist-radius[name]),radial_witness.get(k,float("inf")))
                    continue
            if name.startswith(("0-","1-")) and k=="host-drilled-swerve-deck":
                # The entire motor bounding rectangle fits either the 53 mm or 34 mm aperture.
                continue
            q=j.cad.intersection(xyzbox(w),s)
            if q>.01:controls[name+" :: "+k]=round(q,4)
    return {"corner":tag,"continuousDownMm":195,"screenedBodyBoxes":len(bounds),"fixtureCount":len(obstacles),
            "matchingTemporaryDeckAnchorRemoval":removed_tie,"lowerRadialMarginsMm":radial_witness,
            "positiveConservativeBoxesMm3":controls,"sourceKit":meta["stepSha256"],"rearCorrection":rm["stepSha256"]}
if __name__=="__main__":
    out=Path(sys.argv[1]);tag=sys.argv[2];write(out/("service-"+tag+".json"),run(out,tag))
