"""Measure the installed force laminates and the loaded robot's two scoring sectors."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

from cameras import HERE,ROOT,low_relief,mount,bridge,sha
from registration import parts,dock
from rigid import annulus
field=bridge.mod("occupied_reef",ROOT/"bridge-tolerance/field.py")


def run(cache,lamp,official):
    stowed=low_relief(lamp)
    placed=low_relief(lamp,370)
    sensors={k:v for k,v in stowed.items() if k.startswith("observer-moving-pad-")}
    if len(sensors)!=3:raise ValueError("Missing physical pressure sensors")
    reference=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())
    if sha(official)!=json.loads((ROOT/"sector-alignment/scene.json").read_bytes())["fieldSHA256"]:raise ValueError("field changed")
    structure=parts(official)
    boxes={k:mount.cad.bounds(v) for k,v in structure.items()}
    route=json.loads((HERE/"navigation.json").read_bytes())["cases"]
    registration={}
    for piece in ("coral-A","coral-B"):
        nav=next(r for r in route if r["pipe"]==piece and r["mode"]=="nominal")
        x,y,h=nav["actualStationMmDeg"]
        registration[piece]=dock.native_dock(stowed,structure,boxes,10-x,h,-y)
    witnesses=[]
    for key in ("coral-A","coral-B"):
        data=np.load(cache/(key+"-coupled-250.npy"))
        at=data[np.argmin(abs(data[:,1]-.155))]
        tube=annulus(at[1:4]*1000,at[4:7])
        volumes={k:mount.cad.intersection(tube,v) for k,v in sensors.items()}
        witnesses.append({"piece":key,"localTravelS":at[0],"tubeXYZmm":(1000*at[1:4]).tolist(),"positiveMaterialMm3":volumes})
    frames=json.loads((ROOT/"design/competition-package/shop-duplex/red/part-frames.json").read_bytes())
    rotation=json.loads((ROOT/"design/competition-package/shop-duplex/red/rotation-bounds.json").read_bytes())
    bounds={}
    for key,v in placed.items():
        s=np.array(mount.cad.bounds(stowed[key]))
        e=np.array(mount.cad.bounds(v))
        if key in rotation:
            r=rotation[key]["bounds_mm"];factor=.5 if frames[key]=="middle" else 0
            s=np.array([r[0],r[1],r[2]+560,r[3],r[4],r[5]+560]);e=s+np.array([370*factor,0,0,370*factor,0,0])
        # Fixed-height base and lift are stationary; all translating members move at most
        # the rack endpoint error. Reserve this bound on moving members at both ends.
        margin=15 if np.max(abs(e-s))>.001 else 0.
        slide=np.array([margin,1.5 if key.startswith("grip-") else 0,0])
        bounds[key]=field.heading_sweep(np.r_[np.minimum(e[:3],s[:3])-slide,np.maximum(e[3:],s[3:])+slide],1)
    f=field.corrected();results={}
    first=json.loads((ROOT/"fixed-height-match/native.json").read_bytes())["firstTubeToSecondDockMm"]["inflatedTubeBoxMm"]
    for piece,sector in (("coral-A",-.18),("coral-B",.22)):
        position=f.routes(sector)[1].end
        by=field.check(f,reference["stock"],bounds,field.stationary(position))
        results[piece]={k:r["lower_bound_mm"]-10-1-.526342 for k,r in by.items()}
    neighbor=min((mount.limits(b,first),k) for k,b in bounds.items())
    out={"fieldSha256":sha(official),"nativeBodies":len(placed),"sensorBoxesStowedMm":{k:mount.cad.bounds(v) for k,v in sensors.items()},
         "flatModelIntersections":witnesses,"station":registration,"minimumStockPerGroupMm":results,
         "earlierInflatedFirstTubeAtSecondMm":neighbor[0]-10-1-.526342,"nearestToFirstTube":neighbor[1]}
    (HERE/"native-register.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    print(results,neighbor,witnesses,flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for a in ("cache","lamp","official"):p.add_argument(a,type=Path)
    args=p.parse_args()
    run(args.cache,args.lamp,args.official)
