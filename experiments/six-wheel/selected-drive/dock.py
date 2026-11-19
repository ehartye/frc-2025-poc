"""Challenge the actual fixed bumper with the unimpeded encoder path at the first station."""
import argparse
import json
import math
from pathlib import Path
import sys

from route import HERE,ROOT,integrate,samples,sha
# The historical field and CAD studies each use a different local rehearse module.
# The route retains its imported field object; load the CAD registration separately.
sys.modules.pop("rehearse",None)
sys.path.insert(0,str(ROOT/"axle-retention"))
from measure import cad,read_shop,SHOP_STEP
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import parts,FIRST_SHA
from dock_tolerance import native_dock

def run(artifacts,field):
    shapes,_=read_shop(artifacts)
    bumper={k:v for k,v in shapes.items() if k.startswith("bumper-cover-front-")}
    if len(bumper)!=1:raise ValueError("Front bumper body ambiguous")
    station=parts(field);boxes={n:cad.bounds(v) for n,v in station.items()}
    t,pose,arcs,_,_,f,legs=samples(artifacts/"relieved-match","nominal")
    index=next(i for i,tick in enumerate(t) if tick>=legs[0][2].duration)
    cases={}
    for name,scales,track in (("nominal",(1.,1.),.580),("bias",(.998,1.002),.580),("track",(1.,1.),.582)):
        positions=integrate(pose[0],arcs,scales,track)
        witness=[]
        for i in range(index,-1,-5):
            if t[i]<legs[0][2].duration-.9:break
            x,y=f.rot((positions[i,0]-f.DOCK[0],positions[i,1]-f.DOCK[1]),-f.NORMAL)
            yaw=math.degrees(f.short(positions[i,2]-pose[index,2]))
            row=native_dock(bumper,station,boxes,10+1000*x,yaw,1000*y)
            row["wallS"]=float(t[i]);witness.append(row)
            if row["minimum_native_mm"]>5:break
        cases[name]={"endpoint":witness[0],"firstSampleWithPositiveVolume":next((r for r in reversed(witness) if r["intersections_mm3"]),None),
                     "lastSampleBeforeContact":next((r for r in witness if not r["intersections_mm3"]),None)}
    result={"frontBody":next(iter(bumper)),"shopSHA256":SHOP_STEP,"stationSHA256":FIRST_SHA,
            "unimpededKinematicCounterfactual":True,"cases":cases}
    (HERE/"dock.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("artifacts",type=Path);cli.add_argument("field",type=Path)
    a=cli.parse_args();run(a.artifacts,a.field)
