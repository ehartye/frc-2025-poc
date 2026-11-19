"""Test the real hollow tube and loaded moving frames in the original low robot and January station."""
from collections import defaultdict
from dataclasses import replace
import argparse
import json
import math
from pathlib import Path
from motion import Case,ARM,run,smooth
from gate_cad import cad,local,shapes,base,ROOT,HERE,rod,subtract,rotate,AY,interior,sha
from dock_tolerance import bounded_dock,native_dock
from native_station import parts,FIRST_SHA

def tube(x,y,z,ang,c):
    yaw=math.radians(ang)
    vec=(-math.sin(yaw),math.cos(yaw),0)
    po=(x-500*c.length*vec[0],y-500*c.length*vec[1],z)
    return subtract(rod(*po,vec,500*c.diameter,1000*c.length),rod(*po,vec,500*c.diameter-6.35,1000*c.length))

def review(c):
    state=run(c,trace=True);trace=state.pop("trace")
    phase={0:base(0),370:base(370)}
    bounds={k:{n:cad.bounds(s) for n,s in data.items()} for k,data in phase.items()}
    layer=local();boxes={k:cad.bounds(a[2]) for k,a in layer.items()}
    contacts=defaultdict(list);intended=defaultdict(float)
    last_hit=None;first_clear=None;kept=0
    times=[]
    if state["firstContactS"]:times.append(state["firstContactS"])
    ts=state["eventsS"];end=ts.get("exit",3.8)
    for row in trace:
        t,x,z,vx,vz,spin,left,right,retract=row
        # Dense 2-ms samples around contact and release and sparse samples in steady phases.
        selected=(.17<t<.29 or end-.075<t<end+.15 or int(round(t*1000))%40==0)
        if not selected:continue
        kept+=1;stroke=370 if t>=3.5 else 0
        # Evaluate the target branch at its actual fixed position. All studied issued
        # cases hold the robot/slide after release; planned retreat is a separate control.
        if retract:raise ValueError("Use stationary slide for installed review")
        x*=1000;z*=1000;y=1000*c.lateral
        part=tube(x,y,z,c.yaw,c);abs_part=cad.translated(part,(stroke,0,0))
        bp=cad.bounds(abs_part)
        clashes=[]
        for k,bb in bounds[stroke].items():
            if interior(bp,bb):
                vol=cad.intersection(abs_part,phase[stroke][k])
                if vol>.025:clashes.append((k,vol))
        rise=1000*c.hood_raise*(1-smooth((t-c.close_time)/c.close_duration))
        refs={"base":part,"L":cad.translated(part,(0,85-left*1000,0)),
              "R":cad.translated(part,(0,-85-right*1000,0)),
              "hood":rotate(part,(-ARM*1000,0,1000*c.head_z),AY,math.degrees(math.asin(rise/(1000*ARM))))}
        brefs={f:cad.bounds(s) for f,s in refs.items()}
        for k,(f,_,obj) in layer.items():
            if not interior(brefs[f],boxes[k]):continue
            vol=cad.intersection(refs[f],obj)
            if vol<=.025:continue
            if k in ("rear-bulkhead","L-537-3M-15","R-537-3M-15","hood-wheel-L","hood-wheel-R") and t<end:
                intended[k]=max(intended[k],vol)
            else:clashes.append((k,vol))
        if clashes:last_hit=t
        for k,v in clashes:contacts[k].append((t,v))
    return {"outcome":state["outcome"],"geometricSurfaceExitS":ts.get("exit"),"samples":kept,
            "permittedContactPeakMm3":dict(intended),"lastUnexpectedSampleS":last_hit,
            "unexpected":{k:{"firstS":v[0][0],"lastS":v[-1][0],"peakMm3":max(d for _,d in v)} for k,v in contacts.items()}}

def build(field):
    basecase=Case(withdraw=0.)
    cases={"center-both-gates":basecase,"long-yawed":replace(basecase,lateral=.010,normal=.010,yaw=4.,length=.307975,lag=.025),
           "short-yawed":replace(basecase,lateral=-.010,normal=-.010,yaw=-4.,length=.295275,sector=.18)}
    results={n:review(c) for n,c in cases.items()}
    station=parts(field);bb={k:cad.bounds(v) for k,v in station.items()}
    result=shapes(0,height=40);rb={k:cad.bounds(v) for k,v in result.items()}
    return {"fieldSha256":FIRST_SHA,"splitGateNativeSha256":sha(HERE/"split-gate.step"),"stationCount":len(station),
            "addedDockBound":bounded_dock(rb,bb,10),"nativeDock":[native_dock(result,station,bb,n,a,y)
                   for n,a,y in ((10,0,0),(0,-3,-10),(0,3,10))],"pipe":results}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--field",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(build(a.field),indent=2)+"\n",encoding="utf-8",newline="\n")
