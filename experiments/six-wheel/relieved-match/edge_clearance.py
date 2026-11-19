"""Enclose the free finite shell against the shelf lip and raised pad tops through field arrival."""
import json
import math
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent
HALF=150.8125
R=57.15


def distance(center,axis,corner):
    # Minimize distance from the entire centerline to x<=front, z<=top. Inflating
    # the centerline into a capsule encloses the flat-ended physical pipe.
    p=center[[0,2]]-corner
    v=axis[[0,2]]
    candidates=[-HALF,HALF]
    for j in range(2):
        if v[j]:candidates.append(float(np.clip(-p[j]/v[j],-HALF,HALF)))
    if v@v:candidates.append(float(np.clip(-(p@v)/(v@v),-HALF,HALF)))
    return min(float(np.linalg.norm(np.maximum(0.,p+s*v))) for s in candidates)-R


def run():
    summary=[]
    for piece in ("coral-A","coral-B"):
        for res in (250,500):
            row=json.loads((HERE/f"{piece}-pads-edge-{res}.json").read_bytes())
            c0=np.array(row["edgeRobotMm"])
            h=row["chassisHeadingRad"];c,s=math.cos(h),math.sin(h)
            rot=np.array([[c,s,0],[-s,c,0],[0,0,1]])
            v=1000*rot@np.array(row["velocityMps"])
            axis=rot@np.array(row["axis"])
            w=rot@np.array(row["angularRadps"])
            ratio=6*(R*R+50.8**2)/(3*(R*R+50.8**2)+4*HALF**2)
            freq=w+(ratio-1)*axis*(axis@w)
            speed=np.linalg.norm(freq);n=freq/speed
            def attained(t):
                angle=speed*t
                return c0+v*t-[0,0,4903.325*t*t],axis*math.cos(angle)+np.cross(n,axis)*math.sin(angle)+n*(n@axis)*(1-math.cos(angle))
            for part,corner in (("shelf",[165.,500.]),("sensor-and-tail-upper-envelope",[158.6,500.8])):
                blocks=[(0.,.108)];lower=1000.;sample=1000.;count=0;failed=False
                while blocks:
                    u,q=blocks.pop();t=(u+q)/2
                    center,a=attained(t);d=distance(center,a,corner)
                    sample=min(sample,d);count+=1
                    l=d-(q-u)/2*(np.linalg.norm(v)+9806.65*q+speed*HALF)
                    if l>0:
                        lower=min(lower,l)
                    elif d<=0 or q-u<1e-8:
                        lower=min(lower,l);failed=True;break
                    else:blocks.extend(((u,t),(t,q)))
                summary.append({"pipe":piece,"stepUs":res,"material":part,"clear":not failed,"continuousLowerMm":lower,"minimumSampleMm":sample,"intervals":count})
    (HERE/"free-flight-clearance.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8",newline="\n")
    print(summary,flush=True)

if __name__=="__main__":run()
