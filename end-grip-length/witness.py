"""Compare authentic finite pipe walls with the fixed-low robot and moving candidate jaws."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
from grip_study import Trial, work, HERE, ROOT
from cassette import built, cad, tube, touch, originals, finalize_step

def check_case(case,samples,lag=0.):
    fixed,move=built(0,0,index=case.indexed_center)
    objects=[(key,body,0 if key in fixed else 1 if key.split("-")[1]=="L" else -1)
             for key,body in (fixed|move).items() if not key.endswith("420-3M-15") and key!="receiver-R6-ported"]
    boxes={k:cad.bounds(s) for k,s,_ in objects}
    times=[t for t,*_ in samples]
    idx={min(range(len(samples)),key=lambda i:abs(times[i]-t))
         for t in [times[0]+k*.005 for k in range(94)]+[times[0]+k*.04 for k in range(12,39)]}
    idx.add(min(range(len(samples)),key=lambda i:samples[i][1]))
    hits={}
    for i in sorted(idx):
        t,x,z,*q=samples[i]
        j=max(0,bisect.bisect_right(times,t-lag)-1)
        jaw=samples[j][3:]
        # Left jaw is index 1 of the simulated (-1,+1) ordering.
        byside={0:0,1:jaw[1],-1:-jaw[0]}
        for side in (0,1,-1):
            shift=byside[side]
            piece=tube(x,case.side-shift,z,case.yaw,301.625+case.cut)
            pb=cad.bounds(piece)
            for k,body,b in objects:
                if b!=side:continue
                sh=touch(piece,body,pb,boxes[k])
                if sh>.02:
                    h=hits.setdefault(k,{"peakMm3":0.,"firstLipMs":1000*(t-times[0]),"atPeakMm":[]})
                    if sh>h["peakMm3"]:h.update(peakMm3=sh,atPeakMm=[x,case.side,z,shift])
    return {"distinctSamples":len(idx),"carriageLagMs":1000*lag,"positive":hits}

def build(jdk):
    force=(1.97+.32)*4.4482216152605
    cfg=Trial("center",spring=force,cassette=True)
    cases=[cfg,Trial("long",6.35,1.,.25,5.,spring=force,cassette=True),
           Trial("short",-6.35,-1.,-.25,-5.,spring=force,cassette=True),
           Trial("weak",spring=force,cassette=True,friction=.08)]
    cases.extend(Trial("index-"+label,cut,side,yaw,depth,spring=force,cassette=True,pad=.5,
                       indexed_center=cut/2,open_until_ms=0) for label,cut,side,yaw,depth in (
                       ("short",-6.35,0,0,0),("center",0,0,0,0),("long",6.35,0,0,0),
                       ("short-side",-6.35,1,.25,5),("long-side",6.35,-1,-.25,-5)))
    cases.append(Trial("timed-open",6.35,1.,.25,5.,spring=force,cassette=True,open_until_ms=200))
    runs=work(jdk,cases,keep=True)
    results={}
    for case,row in zip(cases,runs):
        samples=row["mechanical"]["samples"]
        results[case.name]=[check_case(case,samples,lag=d) for d in ([0.,.010] if case.name=="long" else [0.])]
    return results

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--jdk",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(build(a.jdk),indent=2)+"\n",encoding="utf-8",newline="\n")
