"""Screen both inner rails through the loaded cam cycle and each possible dwell release."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from native_score import ROOT,SIX,LIP,SurfaceTrace,digest,read,release_control


def run(trace_path,receipt_path):
    receipt=read(receipt_path)
    trace=SurfaceTrace(trace_path,receipt["traceSha256"])
    appearances=read(ROOT/"cad/full-swerve-r4/appearances.json")
    rails=[r for r in appearances["solids"] if r["id"] in ("drilled-slide-L-inner","drilled-slide-R-inner")]
    original=read(Path(__file__).with_name("native.json"))
    nav=read(SIX/"relieved-match/navigation.json")["cases"]
    result={}
    for piece in ("coral-A","coral-B"):
        old=original["release"][piece]
        # The source CAD comparison uses only these three registered wall planes.
        front,back=old["wallXFromLipMm"]
        bounds=[LIP+front,0,0,LIP+back,old["wallTopMm"],0]
        dx,_,yaw=next(r["finalBodyErrorMmDeg"] for r in nav if r["pipe"]==piece and r["mode"]=="nominal")
        c,s=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
        phi=trace.q[:,5]
        rail_min=float("inf")
        for row in rails:
            bx,by,bz,ex,ey,ez=row["boundsMm"]
            from itertools import product
            for x,y,z in product((bx,ex),(by,ey),(bz,ez)):
                local_x=np.cos(phi)*x-np.sin(phi)*(z-467)+1000*trace.stroke
                rail_min=min(rail_min,float(np.min(490+front-dx+42-c*local_x+s*y)))
        dwell=np.flatnonzero((trace.time>=3)&(trace.time<=3.3))
        margins=[]
        for i in dwell:
            flight=release_control(piece,bounds,trace,trace.time[i])
            case=next(r for r in flight["cases"] if r["setbackMm"]==42)
            margins.append(case["farFaceMarginMm"])
        result[piece]=dict(addedSetbackMm=42,railForeAftSeparatingPlaneMinMm=rail_min,
                           releaseWindowS=[float(trace.time[dwell[0]]),float(trace.time[dwell[-1]])],
                           cases=len(dwell),pipeFarFaceMarginMm=[min(margins),max(margins)],
                           worstReleaseS=float(trace.time[dwell[np.argmin(margins)]]))
    return dict(trajectorySha256=digest(trace_path),candidateStepSha256=receipt["camStepSha256"],
                issuedFixture=receipt["issuedFixture"],samples=len(trace.time),cases=result)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("trace","receipt","output"):
        p.add_argument(key,type=Path)
    args=p.parse_args()
    args.output.write_text(json.dumps(run(args.trace,args.receipt),indent=2,allow_nan=False)+"\n",encoding="utf8")
