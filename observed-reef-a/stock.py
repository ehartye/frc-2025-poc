"""Sweep finite installed stock against registered stowed solids with placement error."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

from retention import HERE, ROOT, nav, path_for
from field_screen import DRIVE
from field import screen


def check(assets):
    dims=nav.read(HERE/"bounds.json")
    names=list(dims["boxesMm"])
    limits=np.array([dims["boxesMm"][name] for name in names])
    # Include the entire seated-pipe enclosure in both field and body accounting.
    limits=np.vstack((limits,nav.read(ROOT/"sector-alignment/scene.json")["carryBoxMm"]))
    names.append("carried-pipe-enclosure")
    f=nav.route.route.field.scenario(nav.read(ROOT/"fixed-height-match/placements.json")["placements"],inboard=True)[0]
    stock=nav.read(ROOT/"full-match/installed-stock.json")
    limit=stock["stock"]["NAUO201"]
    x,z,y=limit["native_center_mm"]
    wx,wy=f.field_from_native(x/25.4,y/25.4)
    radius=stock["ball_radius_mm"]
    # A rigid point's planar chord under any 1-degree yaw is bounded by its own
    # radius from the chassis origin. Include 10 mm isotropic pose displacement.
    radial=np.hypot(np.maximum(abs(limits[:,0]),abs(limits[:,3])),np.maximum(abs(limits[:,1]),abs(limits[:,4])))
    allowance=10+2*radial*math.sin(math.radians(.5))
    out={}
    for side in ("leftSmall","rightSmall"):
        t,p,_=path_for(assets,side,"nominal")
        start=np.searchsorted(t,t[-1]-3.276)
        p=p[start:];t=t[start:]
        # Include center and heading motion across the interval from this sample.
        arc=1000*np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
        dt=np.r_[arc+.5,0.]
        angle=np.r_[abs(np.diff(np.unwrap(p[:,2]))),0.]
        program=nav.route.route.field.scenario(nav.read(ROOT/"fixed-height-match/placements.json")["placements"],inboard=True)[2]
        other=screen(f,DRIVE+t,p,stock["stock"],program,DRIVE+t[-1])
        stowed=10+2*math.hypot(425,425)*math.sin(math.radians(.5))
        best=(math.inf,None,None)
        for i,(qx,qy,h) in enumerate(p):
            c,s=math.cos(h),math.sin(h)
            cx,cy=1000*(wx-qx),1000*(wy-qy)
            here=np.array([c*cx+s*cy,-s*cx+c*cy,z])
            sep=np.maximum(np.maximum(limits[:,:3]-here,here-limits[:,3:]),0)
            distance=np.linalg.norm(sep,axis=1)-radius-allowance-dt[i]-radial*angle[i]
            j=np.argmin(distance)
            if distance[j]<best[0]:
                best=(float(distance[j]),names[j],float(DRIVE+t[i]))
        out[side]={"registeredBodyCount":dims["nativeBodyCount"],"stockId":"NAUO201","sphereRadiusMm":radius,
            "includedCableBounds":dims["optionalCableEnvelopeCount"],"minBoundWith10Mm1DegMm":best[0],
            "otherStockLowerWith10Mm1DegMm":min(v for k,v in other["stockMm"].items() if k!="NAUO201")-stowed,
            "barrierLowerWith10Mm1DegMm":other["minimumBarrierMm"]-stowed,
            "peerLowerWith10Mm1DegMm":other["minimumPeerMm"]-stowed,
            "limitingBody":best[1],"wallS":best[2],"sourceBoundsSha256":nav.sha(HERE/"bounds.json")}
    return out


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("assets",type=Path);a=p.parse_args()
    (HERE/"stock.json").write_text(json.dumps(check(a.assets),indent=2)+"\n",encoding="utf8",newline="\n")
