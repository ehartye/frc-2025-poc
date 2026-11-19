"""Render finite guide obstructions on the corrected stowed stereo heads and rerun the station controller."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-landmark"))
from panel import referenced, EYES, FOCAL_PX, PITCH
from drive import approach


def overlap(origin, direction, lo, hi, near, far):
    # Ray slab intersection, including parallel rays fully inside a slab.
    safe=np.where(np.abs(direction)>1.e-12,direction,1.)
    a=(lo-origin)/safe
    b=(hi-origin)/safe
    parallel=np.abs(direction)<=1.e-12
    outside=parallel&((origin<lo)|(origin>hi))
    return np.maximum(near,np.where(outside,np.inf,np.where(parallel,-np.inf,np.minimum(a,b)))), np.minimum(far,np.where(parallel,np.inf,np.maximum(a,b)))


def depth(eye, front, rear, width=80., x_offset=0.):
    v,u=np.mgrid[:480,:640]
    q=(240-v)/FOCAL_PX
    dx=math.cos(PITCH)+q*math.sin(PITCH)
    dy=-(u-320)/FOCAL_PX
    dz=-math.sin(PITCH)+q*math.cos(PITCH)
    result=np.full_like(dx,np.inf)
    for side in (-1,1):
        for low,high,slope in ((625.,770.,0.),(770.,850.,.038)):
            n,f=overlap(970.,dz,low,high,0.,np.inf)
            n,f=overlap(-315.-.46*970,dx-.46*dz,145.+x_offset-width/2-.46*625,145.+x_offset+width/2-.46*625,n,f)
            y=150.8125-.3-slope*770
            n,f=overlap(side*eye-slope*970,side*dy-slope*dz,y+front,y+rear,n,f)
            result=np.minimum(result,np.where(n<=f,n,np.inf))
    return result


def run(artifacts, width=80., x_offset=0., output=None, final=False):
    scenes=artifacts/"selected-optics/issued/marked"
    body,_,bias,_=referenced(scenes)
    image_path=scenes.parent/"station-overlay/stowed-robot-fixed.npz"
    if final:
        image_path=artifacts/"station-guide/selected-background/stowed-robot-fixed.npz"
        raw=np.load(image_path)
        body=[raw[k+"_z"] for k in ("left","right")]
    add=[depth(e,0,3.8,width,x_offset) for e in EYES]
    covered=[np.minimum(d,z) for d,z in zip(add,body)]
    routes=json.loads((ROOT/"station-landmark/routes.json").read_bytes())["cases"]
    result={}
    if final:
        sys.path.insert(0,str(ROOT/"station-landmark"))
        from field import join,screen
        from inertial import staged
        from drive import route
        stock=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())["stock"]
        placement=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
        _,_,team,_=route.route.field.scenario(placement,inboard=True)
    names=("nominal","leftSmall","rightSmall","trackOnly","bothLarge","bothSmall","crossedDrift","badRegistration") if final else ("nominal","leftSmall","rightSmall","badRegistration")
    for name in names:
        case=routes[name]
        n,y,h=case["stationHandoffMmDeg"]
        r=approach(np.array([n,y,math.radians(h)]),case["handoffWallS"],
                   np.array(case["wheelScale"]),case["effectiveTrackMm"],covered,bias,
                   side_bias=case["assumedStationExtrinsicErrorMm"])
        local=r.pop("path")
        result[name]={"status":r["status"],"wallS":r["wallS"],"finalStationMmDeg":r["finalStationMmDeg"],
                      "sameEndpointMmDeg":bool(np.allclose(r["finalStationMmDeg"],case["observer"]["finalStationMmDeg"],rtol=0,atol=1.e-6)),
                      "originalStatus":case["observer"]["status"]}
        if final:
            _,_,field,leg,t,truth,_=staged(artifacts/"relieved-match",np.array(case["wheelScale"]),case["effectiveTrackMm"],
                 case["gyroBiasDegS"],case["gyroScaleError"],case["startHeadingErrorDeg"],stage_mm=400,side_mm=0)
            times,poses=join(field,leg,t,truth,local)
            gaps=screen(field,times,poses,stock,team,times[-1])
            result[name]["fieldMinimumMm"]={k:gaps[k] for k in ("minimumPeerMm","minimumStockMm","minimumBarrierMm")}
    out={"registeredReceiverDepthSha256":hashlib.sha256(image_path.read_bytes()).hexdigest(),
         "profileStepSha256":hashlib.sha256((HERE/("narrow-face.step" if final else "short-face-profile.step")).read_bytes()).hexdigest(),
         "newlyOccludedPixels":[int(np.count_nonzero(d<z)) for d,z in zip(add,body)],"trials":result}
    target=output if output else HERE/"optics.json"
    target.write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("artifacts",type=Path);p.add_argument("--narrow",action="store_true");a=p.parse_args();print(run(a.artifacts,40,20,HERE/"optics-narrow.json",True) if a.narrow else run(a.artifacts))
