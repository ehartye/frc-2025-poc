"""Replay acquired chassis states on the scheduled field clock; preserve actual input hashes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from drive import ROOT,approach,entry,route
from field import join,screen,parked_peers
from inertial import staged
from panel import fit,referenced

HERE=Path(__file__).resolve().parent


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def physical_limit():
    # Translation of the mounted camera in station Y is indistinguishable from a
    # chassis displacement at zero yaw. The pinhole projection uses their sum.
    from panel import CENTERS,EYES,project
    baseline=np.array([project(CENTERS,(444.525,0,0),e) for e in EYES])
    shifted=np.array([project(CENTERS,(444.525,3,0),e+3) for e in EYES])
    return float(np.max(abs(baseline-shifted)))


def run(artifacts,out):
    out.mkdir(parents=True,exist_ok=True)
    scenes=artifacts/"selected-optics/issued/marked"
    depths,cal,model_bias,other=referenced(scenes)
    native_pose,err=fit(other,cal,[0,0,0])
    stock=json.loads((ROOT/"full-match/installed-stock.json").read_bytes())["stock"]
    placements=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"]
    _,_,team,_=route.route.field.scenario(placements,inboard=True)
    answers={}
    # Roll radius, effective track, gyro zero rate, gyro scale, initial heading.
    cases={"nominal":(1,1,580,0,0,0),"leftSmall":(.998,1.002,578,.02,.001,.2),
        "rightSmall":(1.002,.998,582,-.02,-.001,-.2),"trackOnly":(1,1,582,0,0,0),
        "bothLarge":(1.002,1.002,582,.02,.001,.2),"bothSmall":(.998,.998,578,-.02,-.001,-.2),
        "crossedDrift":(.998,1.002,582,-.02,-.001,-.2),"badRegistration":(1,1,580,0,0,0),
        "lateStereo":(1,1,580,0,0,0),"lostStereo":(1,1,580,0,0,0),"noContrast":(1,1,580,0,0,0),
        "noGyro":(1,1,580,0,0,0),"lostGyro":(1,1,580,0,0,0),
        "encoderOnly":(.998,1.002,580,0,0,0),"stockAtFarStage":(1,1,580,0,0,0)}
    for name,(l,rr,w,bias,g,heading) in cases.items():
        scale=np.array([l,rr]);raw=artifacts/"relieved-match"
        distance=925 if name=="stockAtFarStage" else 400
        if name=="encoderOnly":
            q,age,f,leg,t,truth=entry(raw,scale,w,distance,0);reg={"latched":False,"residualWheelMmS":0}
        else:
            q,age,f,leg,t,truth,reg=staged(raw,scale,w,bias,g,heading,
                drop_s=0 if name=="noGyro" else 3.50 if name=="lostGyro" else None,
                stage_mm=distance,side_mm=0)
        if reg["residualWheelMmS"]>.00001:raise ValueError("Moving visual handoff")
        if reg["latched"]:
            o={"status":"inertial stage inhibited","wallS":age,"finalStationMmDeg":[q[0],q[1],math.degrees(q[2])],
               "imagesConfirmed":0,"maxTransitionWheelMmS":0.,"path":[(age,*q,0.,0.)]}
        else:
            o=approach(q,age,scale,w,depths,model_bias,
                       fault={"lateStereo":"late","lostStereo":"interrupt","noContrast":"noContrast"}.get(name),
                       side_bias=3 if name=="badRegistration" else 0)
        local=o.pop("path")
        times,poses=join(f,leg,t,truth,local)
        field=screen(f,times,poses,stock,team,times[-1])
        if not o["status"]=="observed stationary":
            field["holdThroughTeleopPeerMm"]=parked_peers(team,poses[-1],times[-1])
        filename=out/(name+".csv")
        np.savetxt(filename,np.c_[times,poses],delimiter=",",fmt="%.9f",header="wall_s,x_m,y_m,heading_rad",comments="")
        eligible=(o["status"]=="observed stationary" and o["wallS"]<21.1 and o["maxTransitionWheelMmS"]<.00001 and
                  min(field[k] for k in ("minimumStockMm","minimumBarrierMm","minimumPeerMm"))>0)
        answers[name]={"wheelScale":[l,rr],"effectiveTrackMm":w,"gyroBiasDegS":bias,"gyroScaleError":g,
            "startHeadingErrorDeg":heading,"assumedStationExtrinsicErrorMm":3 if name=="badRegistration" else 0,
            "stageDistanceMm":distance,"handoffWallS":age,"stationHandoffMmDeg":[q[0],q[1],math.degrees(q[2])],
            "inertial":reg,"observer":o,"occupancy":field,"trajectorySHA256":sha(filename),"eligibleInRehearsal":eligible}
        print(name,o["status"],round(o["wallS"],3),eligible,flush=True)
    source={key:sha(path) for key,path in {"station A grayscale and depth":scenes/"first-fixed.npz",
        "station B grayscale and depth":scenes/"return-fixed.npz",
        "stowed fixed head robot-only depth":scenes.parent/"station-overlay/stowed-robot-fixed.npz",
        "selected-optics/mount.json":ROOT/"selected-optics/mount.json","native.json":HERE/"native.json",
        "full-match/installed-stock.json":ROOT/"full-match/installed-stock.json",
        "relieved-match/return-feed.json":ROOT/"relieved-match/return-feed.json"}.items()}
    return {"sources":source,"sourceCalibrationPx":cal.tolist(),"syntheticCalibrationPx":model_bias.tolist(),
            "returnEstimatedMmDeg":[native_pose[0],native_pose[1],math.degrees(native_pose[2])],
            "returnMaxPixelResidual":err,"equalChassisAndCameraShiftPixelDifference":physical_limit(),"cases":answers}


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("artifacts",type=Path);cli.add_argument("output",type=Path);a=cli.parse_args()
    (HERE/"routes.json").write_text(json.dumps(run(a.artifacts,a.output),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
