"""Reopen shop bumper covers and test the reserved corridor through the buzzer."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"leave-park"))
from rehearse import TAPE,ZONE,corrected,overlap,traffic
from straight_role import team_at_teleop
sys.path.insert(0,str(ROOT/"moving-harness"))
from continuous_screen import read_shop, SHOP_STEP
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import parts, cad, FIRST_SHA


def check(artifacts,field):
    study=json.loads((HERE/"results.json").read_bytes())["short"]
    native=json.loads((ROOT/"leave-park/near-tape.json").read_bytes())["cases"]["near-tape"]
    station=parts(field)
    toe=next(v for k,v in station.items() if k.endswith("/GE-25007"))
    toe_bounds=list(cad.bounds(toe))
    if max(abs(a-b) for a,b in zip(toe_bounds,[-3.175,-1012.825,0,9.525,1012.825,304.8])) > .001:
        raise ValueError("Unexpected station toe geometry")
    f=corrected()
    shapes,_=read_shop(artifacts)
    covers={k:v for k,v in shapes.items() if k.startswith("bumper-cover-")}
    if len(covers)!=2:
        raise ValueError("Expected the two removable covers")
    run=f.Route(tuple(native["start"]))
    run.go(native["finish"][:2])
    err=study["sweptHullErrorMm"]/25.4
    initial=(5+2*math.hypot(425,425)*math.sin(math.radians(.4)/2))/25.4
    def eroded(region,margin):
        x0,y0,x1,y1=region
        return x0+margin,y0+margin,x1-margin,y1-margin
    robust_start=overlap(f,covers,run.start,eroded((TAPE[0],0,TAPE[1],317),initial))
    robust_end=overlap(f,covers,run.end,eroded(ZONE,err))
    # Treat every mid-route abort as an occupied point for the full TELEOP.
    # The enclosing prism also covers continuous trajectories between samples.
    team=team_at_teleop(f,run)
    half_step=.010
    half_length=.425+(run.start[0]-run.end[0])/2
    middle=((run.start[0]+run.end[0])/2,run.start[1],run.start[2])
    acceleration=f.DRIVE["maximumWheelAccelMps2"]*(1+2*traffic.RADIUS/f.DRIVE["trackM"])
    distances={}
    for key,program in team.items():
        if key=="red2":
            continue
        lowest=math.inf
        for tick in range(6750):
            pose,wheels,_=program.at((2*tick+1)*half_step)
            gap = traffic.separation(middle,pose,(half_length,.425))
            lower = gap-half_step*traffic.motion_bound(wheels)-acceleration*half_step**2
            lowest=min(lower,lowest)
        distances[key]=1000*lowest-study["sweptHullErrorMm"]
    return {"fieldStepSha256":FIRST_SHA,"toeNativeBoundsMm":toe_bounds,"shopStepSha256":SHOP_STEP,"twoCoverIds":sorted(covers),
            "startErodedTapeCommonMm3":robust_start,"parkErodedZoneCommonMm3":robust_end,
            "fullCorridorTeleopPeerGapMm":distances,
            "rule":"source-scheduled peers with Red 1 assigned middle; includes every partial stop",
            "sourceResultsSha256":hashlib.sha256((HERE/"results.json").read_bytes()).hexdigest()}


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("artifacts",type=Path)
    cli.add_argument("field_step",type=Path)
    args=cli.parse_args()
    result=check(args.artifacts,args.field_step)
    (HERE/"field.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
