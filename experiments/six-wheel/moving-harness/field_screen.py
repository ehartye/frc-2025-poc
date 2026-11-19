"""Check the proposed service corridor against the registered station and reef."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from routing import PITCH, data_feed, power_feed

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0,str(ROOT/"bridge-tolerance"))
from field import box_gaps, cad, coordinate, corrected, heading_sweep, mod, native_pose
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import FIRST_SHA, parts as station_parts
from dock_tolerance import moved


def check(field):
    if hashlib.sha256(field.read_bytes()).hexdigest()!=FIRST_SHA:
        raise ValueError("Different field geometry")
    source = mod("harness_field_geometry",ROOT/"moving-observer/clearance.py")
    received = json.loads((HERE/"geometry.json").read_bytes())["carrier"]["sweptBoundsMm"]
    # Enclose both limiting rack endpoint errors even though the carrier already
    # nests all nominal positions. A three-millimeter mounting error is separate.
    boxes = {"carrier":np.array(received)+[-15,0,0,15,0,0]}
    for name, feed, y, radius in (("fixed-power",power_feed,-329,4.35),("fixed-data",data_feed,-317,3.5)):
        path = np.concatenate([p for _,p in feed(y)])
        boxes[name] = np.r_[path.min(axis=0)-radius-PITCH/2,path.max(axis=0)+radius+PITCH/2]
    boxes = {key:heading_sweep(value,2) for key,value in boxes.items()}
    envelopes = {key:cad.brep(["box",[*v[:3],*(v[i+3]-v[i] for i in range(3))]]) for key,v in boxes.items()}
    reef = source.reef_parts(field)
    station = source.find_min(station_parts(field),moved(envelopes,444.525,0,0))
    station["lowerMm"] = station["minimum_mm"]-35-3
    register = json.loads((ROOT/"full-match/installed-stock.json").read_bytes())
    if register["field_step_sha256"]!=FIRST_SHA:
        raise ValueError("Installed stock has different registration")
    layout = corrected()
    sectors = {}
    for sector in (-.18,.22):
        fixed = source.find_min(reef,native_pose(envelopes,sector*1000))
        pose = layout.routes(sector)[1].end
        balls = []
        for key, obj in register["stock"].items():
            if obj["group"]!="reef-ball":
                continue
            center = coordinate(layout,obj,pose)
            for part, box in boxes.items():
                dx,dy,dz = box_gaps(center,box)
                gap = math.hypot(max(0,math.hypot(dx,dy)-35),dz)-register["ball_radius_mm"]-3
                balls.append((gap,part,key))
        gap, part, occurrence = min(balls)
        sectors[str(sector)] = {"structureLowerMm":fixed["minimum_mm"]-35-3,"structurePair":fixed["pair"],
                                "occupiedBallLowerMm":gap,"occupiedPair":[part,occurrence]}
    return {"fieldSha256":FIRST_SHA,"poseTolerance":{"translationDiskMm":35,"yawDeg":2,
            "rackEndpointMm":15,"mountMm":3},"station":station,"sectors":sectors}


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(check(args.field),indent=2)+"\n",encoding="utf-8",newline="\n")
