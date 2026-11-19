"""Test a dual-cable routing corridor against the continuous shop robot stroke."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from route_screen import ROOT, BODY_X, cad, lower_bound
from converter_panel import digest
from routing import (CHAIN_LINKS, CHAIN_PITCH, FIXED_X, HIGH_Z, LOW_Z, MANAGED_LENGTH,
                     MOVING_X, PITCH, RADIUS, STROKE,
                     data_feed, gap_to_boxes, power_feed, rolling_loop)

sys.path.insert(0, str(ROOT / "design/competition-package"))
from travel import rotation_bounds, swept_bounds

SHOP_STEP = "e6f8dac13611d8fa3bfa57d976f67e1e8e84fa4db0a50755821446fa25ea6df7"


def read_json(relative):
    return json.loads((ROOT / relative).read_bytes())


def read_shop(artifacts):
    manifest = read_json("design/fixed-no-air/whole-robot.json")["cases"]["place"]
    frames = read_json("design/competition-package/shop-duplex/red/part-frames.json")
    names = set(manifest["identical_brep_sha256"]) | set(manifest["new_or_changed"])
    names = {key for key in names if not key.startswith("roof-")}
    kit = read_json("camera-datum-shop/native.json")["parts"]
    names.update("roof-" + key for key in kit if key != "receiver-R6-drilled-for-camera")
    step = artifacts / "camera-datum-shop/place.step"
    if digest(step) != SHOP_STEP or len(names) != 603:
        raise ValueError("Shop robot differs from the released 603-solid configuration")
    shapes = cad.read_named(step, [{"id":key} for key in sorted(names)])
    swept = {}
    turns = rotation_bounds()
    for key, shape in shapes.items():
        if key.startswith("roof-") or key == "receiver-R6-drilled-for-camera":
            frame = "slide"
        elif key.startswith(("crossbar-", "height-", "rail-L-fillet-", "rail-R-fillet-")) and key not in frames:
            frame = "lift"
        elif key in {"am-3583", "light-spacer"}:
            frame = "base"
        else:
            frame = frames[key]
        swept[key] = np.array(swept_bounds(key, shape, frame, (560,370), (560,0), turns))
    # The three converter housings are an addition to the verified shop exchange.
    for x in BODY_X:
        swept[f"converter-{x}"] = np.array([x,-193,186,x+36,-134,277])
    return shapes, swept


def check(artifacts):
    shapes, boxes = read_shop(artifacts)
    results = {}
    paths = []
    for name, y, radius, feed, origin in (("power",-329,4.35,power_feed,"converter--96"),
                                          ("data",-317,3.5,data_feed,"observer-pi4-service-envelope")):
        fixed = feed(y)
        paths.append(np.concatenate([points for _,points in fixed]))
        fixed_gap, nearest = gap_to_boxes(fixed, boxes, radius, {origin})[0]
        # Superset of the solid occupied by the U over continuous travel.
        turn_stowed = (FIXED_X + MOVING_X + np.pi*RADIUS - MANAGED_LENGTH) / 2
        rear = turn_stowed - RADIUS
        outer = [rear-radius,y-radius,LOW_Z-radius,MOVING_X+STROKE+radius,y+radius,HIGH_Z+radius]
        loop_gap, loop_nearest = min((lower_bound(outer,box), key) for key,box in boxes.items())
        fixed_length = sum(length for length,_ in fixed)
        lengths = [sum(length for length,_ in rolling_loop(stroke,y)) for stroke in (0,185,370)]
        if max(lengths)-min(lengths) > 1e-8 or min(fixed_gap,loop_gap) <= 0:
            raise ValueError(f"Lost {name} clearance or length: {fixed_gap}, {loop_gap}, {lengths}")
        results[name] = {"yMm":y, "fixedMm":fixed_length, "loopMm":lengths[0], "toHandoffMm":fixed_length+lengths[0],
                         "fixedClearanceLowerMm":fixed_gap, "fixedNearestBody":nearest,
                         "wholeStrokeLoopLowerMm":loop_gap, "loopNearestBody":loop_nearest}
    # Witness a physical collision in the narrower route with a solid cable cylinder.
    red = cad.brep(["cyl",[-210,-282,LOW_Z,1,0,0,4.35,205]])
    clash = cad.intersection(red,shapes["backup-R-service-pin-rail"])
    pair_gap = float(np.linalg.norm(paths[0][:,None,:]-paths[1][None,:,:],axis=2).min()-4.35-3.5-PITCH)
    if pair_gap <= 0 or clash <= 0.001:
        raise ValueError("Cable separation or rail collision control failed")
    # Superset also covers chain overlength D=157 and 20 mm upper camber allowance.
    chain_box = [turn_stowed-157, -323-18, LOW_Z-11.5,
                 MOVING_X+STROKE+75, -323+18, HIGH_Z+11.5+20]
    carrier_gap, nearest = min((lower_bound(chain_box,box),key) for key,box in boxes.items())
    if carrier_gap < 3 or not np.isclose(CHAIN_LINKS*CHAIN_PITCH,MANAGED_LENGTH):
        raise ValueError("Carrier lane lost its lateral installation margin")
    return {"shopStepSha256":SHOP_STEP,"nativeBodies":len(shapes),"cables":results,
            "carrier":{"part":"igus 10.025.100.0","links":CHAIN_LINKS,"pitchMm":CHAIN_PITCH,
                       "managedMm":MANAGED_LENGTH,"catalogChainKg":0.40*MANAGED_LENGTH/1000,
                       "sweptBoundsMm":chain_box,"clearanceLowerMm":carrier_gap,"nearestBody":nearest},
            "fixedToFixedCableGapLowerMm":pair_gap,"parallelLoopJacketGapMm":12-4.35-3.5,
            "rejectedLaneRailIntersectionMm3":clash}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(check(args.artifacts),indent=2)+"\n")
