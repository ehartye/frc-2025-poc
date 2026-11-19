"""Verify the reversible wheel extraction path through the welded frame (all dimensions mm)."""
import argparse
import json
import math
from pathlib import Path
import sys

from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec

HERE = Path(__file__).resolve().parent
AXLE = HERE.parent
ROOT = AXLE.parent
sys.path.insert(0, str(AXLE))
from fit import STATIONS, SHOP_STEP, add_exchange, cad, chassis_scene, digest, distance, subtract

AXIS_Z = 76.2
PITCH_RADIUS = 23.8125
HEAD_FACE = 302.7 + .188 * 25.4
SHANK = (HEAD_FACE - 31.8, HEAD_FACE, 2.45)
HEAD = (HEAD_FACE, HEAD_FACE + 4, 5.5)
SERVICE_DROP = 8.8
FULL_DROP = 180
HUB_END = 302.7 + .313 * 25.4
# Source drive_parts.hub() profile, stepped axial extents and diameters in inches.
HUB = [(0,.063,.75), (.063,.125,1.123), (.125,.313,2.31),
       (.313,.375,1.123), (.375,.938,.88), (.938,1,.75)]


def data(path):
    return json.loads(path.read_bytes())


def cylinder(x, z, sign, start, end, radius):
    return cad.brep(["cyl", [x, sign * start, z, 0, sign, 0, radius, end - start]])


def capsule(spec, sign, z0, z1):
    """A cylinder translated vertically sweeps two round caps and their common tangent prism."""
    x, z, start, end, radius = spec
    poly = BRepBuilderAPI_MakePolygon()
    for corner_x, corner_z in ((x-radius,z+z0), (x+radius,z+z0),
                               (x+radius,z+z1), (x-radius,z+z1)):
        poly.Add(gp_Pnt(corner_x, sign * start, corner_z))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    slab = BRepPrimAPI_MakePrism(face, gp_Vec(0, sign * (end-start), 0)).Shape()
    return [cylinder(x,z+dz,sign,start,end,radius) for dz in (z0,z1)] + [slab]


def gap_bound(a, b):
    return math.sqrt(sum(max(0, a[i]-b[i+3], b[i]-a[i+3])**2 for i in range(3)))


def evaluate(groups, fixed):
    boxes = {key:cad.bounds(shape) for key,shape in fixed.items()}
    closest = (float("inf"), "", "")
    conflicts = {}
    for label, shapes in groups.items():
        for shape in shapes:
            box = cad.bounds(shape)
            candidates = sorted((gap_bound(box, other), key) for key,other in boxes.items())
            for bound,key in candidates:
                if bound > closest[0] + 1e-6:
                    break
                gap = distance(shape, fixed[key])
                closest = min(closest, (gap, label, key))
                if gap < 1e-6:
                    volume = cad.intersection(shape, fixed[key])
                    if volume > .001:
                        pair = label + "/" + key
                        conflicts[pair] = max(conflicts.get(pair, 0), round(volume, 4))
    return {"gapMm": round(closest[0], 5), "pair": list(closest[1:]), "overlapMm3": conflicts}


def load_scene(artifacts):
    _, scene = chassis_scene(artifacts)
    article_path = ROOT / "design/selected-shop/articles.json"
    article = data(article_path)["hub_screws"]
    if (article["sku"], article["quantity_each_wheel"], article["head_max_diameter_mm"],
        article["head_max_height_mm"]) != ("am-1592", 6, 11, 4):
        raise ValueError("The wheel screw changed. Rebuild its screw and tool gauges.")
    service_path = AXLE / "service.json"
    pull = data(service_path)
    pins = {"camera-datum-shop/place.step": SHOP_STEP,
            "design/selected-shop/articles.json": digest(article_path),
            "design/installed-procurement/inner-clamps.step": data(ROOT/"design/installed-procurement/clamp-review.json")["candidateStepSha256"],
            "frame-corners/frame-weldment.step": data(ROOT/"frame-corners/weld-fit.json")["stepSha256"],
            "axle-retention/service.json": digest(service_path)}
    for filename, receipt, field in (("axle-kit.step","fit.json","kitSha256"),
                                     ("bearing-keepers.step","keepers.json","keeperSha256")):
        issued = data(AXLE / receipt)
        if issued["stepSha256"] != pull[field]:
            raise ValueError("Shaft pull and native wheel assembly have different inputs")
        add_exchange(scene, AXLE/filename, issued["volumeMm3"], issued["stepSha256"])
        pins["axle-retention/" + filename] = issued["stepSha256"]
    for pos in STATIONS:
        for side in "LR":
            witness = pull["wheels"][pos+"-"+side]
            if witness["outwardTravelMm"] != 160 or witness["sweptMaterialOverlapMm3"]:
                raise ValueError("The bare shaft must have a clear 160 mm outward path")
    return scene, pins


def separate(scene, pos, side):
    suffix = pos + "-" + side
    index = 3 if pos == "center" else 2
    inner = f"spacer-{pos}-{index-1}-{side}"
    outer = f"spacer-{pos}-{index}-{side}"
    moving = {key:scene[key] for key in ("wheel-"+suffix, "hub-"+suffix, inner)}
    removed = {k for k in scene if k.startswith("bumper-") and "sleeve" not in k}
    removed.update(k for k in scene if k.startswith(("retaining-screw-"+suffix, "select-shim-"+suffix)))
    removed.update({"axle-"+suffix, "wheel-hardware-head-"+suffix, "wheel-hardware-thread-"+suffix})
    if pos == "center":
        removed.update({"clip-sweep-"+side, "clip-thrust-"+side})
    return moving, {k:v for k,v in scene.items() if k not in removed and k not in moving}, inner, outer


def wheel_gauges(x, sign, inner, phase, missing=None):
    bounds = cad.bounds(inner)
    lo, hi = sorted((bounds[1]*sign, bounds[4]*sign))
    gauges = {"wheel": (x, AXIS_Z, 277.3, 302.7, 76.2),
              "inner-spacer": (x, AXIS_Z, lo, hi, 9)}
    for i,(start,end,dia) in enumerate(HUB):
        gauges[f"hub-{i}"] = (x, AXIS_Z, HUB_END-end*25.4, HUB_END-start*25.4, dia*12.7)
    for i in range(6):
        if i == missing:
            continue
        angle = phase+i*math.pi/3
        cx, cz = x+PITCH_RADIUS*math.cos(angle), AXIS_Z+PITCH_RADIUS*math.sin(angle)
        for label, extents in (("head",HEAD), ("shank",SHANK)):
            gauges[f"{label}-{i}"] = (cx,cz,*extents)
    return gauges


def sweep(gauges, sign, z0, z1, fixed):
    return evaluate({name:capsule(spec,sign,z0,z1) for name,spec in gauges.items()}, fixed)


def full_clock(x, sign, drop, fixed):
    groups = {}
    for label,(a,b,r) in (("head",HEAD),("shank",SHANK)):
        outer = cylinder(x,AXIS_Z-drop,sign,a,b,PITCH_RADIUS+r)
        core = cylinder(x,AXIS_Z-drop,sign,a-1,b+1,PITCH_RADIUS-r)
        groups[label] = [subtract(outer, core)]
    return evaluate(groups, fixed)


def cradle(x, sign, offset=0, extra_z=0):
    # Open-top side-pressure fixture envelope: 1.4 mm axial jaw, 24 mm total width.
    # Extend the lower stem below the robot to cover the vertical insertion sweep.
    start, end = 312+offset, 313.4+offset
    y = min(sign*start,sign*end)
    body = cad.brep(["box",[x-12,y,-200,24,end-start,278+extra_z]])
    seat = cylinder(x,AXIS_Z+.1+extra_z,sign,start-.1,end+.1,9.1)
    mouth = cad.brep(["box",[x-9.1,y-.1,AXIS_Z+.1+extra_z,18.2,end-start+.2,8]])
    return subtract(subtract(body,seat),mouth)


def review(scene, pos, side):
    sign = 1 if side == "L" else -1
    x = STATIONS[pos]
    moving,fixed,inner,outer = separate(scene,pos,side)
    all_screws = wheel_gauges(x,sign,moving[inner],-math.pi/2)
    five = wheel_gauges(x,sign,moving[inner],math.pi/2,missing=0)
    # A flush, headless 32 mm slave mandrel enters after the service shaft.
    # Its parked ends lie inside the traveling wheel/inner-spacer stack.
    pilot = (x, AXIS_Z, 278.35, 310.35, 6.2)
    inner_bounds = cad.bounds(moving[inner])
    inner_end = max(sign * inner_bounds[1], sign * inner_bounds[4])
    base_fixture = cradle(x,sign)
    with_fixture = {**fixed,"parked-spacer-cradle":base_fixture}
    source = {}
    for name,(cx,cz,a,b,r) in all_screws.items():
        if name.startswith(("head-","shank-")):
            source[name] = cylinder(cx,cz,sign,a,b,r)
    around_cradle = {**moving, **source, **fixed}
    around_cradle.pop(outer)
    case = {"moving": list(moving), "outboard": outer,
            "cradleInsertion": evaluate({"fixture":[base_fixture]}, around_cradle),
            "cradleToSpacerVolumeMm3": round(cad.intersection(base_fixture, fixed[outer]),5),
            "cradleRaisedPoint2ControlMm3": round(cad.intersection(cradle(x,sign,extra_z=.2),fixed[outer]),5),
            "clockAllAtSource": full_clock(x,sign,0,with_fixture),
            "lower8p8": sweep(all_screws,sign,0,-SERVICE_DROP,with_fixture),
            "clockAtService": full_clock(x,sign,SERVICE_DROP,with_fixture)}
    case["pilotEngagementInboardMm"] = round(inner_end - pilot[2], 4)
    case["pilotAxialInsertion"] = evaluate({"pilot":[cylinder(x,AXIS_Z,sign,pilot[2],450,6.2)]},
                                           {**with_fixture,**moving,**source})
    case["pilotDuringDrop"] = sweep({"pilot":pilot},sign,0,-FULL_DROP,with_fixture)
    if case["pilotEngagementInboardMm"] < 6:
        raise ValueError("The temporary pilot does not support the inner spacer")
    z = AXIS_Z - SERVICE_DROP - PITCH_RADIUS
    filtered_jaw = subtract(base_fixture, cylinder(x,z,sign,311,315,6.5))
    working = {**fixed,"slotted-spacer-cradle":filtered_jaw}
    case["lowestScrewExtraction"] = evaluate({
        "head":[cylinder(x,z,sign,HEAD[0],HEAD[1]+38,HEAD[2])],
        "shank":[cylinder(x,z,sign,SHANK[0],HEAD[0]+38,SHANK[2])],
        "socket":[cylinder(x,z,sign,HEAD[0],450,4.5)]}, working)
    case["withdrawWithFive"] = sweep(five,sign,-SERVICE_DROP,-FULL_DROP,with_fixture)
    if pos == "center":
        bb = cad.bounds(fixed[outer])
        lo, hi = sorted((sign*bb[1], sign*bb[4]))
        free = {k:v for k,v in fixed.items() if k != outer}
        case["centerSpacerOutward"] = evaluate({"outboard-spacer":[cylinder(x,AXIS_Z,sign,lo,hi+1.5,9)]},
                                               {**free,**moving,**source})
        # 2.9 mm wide cover bounds the entire axial motion of the 1.4 mm jaw.
        carrier = cradle(x,sign)
        shifted_jaw = cradle(x,sign,offset=1.5)
        mid_jaw = cradle(x,sign,offset=.75)
        case["centerCradleShift"] = evaluate({"cradle":[carrier,mid_jaw,shifted_jaw]},
                                              {**free,**moving,**source})
        parked = {**free, outer:cad.translated(fixed[outer],[0,sign*1.5,0]),
                  "parked-spacer-cradle":shifted_jaw}
        case["centerStraightDrop"] = sweep(all_screws,sign,0,-FULL_DROP,parked)
        disks = {label:(x,AXIS_Z,a,b,PITCH_RADIUS+r)
                 for label,(a,b,r) in (("head",HEAD),("shank",SHANK))}
        case["centerAnyPhaseDrop"] = sweep(disks,sign,0,-FULL_DROP,parked)
    # A positive interference control using the original six fasteners, held nominal spacer.
    head_specs = {k:v for k,v in all_screws.items() if k.startswith("head-")}
    case["straightSixControl"] = sweep(head_specs,sign,0,-FULL_DROP,{outer:fixed[outer]})
    for stage, check in case.items():
        if isinstance(check,dict) and stage != "straightSixControl":
            if check["overlapMm3"] or check["gapMm"] <= 0:
                raise ValueError((pos,side,stage,check))
    if (not case["straightSixControl"]["overlapMm3"] or case["cradleToSpacerVolumeMm3"] > .001
            or case["cradleRaisedPoint2ControlMm3"] <= .001):
        raise ValueError((pos,side,"interference control",case))
    return case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path, help="root containing camera-datum-shop/place.step")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    scene, pins = load_scene(args.artifacts)
    output = {"sha256":pins,"fixedSceneIncludingWheelHardwareGauges":len(scene),"cases":{}}
    for pos in STATIONS:
        for side in "LR":
            output["cases"][pos+"-"+side] = review(scene,pos,side)
            print(pos,side,"passed",flush=True)
    args.output.write_text(json.dumps(output,indent=2)+"\n",encoding="utf-8")


if __name__ == "__main__":
    main()
