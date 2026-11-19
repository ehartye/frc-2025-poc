"""Register the selected shop hardware on both endpoints of the relieved match robot."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "axle-retention"))
from fit import add_exchange, cad, chassis_scene, clamps, digest

sys.path.insert(0, str(ROOT / "return-station"))
from registration import dock, parts as station_parts
from receiver_relief import low_relief

sys.path.insert(0, str(ROOT / "bridge-tolerance"))
from field import check, corrected, heading_sweep, mod, native_pose, stationary
from match import HULL

sys.path.insert(0, str(ROOT / "camera-disconnects"))
from routing import PITCH, data_feed, power_feed
from route_screen import BODY_X


def read(relative):
    return json.loads((ROOT / relative).read_bytes())


def native_box(bounds):
    b = np.asarray(bounds, dtype=float)
    return cad.brep(["box", [*b[:3], *(b[3:] - b[:3])]])


def shop_overlays(artifacts):
    original, installed = chassis_scene(artifacts)
    changed = set(installed) - set(original)
    changed.update(("cross-front", "cross-rear", "drive-rail-outer-L", "drive-rail-outer-R"))
    for step, receipt in (("axle-kit.step", "fit.json"), ("bearing-keepers.step", "keepers.json")):
        package = read("axle-retention/" + receipt)
        add_exchange(installed, ROOT/"axle-retention"/step, package["volumeMm3"], package["stepSha256"])
        changed.update(package["volumeMm3"])
    roof = {k:v for k,v in original.items() if k.startswith("roof-") or k == "receiver-R6-drilled-for-camera"}
    if len(roof) != 32 or len(installed) != 795:
        raise ValueError("Unrecognized shop camera or chassis package")
    return {k:installed[k] for k in sorted(changed)}, roof


def poses(artifacts, lamp):
    fixed, roof = shop_overlays(artifacts)
    support = read("camera-disconnects/support.json")
    step = ROOT / "camera-disconnects/service-support.step"
    if digest(step) != support["step_sha256"]:
        raise ValueError("Selected connector support changed")
    plugs = cad.read_named(step, [{"id": k} for k in support["parts"]])
    result = []
    for stroke in (0, 370):
        robot = low_relief(lamp, stroke)
        superseded = {k for k in robot if k.startswith("roof-") or k == "receiver-R6-drilled-for-camera"}
        if len(superseded) != 22 or not set(clamps.REPLACED) <= robot.keys():
            raise ValueError("Unexpected mission receiver or drive assembly")
        for k in superseded | set(clamps.REPLACED):
            del robot[k]
        robot.update(fixed)
        robot.update({k:cad.translated(v,[stroke-370,0,0]) for k,v in roof.items()})
        robot.update({"support/"+k:cad.translated(v,[stroke,0,0]) for k,v in plugs.items()})
        result.append(robot)
    if result[0].keys() != result[1].keys():
        raise ValueError("Different stock at the two endpoints")
    return result


def bounded_curve(points, depth=5):
    """Bisect the Bezier curve. Each segment stays in its control polygon's convex hull."""
    if not depth:
        yield np.r_[points.min(axis=0), points.max(axis=0)]
        return
    rows = [points]
    while len(rows[-1]) > 1:
        rows.append((rows[-1][:-1] + rows[-1][1:]) / 2)
    yield from bounded_curve(np.array([row[0] for row in rows]), depth-1)
    yield from bounded_curve(np.array([row[-1] for row in reversed(rows)]), depth-1)


def wire_boxes():
    # Published mounted stock bounds, in the stowed slide frame.
    slide = {
        "registered-camera-case": [29.5,-45.5,842.05,57.5,45.5,859.5],
        "Y-case-and-output": [19.5,-28.5,841.3,39,-2.6,854.25],
        "data-plug": [-13,-27.1,837.4,25.1,-20,858.2],
        "power-plug": [-53,-11.4,841.35,26,-3.7,854.2],
    }
    curves = read("camera-disconnects/route-controls.json")
    for label, radius in (("data",3.15),("power",4.30)):
        pad = np.array([-radius]*3 + [radius]*3)
        for i, bounds in enumerate(bounded_curve(np.asarray(curves[label],dtype=float))):
            slide[f"{label}-tail-{i}"] = bounds + pad
    fixed = {f"converter-{x}":[x,-193,186,x+36,-134,277] for x in BODY_X}
    for label, path, radius in (("power",power_feed(-329),4.35),("data",data_feed(-317),3.5)):
        points = np.concatenate([points for _, points in path])
        pad = radius + PITCH/2
        fixed[label+"-feed"] = np.r_[points.min(axis=0)-pad, points.max(axis=0)+pad]
    # The same envelope contains every intermediate chain/cable U, at both endpoints.
    fixed["carrier"] = read("moving-harness/geometry.json")["carrier"]["sweptBoundsMm"]
    return {k:np.array(v) for k,v in fixed.items()}, {k:np.array(v) for k,v in slide.items()}


def extrema(boxes):
    return [min((float(b[i]), k) for k,b in boxes.items()) if i<3 else
            max((float(b[i]), k) for k,b in boxes.items()) for i in range(6)]


def hull_margins(extents):
    return [pair[0]-HULL[i] if i<3 else HULL[i]-pair[0] for i,pair in enumerate(extents)]


def run(artifacts, lamp, field):
    original = read("relieved-match/native-register.json")
    if digest(field) != original["fieldSha256"]:
        raise ValueError("Unregistered field")
    stowed, placed = poses(artifacts, lamp)
    fixed, moving = wire_boxes()
    at_rest = {k:np.array(cad.bounds(v)) for k,v in stowed.items()}
    at_rest.update(fixed)
    at_rest.update(moving)
    stroke = np.array([370,0,0,370,0,0])
    at_end = {k:np.array(cad.bounds(v)) for k,v in placed.items()}
    at_end.update(fixed)
    at_end.update({k:v+stroke for k,v in moving.items()})
    rotations = read("design/competition-package/shop-duplex/red/rotation-bounds.json")
    frames = read("design/competition-package/shop-duplex/red/part-frames.json")
    sweeps = {}
    unrotated = {}
    for k, b in at_rest.items():
        e = at_end[k]
        if k in rotations:
            b = np.array(rotations[k]["bounds_mm"]) + [0,0,560,0,0,560]
            e = b + stroke * (.5 if frames[k] == "middle" else 0)
            at_rest[k] = b
        grip = 1.5 if k.startswith("grip-") else 0.
        travel = 15. if np.max(abs(e-b)) > .001 or k=="carrier" else 0.
        pad = np.array([travel,grip,0])
        unrotated[k] = np.r_[np.minimum(b[:3],e[:3])-pad,np.maximum(b[3:],e[3:])+pad]
        sweeps[k] = heading_sweep(unrotated[k],1.)
        if grip:
            at_rest[k] = b + [0,-grip,0,0,grip,0]
    bounds = extrema(at_rest)
    limits = hull_margins(bounds)
    navigation = read("relieved-match/navigation.json")["cases"]
    structure = station_parts(field)
    structure_boxes = {k:cad.bounds(v) for k,v in structure.items()}
    dock_scene = dict(stowed)
    dock_scene.update({k:native_box(v) for k,v in {**moving, **fixed}.items()})
    dock_checks = {}
    registered = read("full-match/installed-stock.json")
    if registered["field_step_sha256"] != digest(field):
        raise ValueError("Field inventory mismatch")
    layout = corrected()
    sectors = {}
    structure_source = mod("shop_reef_geometry", ROOT/"moving-observer/clearance.py")
    reef = structure_source.reef_parts(field)
    world_boxes = {k:native_box(b) for k,b in sweeps.items()}
    structures = {}
    for piece, sector in (("coral-A",-.18),("coral-B",.22)):
        nav = next(r for r in navigation if r["pipe"]==piece and r["mode"]=="nominal")
        normal, side, yaw = nav["actualStationMmDeg"]
        dock_checks[piece] = dock.native_dock(dock_scene,structure,structure_boxes,10-normal,yaw,-side)
        structures[piece] = structure_source.find_min(reef,native_pose(world_boxes,sector*1000))
        structures[piece]["lowerMm"] = structures[piece]["minimum_mm"]-10-1-.526342
        hits = check(layout,registered["stock"],sweeps,stationary(layout.routes(sector)[1].end))
        sectors[piece] = {k:{**v,"qualifiedLowerMm":v["lower_bound_mm"]-10-1-.526342} for k,v in hits.items()}
    witness = dock_checks["coral-A"]
    key, obstacle = witness["closest_body"], witness["closest_station_occurrence"]
    displaced = dock.moved({key:stowed[key]},dock.BASE-8,0,0)[key]
    interference = cad.intersection(displaced,structure[obstacle])
    earlier = read("fixed-height-match/native.json")["firstTubeToSecondDockMm"]["inflatedTubeBoxMm"]
    from registration import mount
    projected_gap, nearest = min((mount.limits(b,earlier),k) for k,b in sweeps.items())
    extension = extrema(unrotated)
    occupied_x = max(abs(extension[0][0]),abs(extension[3][0]))
    occupied_y = max(abs(extension[1][0]),abs(extension[4][0]))
    roster_excess = float(np.hypot(max(0,occupied_x-535),max(0,occupied_y-425)))
    nominal = read("relieved-match/match.json")["cases"]["nominal"]
    return {
        "sources": {p:digest(ROOT/p) for p in (
            "axle-retention/fit.json","axle-retention/keepers.json","camera-datum-shop/native-place.json",
            "camera-disconnects/support.json","camera-disconnects/route-controls.json","moving-harness/geometry.json",
            "station-relief/geometry.json","station-relief/shims.json","relieved-match/navigation.json",
            "camera-disconnects/target_stock.py","moving-harness/routing.py","full-match/match.py",
            "relieved-match/native-register.json","relieved-match/match.json","full-match/installed-stock.json")},
        "fieldSha256":digest(field), "lampSha256":digest(lamp), "nativeAndRotationalBodies":len(stowed),
        "additionalEnvelopeCount":len(fixed)+len(moving),"stowedHullMm":HULL,
        "stowedExtrema":bounds,"hullFaceMarginsMm":limits,"containedAtStow":min(limits)>=-1e-6,
        "station":dock_checks, "stationOverrunControl":{"body":key,"field":obstacle,"extraApproachMm":18,"commonMm3":interference},
        "strokeExtrema":extension, "reefStructure":structures, "sectorStock":sectors,
        "matchedRoute":{"stowedBarrierLowerMm":nominal["stowedFieldLowerMm"] if min(limits)>=-1e-6 else None,
                        "stowedStockLowerMm":nominal["stowedStockLowerMm"] if min(limits)>=-1e-6 else None,
                        "extraPlacementRadiusMm":roster_excess,
                        "sixRobotLowerMm":nominal["allRobotLowerMm"]-roster_excess if min(limits)>=-1e-6 else None}, "previousPipe":{"body":nearest,"lowerMm":projected_gap-10-1-.526342},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ("artifacts","lamp","field","output"):
        parser.add_argument(arg,type=Path)
    args = parser.parse_args()
    report = run(args.artifacts,args.lamp,args.field)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
