"""Measure the hanging hardware around the complete stowed straight-drive corridors."""
import argparse
import itertools
import json
import math
from pathlib import Path

from native_park import FIRST_SHA, HERE, KIT, bounds, cad, digest, direct_children, name, native_box
from rehearse import ROOT, STOCK, TAPE, ZONE, corrected, overlap
from installed_stock import check as check_stock
from straight_role import assess
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf


def children(table, parent, placement):
    for occurrence, definition in direct_children(table, parent):
        yield (name(occurrence), name(definition), definition,
               placement.Multiplied(table.GetLocation_s(occurrence)))


def horizontal_center(box):
    return [(box[0] + box[3]) / 2, (box[2] + box[5]) / 2]


def radius(box, axis):
    corners = itertools.product((box[0], box[3]), (box[2], box[5]))
    return max(math.dist(axis, point) for point in corners)


def clearance(axis, route_box):
    x, z = axis
    return math.hypot(max(route_box[0] - x, x - route_box[3], 0),
                      max(route_box[2] - z, z - route_box[5], 0))


def hanging_inventory(field):
    if digest(field) != FIRST_SHA:
        raise ValueError("Unexpected field STEP revision")
    doc, table = cad.xcaf_doc()
    reader = cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(field)) != cad.IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Could not reopen field")
    roots = cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    if roots.Length() != 1 or name(roots.Value(1)) != "FE-2025":
        raise ValueError("Expected the complete field")
    _, _, barge, loc = next(row for row in children(table, roots.Value(1), TopLoc_Location())
                            if row[1] == "GE-25200")
    stations = {}
    moving = []
    cage_shapes = {}
    for occurrence, part, definition, placement in children(table, barge, loc):
        if part == "GE-25230":
            _, _, eye, eye_loc = next(row for row in children(table, definition, placement)
                                      if row[1].startswith("eyenut_"))
            box = bounds(table.GetShape_s(eye).Moved(eye_loc))
            stations[occurrence] = {"axis_mm": horizontal_center(box), "eye_box_mm": box,
                                    "parts": {}}
        elif part.startswith("GE-25231_") or part in ("3216T41_Carabiner", "GE-25229"):
            shape = table.GetShape_s(definition).Moved(placement)
            moving.append((occurrence, part, bounds(shape)))
            if part.startswith("GE-25231_"):
                cage_shapes[occurrence] = shape
    if len(stations) != 6 or len(cage_shapes) != 6:
        raise ValueError("Expected six hang points and six complete cages")
    for occurrence, part, box in moving:
        station = min(stations.values(), key=lambda s: math.dist(s["axis_mm"], horizontal_center(box)))
        if math.dist(station["axis_mm"], horizontal_center(box)) > 5:
            raise ValueError("Unregistered hanging part")
        station["parts"][occurrence] = {"type": part, "box_mm": box}
    for station in stations.values():
        parts = station["parts"]
        cage_id, = [key for key in parts if key in cage_shapes]
        chains = [p for p in parts.values() if p["type"] == "GE-25229"]
        clips = [p for p in parts.values() if p["type"] == "3216T41_Carabiner"]
        if len(chains) not in (0, 1) or len(clips) != 1 + len(chains):
            raise ValueError("Missing suspension hardware")
        station["cage"] = cage_id
        station["setup"] = "deep" if chains else "shallow"
        station["radius_mm"] = max(radius(p["box_mm"], station["axis_mm"]) for p in parts.values())
        upper_clip = max(clips, key=lambda p: p["box_mm"][4])["box_mm"]
        eye = station["eye_box_mm"]
        lo, hi = max(eye[1], upper_clip[1]), min(eye[4], upper_clip[4])
        if lo >= hi:
            raise ValueError("Upper clip and fixed eye have disjoint height intervals")
        station["trial_pivot_height_mm"] = (lo + hi) / 2
    return stations, cage_shapes


def contact_trial(station, cage, route, robot):
    # In the straight red pose native X opposes chassis X, Y is up, and Z follows chassis Y.
    rx = route["finish"][0] * 1000 - 690.875 * 25.4 / 2
    rz = 317 * 25.4 / 2 - route["finish"][1] * 1000
    tr = gp_Trsf()
    tr.SetValues(-1, 0, 0, rx, 0, 0, 1, 0, 0, 1, 0, rz)
    bumper = robot["bumper-cover-front-red"].Moved(TopLoc_Location(tr))
    x, z = station["axis_mm"]
    pivot = gp_Ax1(gp_Pnt(x, station["trial_pivot_height_mm"], z), gp_Dir(0, 0, 1))
    trials = {}
    for degrees in (0, 4, 6, 8, 10):
        tilt = gp_Trsf()
        tilt.SetRotation(pivot, math.radians(degrees))
        common = BRepAlgoAPI_Common(bumper, cage.Moved(TopLoc_Location(tilt)))
        if not common.IsDone():
            raise ValueError("Failed native bumper intersection")
        trials[str(degrees)] = cad.volume(common.Shape())[0]
    return {"route": "shared-105", "cage": station["cage"], "body": "bumper-cover-front-red",
            "pivot_native_mm": [x, station["trial_pivot_height_mm"], z],
            "axis_native": [0, 0, 1], "overlap_mm3_by_degrees": trials}


def check_locked_robot(folder, robot, row):
    native = json.loads((folder / "native.json").read_bytes())["red"]
    loads = json.loads((folder / "review.json").read_bytes())["straightDrive"]["red"]
    if digest(folder / "locks.step") != native["exportSha256"] or native["revision"] != digest(KIT / "stowed.step"):
        raise ValueError("Unexpected locked revision")
    edits, additions = set(native["editedVolumesMm3"]), set(native["addedVolumesMm3"])
    if not edits <= robot.keys() or additions & robot.keys() or any(k.startswith("bumper-") for k in edits):
        raise ValueError("Lock kit and issued bumper inventory disagree")
    kit = cad.read_named(folder / "locks.step", [{"id": key} for key in edits | additions])
    installed = {**robot, **kit}
    boxes = [cad.bounds(shape) for shape in installed.values()]
    hull = [min(b[i] for b in boxes) for i in range(3)] + [max(b[i] for b in boxes) for i in range(3, 6)]
    f = corrected()
    route = f.Route(tuple(row["start"]))
    route.go(row["finish"][:2])
    if native_box(f, route, hull) != row["native_swept_box_mm"] or len(installed) != 583:
        raise ValueError("Locked robot requires a different corridor")
    tape = (TAPE[0], 0, TAPE[1], f.PROFILE["field"]["width"])
    material = [sum(overlap(f, installed, pose, region).values())
                for pose, region in [(route.start, tape), (route.end, tape), (route.end, ZONE)]]
    stock = json.loads(STOCK.read_bytes())
    if stock["field_step_sha256"] != FIRST_SHA:
        raise ValueError("Unregistered stock")
    gaps = check_stock(f, stock["stock"], {"locked robot": hull}, route.segments)
    demands = [assess(dict(loads["parameters"], massKg=m), {"segments": route.segments}) for m in loads["massKg"]]
    if material[0] <= 0 or material[1] != 0 or material[2] <= 0 or min(v["lower_bound_mm"] for v in gaps.values()) <= 0:
        raise ValueError("Locked robot failed the scoring or stock check")
    return {"installed_bodies": len(installed), "revision_sha256": native["revision"],
            "lock_kit_sha256": native["exportSha256"], "load_source_sha256": digest(folder / "review.json"),
            "route": "near-tape", "hull_mm": hull, "start_tape_end_tape_end_zone_mm3": material,
            "stock_count": len(stock["stock"]), "stock_bounds": gaps, "mass_kg": loads["massKg"],
            "drive_cases_clear": all(d["parameterCaseClears"] for d in demands),
            "peak_phase_a": max(d["extrema"]["phaseA"] for d in demands),
            "minimum_bus_v": min(d["extrema"]["busV"] for d in demands),
            "required_friction": max(d["extrema"]["frictionRequired"] for d in demands)}


def measure(field, locks=ROOT / "design/fixed-height"):
    path = HERE / "native-park.json"
    route_data = json.loads(path.read_bytes())
    if route_data["field_step_sha256"] != FIRST_SHA or digest(KIT / "stowed.step") != route_data["robot_step_sha256"]:
        raise ValueError("Field and parked robot must match the route registration")
    stations, shapes = hanging_inventory(field)
    # Use the widest actual assembly for every slot, covering a change of cage height.
    section_radius = max(station["radius_mm"] for station in stations.values())
    upper_height = max(station["eye_box_mm"][4] for station in stations.values())
    cases = {}
    for key, route in route_data["cases"].items():
        box = route["native_swept_box_mm"]
        limiting = min(stations, key=lambda s: clearance(stations[s]["axis_mm"], box))
        fixed_axis_gap = clearance(stations[limiting]["axis_mm"], box) - section_radius
        tracking = route["one_percent_wheel_case"]["footprint_error_bound_m"] * 1000
        remaining = fixed_axis_gap - tracking
        cases[key] = {"nearest": limiting, "native_route_box_mm": box, "upright_gap_mm": fixed_axis_gap,
                      "tracking_mm": tracking, "displacement_budget_mm": remaining,
                      "floor_cone_limit_deg": math.degrees(math.atan(remaining / upper_height)) if remaining > 0 else None}
    station = stations[cases["shared-105"]["nearest"]]
    ids = json.loads((KIT / "part-frames.json").read_bytes())
    robot = cad.read_named(KIT / "stowed.step", [{"id": key} for key in ids])
    trial = contact_trial(station, shapes[station["cage"]], route_data["cases"]["shared-105"], robot)
    locked = check_locked_robot(locks, robot, route_data["cases"]["near-tape"])
    if trial["overlap_mm3_by_degrees"]["0"] > 0.05 or max(trial["overlap_mm3_by_degrees"].values()) <= 0.05:
        raise ValueError("Expected a clear registered pose and at least one material collision")
    return {"field_sha256": FIRST_SHA, "robot_sha256": route_data["robot_step_sha256"],
            "route_sha256": digest(path), "stations": stations, "radius_mm": section_radius,
            "upper_height_mm": upper_height, "cases": cases, "tilt_trial": trial, "high_locked_robot": locked}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field", type=Path)
    parser.add_argument("--output", type=Path, default=HERE / "cage-motion.json")
    args = parser.parse_args()
    args.output.write_bytes((json.dumps(measure(args.field), indent=2, allow_nan=False) + "\n").encode())
