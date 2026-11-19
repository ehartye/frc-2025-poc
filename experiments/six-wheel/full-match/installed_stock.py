"""Check the focal robot against the stationary game pieces in the official field assembly."""
import argparse
import json
import math
from pathlib import Path

from match import ROOT, build_park, corrected
from mission.check_branch_exclusion import FIRST_SHA, cad, direct_children, name
from mission.heavy_field_clock import revised_motion
from mission.review_combined_departure import (
    POSES, SuppliedRobot, clearance, digest, enclosing, translated,
)
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

HERE = Path(__file__).resolve().parent
BALL_RADIUS = 16.5 * 25.4 / 2
GROUPS = {"reef-ball": 12, "floor-ball": 6, "floor-pipe": 6}


def field_stock(path):
    if digest(path) != FIRST_SHA:
        raise ValueError("Unreviewed field revision")
    doc, table = cad.xcaf_doc()
    reader = cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(path)) != cad.IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Failed to read native field")
    roots = cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    if roots.Length() != 1 or name(roots.Value(1)) != "FE-2025":
        raise ValueError("Unexpected field root")
    stock, shapes = {}, {}
    for node, ref in direct_children(table, roots.Value(1)):
        part = name(ref)
        if part not in ("GE-25500", "GE-25501"):
            continue
        shape = table.GetShape_s(ref).Moved(table.GetLocation_s(node))
        box = cad.bounds(shape)
        if part == "GE-25500" and abs(box[1]) > .01:
            continue  # The remaining native pipes are at supply stations.
        size = [box[i+3] - box[i] for i in range(3)]
        center = [(box[i+3] + box[i]) / 2 for i in range(3)]
        if part == "GE-25501":
            if any(abs(v - 412.75) > .02 for v in size):
                raise ValueError("Unexpected ball diameter")
            group = "reef-ball" if center[1] > 800 else "floor-ball"
        else:
            if any(abs(v - nominal) > .02 for v, nominal in zip(size, (114.3, 301.625, 114.3))):
                raise ValueError("Expected upright floor pipe")
            group = "floor-pipe"
        key = name(node)
        if key in stock:
            raise ValueError("Duplicate field occurrence")
        stock[key] = {"group": group, "native_center_mm": center, "native_bounds_mm": list(box)}
        shapes[key] = shape
    if {g: sum(obj["group"] == g for obj in stock.values()) for g in GROUPS} != GROUPS:
        raise ValueError("Installed stock disagrees with FE-2025 and the setup roster")
    for key, obj in stock.items():
        if obj["group"] == "floor-ball":
            x, _, z = obj["native_center_mm"]
            below = [p for p in stock.values() if p["group"] == "floor-pipe" and
                     math.dist((x,z), p["native_center_mm"][::2]) < .02]
            if len(below) != 1:
                raise ValueError("Missing support pipe under " + key)
    return stock, shapes


def coordinate(f, obj, pose):
    x, up, z = obj["native_center_mm"]
    wx, wy = f.field_from_native(x / 25.4, z / 25.4)
    dx, dy = f.rot(((wx - pose[0])*1000, (wy - pose[1])*1000), -pose[2])
    return dx, dy, up


def gap_from_distances(gaps, obj):
    if obj["group"] != "floor-pipe":
        return math.hypot(*gaps) - BALL_RADIUS
    box = obj["native_bounds_mm"]
    radius = max(box[3]-box[0], box[5]-box[2]) / 2
    half_height = (box[4]-box[1]) / 2
    return math.hypot(max(0, math.hypot(*gaps[:2])-radius), max(0, gaps[2]-half_height))


def box_gaps(point, box):
    return [max(0, box[i]-point[i], point[i]-box[i+3]) for i in range(3)]


def segment_gap(point, box, obj, kind, demand):
    # For a straight, the box translates on its own X axis. For a turn,
    # enclose every orientation in the disk about the actual chassis pivot.
    if kind == "turn":
        radius = max(math.hypot(x,y) for x in box[::3] for y in box[1::3])
        gaps = (max(0, math.hypot(*point[:2]) - radius), 0, box_gaps(point, box)[2])
    else:
        sweep = (box[0]+min(0,demand), box[1], box[2],
                 box[3]+max(0,demand), box[4], box[5])
        gaps = box_gaps(point, sweep)
    gap = gap_from_distances(gaps, obj)
    if kind != "turn" or gap > 0:
        return gap
    angle = demand
    steps = max(1, math.ceil(abs(angle)/math.radians(1)))
    chord = 2*math.hypot(*point[:2])*math.sin(abs(angle)/(4*steps))
    refined = math.inf
    for j in range(steps):
        h = -angle*(j+.5)/steps
        c, s = math.cos(h), math.sin(h)
        mid = (point[0]*c-point[1]*s, point[0]*s+point[1]*c, point[2])
        dx,dy,dz = box_gaps(mid, box)
        refined = min(refined, gap_from_distances((max(0, math.hypot(dx,dy)-chord),0,dz), obj))
    return max(gap, refined)


def bounds(robot, height, stroke):
    return {part: translated(box, (stroke-370)*clearance.FACTORS[robot.frames[part]],
                             0 if robot.frames[part] == "base" else height-560)
            for part, box in robot.boxes.items()}


def across(*states):
    return {part: enclosing([state[part] for state in states]) for part in states[0]}


def check(f, stock, boxes, segments):
    result = {}
    for i, seg in enumerate(segments):
        for key, obj in stock.items():
            point = coordinate(f, obj, seg["start"])
            demand = seg["demand"] if seg["kind"] == "turn" else seg["demand"]*1000
            g, part = min((segment_gap(point, box, obj, seg["kind"], demand), part)
                          for part, box in boxes.items())
            group = obj["group"]
            if group not in result or g < result[group]["lower_bound_mm"]:
                result[group] = {"lower_bound_mm": g, "body": part, "occurrence": key, "segment": i}
    return result


def stationary(pose):
    return [{"start": pose, "kind": "drive", "demand": 0}]


def old_impact(f, shapes, stock, robot):
    old = revised_motion().routes(-.18)[0]
    historical = json.loads((HERE/"stack-negative.json").read_text())
    t = historical["timeS"]
    (x,y,h), _ = old.at(t)
    ox, oy = f.field_from_native(0,0)
    dx, dy = (ox-x)*1000, (oy-y)*1000
    c, s = math.cos(h), math.sin(h)
    transform = gp_Trsf()
    transform.SetValues(c,0,-s,c*dx+s*dy, -s,0,-c,-s*dx+c*dy, 0,1,0,0)
    hits = []
    nearest = (math.inf, "", "")
    boxes = {k: cad.bounds(v) for k,v in robot.items()}
    control = {}
    for key, shape in shapes.items():
        if stock[key]["group"] == "reef-ball":
            continue
        local = BRepBuilderAPI_Transform(shape, transform, True).Shape()
        box = cad.bounds(local)
        for part, body in robot.items():
            coarse = math.hypot(*(max(0,box[i]-boxes[part][i+3],boxes[part][i]-box[i+3]) for i in range(3)))
            if coarse <= nearest[0]:
                dist = clearance.BRepExtrema_DistShapeShape(body, local)
                if not dist.IsDone():
                    raise ValueError("Native clearance unresolved for "+key)
                nearest = min(nearest, (dist.Value(), key, part))
            if cad.occupied(box, boxes[part]):
                v = cad.intersection(body, local)
                if v > .001:
                    hits.append({"occurrence": key, "body": part, "overlap_mm3": round(v,5)})
        if key == "NAUO194":
            # Keep the installed pipe vertical and translate its axis into front bumper foam.
            foam = boxes["bumper-foam-front"]
            # The overall foam is U-shaped. Its bounding-box midpoint is in the chassis.
            shift = [foam[3]-25-(box[0]+box[3])/2, (foam[1]+foam[4]-box[1]-box[4])/2, 0]
            overlap = cad.intersection(robot["bumper-foam-front"], cad.translated(local,shift))
            if overlap <= .001:
                raise ValueError("The displaced-pipe intersection control failed")
            control = {"occurrence":key, "robot_frame_pipe_shift_mm":shift, "foam_overlap_mm3":overlap}
    return {"auto_s": t, "hits": hits, "minimum_nominal_native_gap_mm": nearest[0],
            "closest_occurrence": nearest[1], "closest_body": nearest[2], "displaced_pipe_control":control}


def run(source, poses):
    f = corrected()
    trial = json.loads((HERE/"field-trial.json").read_text())
    assembly = json.loads((HERE/"red-assembly.json").read_text())
    if (digest(HERE/"field-trial.json") != assembly["full_match_configuration_sha256"] or
        digest(poses/"part-frames.json") != assembly["inventory_sha256"] or
        digest(poses/"rotation-bounds.json") != digest(HERE/"red-kit/rotation-bounds.json")):
        raise ValueError("Different field trial or motion inventory")
    stock, shapes = field_stock(source)
    robot = SuppliedRobot(poses/"part-frames.json", {p: poses/(p+".step") for p in POSES},
                          hashes=assembly["pose_step_sha256"],
                          radial=json.loads((poses/"rotation-bounds.json").read_text()),
                          outer_error=15, grip_error=1.5)
    stowed, receive, transport, place = (bounds(robot, *POSES[p]) for p in ("stowed","receive","transport","place"))
    low = bounds(robot, 560, 0)
    rows = {}
    legs = []
    for i, trip in enumerate(trial["sources"]):
        key = trip["piece_id"]
        auto, loaded, back = f.routes(trip["sector_m"])
        if i == 0:
            legs.append(("autonomous-approach", stowed, auto))
        for label, boxes, pose in (
                ("station-extension", across(stowed, receive), loaded.start),
                ("station-lift", across(stowed, transport), loaded.start),
                ("target-lift", across(transport, low), loaded.end),
                ("target-extension", across(low, place), loaded.end)):
            rows[label+"-"+key] = check(f, stock, boxes, stationary(pose))
        legs.extend([("loaded-"+key, transport, loaded),
                     ("after-"+key, transport, back if i == 0 else build_park(f,loaded.end))])
    issued = trial["hold_variants"]["0"]["field"]
    for (label, boxes, route), record in zip(legs, issued[:len(legs)], strict=True):
        if record["leg"] != label or abs(record["drive_time_s"]-route.duration) > 1e-9:
            raise ValueError("Route differs from issued trial: " + label)
        rows[label] = check(f, stock, boxes, route.segments)
    stowed_shapes = cad.read_named(poses/"stowed.step", [{"id": k} for k in robot.frames])
    witness = old_impact(f, shapes, stock, stowed_shapes)
    witness["earlier_auto_continuous_bounds"] = check(f, stock, stowed, revised_motion().routes(-.18)[0].segments)
    return {"schema": "installed-start-stock-clearance/1", "field_step_sha256": FIRST_SHA,
            "field_trial_sha256": digest(HERE/"field-trial.json"), "robot_pose_step_sha256": robot.hashes,
            "body_count": len(robot.frames), "ball_radius_mm": BALL_RADIUS, "motion_error_mm": robot.errors,
            "stock": stock, "stages": rows, "earlier_route_native_check": witness}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field", required=True, type=Path)
    parser.add_argument("--poses", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(run(args.field, args.poses), indent=2, allow_nan=False)+"\n",
                           encoding="utf-8", newline="\n")

