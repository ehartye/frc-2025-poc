"""Check stowed R4 park targets against the registered zone and static field CAD."""
import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SIX = ROOT / "experiments/six-wheel"
R4 = ROOT / "cad/full-swerve-r4"
spec = importlib.util.spec_from_file_location("swerve_assembly", ROOT / "cad/full-swerve-r3/build.py")
assembly = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assembly)
sys.path.insert(0, str(SIX / "leave-park"))
import native_park as field
import rehearse as park

# Candidate terminal-pose budget, a design requirement for the later drive replay.
TRANSLATION_MM = 25.0  # independently on each field axis
YAW_DEG = 3.0


def read(path):
    return json.loads(path.read_bytes())


def union(boxes):
    boxes = list(boxes)
    return [min(b[i] for b in boxes) for i in range(3)] + [max(b[i] for b in boxes) for i in range(3, 6)]


def planar_shift(box):
    radius = max(math.hypot(x, y) for x in (box[0], box[3]) for y in (box[1], box[4]))
    return TRANSLATION_MM + 2 * radius * math.sin(math.radians(YAW_DEG) / 2)


def solve(field_step):
    native = read(R4 / "native-reopen.json")
    step = R4 / "full-robot.step"
    appearance = read(R4 / "appearances.json")
    if assembly.sha(step) != native["source"]["publicStepSha256"] or assembly.sha(step) != appearance["sourceSha256"]:
        raise ValueError("R4 STEP and Fusion receipt disagree")
    if assembly.sha(R4 / "appearances.json") != native["source"]["publicRegisterSha256"]:
        raise ValueError("R4 occurrence register changed")
    shapes = assembly.named(step)
    ids = {p["id"] for p in appearance["solids"]}
    if set(shapes) != ids or len(shapes) != 975:
        raise ValueError("R4 named geometry differs from the issued register")
    covers = {key: shape for key, shape in shapes.items() if key.startswith("bumper-cover-")}
    if set(covers) != {"bumper-cover-front-red", "bumper-cover-rear-red"}:
        raise ValueError("Physical red covers missing")
    envelopes = read(HERE.parent / "packaging/envelopes.json")
    hull = union([*(p["boundsMm"] for p in appearance["solids"]), *envelopes["cornerBoundsMm"].values()])
    cover_shift = planar_shift(union(assembly.cad.bounds(s) for s in covers.values()))
    hull_shift = planar_shift(hull)
    f = park.corrected()
    old_targets = read(SIX / "fixed-height-match/native.json")
    targets = {"scoring-return": old_targets["parkPose"], "inboard": old_targets["inboardPose"]}
    targets["offset-for-cage-swing"] = [9.600, targets["inboard"][1], math.pi]
    # Beyond the far edge by more than one chassis half-diagonal.
    targets["outside-control"] = [(park.ZONE[2] + 30) * f.IN, targets["inboard"][1], math.pi]
    boxes = {}
    for name, pose in targets.items():
        route = f.Route(tuple(pose))
        bounds = field.native_box(f, route, hull)
        for axis in (0, 2):
            bounds[axis] -= hull_shift
            bounds[axis + 3] += hull_shift
        boxes[name] = bounds
    leaves, supports = field.field_tree(field_step, union(boxes.values()))
    cage_source = SIX / "leave-park/cage-motion.json"
    cage_inventory = read(cage_source)
    if cage_inventory["field_sha256"] != field.FIRST_SHA or len(cage_inventory["stations"]) != 6:
        raise ValueError("Cage inventory and field revision disagree")
    cage_radius = max(row["radius_mm"] for row in cage_inventory["stations"].values())
    eye_height = max(row["eye_box_mm"][4] for row in cage_inventory["stations"].values())
    z = park.ZONE
    trim = cover_shift / 25.4
    eroded = (z[0]+trim, z[1]+trim, z[2]-trim, z[3]-trim)
    results = {}
    for name, pose in targets.items():
        box = boxes[name]
        prism = field.prism(box)
        nominal = park.overlap(f, covers, pose, z)
        robust = park.overlap(f, covers, pose, eroded)
        nearby = {key: field.distance(prism, shape) for key,(shape,b) in leaves.items()
                  if field.box_distance(box,b) <= field.SEARCH_MM}
        closest = min(nearby, key=nearby.get) if nearby else None
        gap = min([field.SEARCH_MM, *nearby.values()])
        # Conditional on all hanging material staying inside a circular cylinder about its
        # local centerline. At floor level a six-degree centerline cone is widest.
        def cage_clearance(row):
            x, z_native = row["axis_mm"]
            return math.hypot(max(box[0]-x, x-box[3], 0), max(box[2]-z_native, z_native-box[5], 0)) - cage_radius
        upright = min(cage_clearance(row) for row in cage_inventory["stations"].values())
        angle = math.radians(6)
        six_deg = upright - eye_height * math.tan(angle) - cage_radius * (1 / math.cos(angle) - 1)
        results[name] = dict(poseXYMetresYawRadians=pose, nominalClothVolumeMm3=nominal,
                             guaranteedClothVolumeMm3=robust, staticClearanceLowerMm=gap,
                             nearestLeafWithinSearch=closest if gap < field.SEARCH_MM else None,
                             clearanceOutsideSixDegreeCageConeMm=six_deg,
                             staticParkGeometrySupported=sum(robust.values()) > 1e-4 and gap > 0)
    if sum(results["outside-control"]["nominalClothVolumeMm3"].values()) > 1e-4:
        raise ValueError("Outside control unexpectedly reaches the zone")
    files = [cage_source, step, R4/"appearances.json", R4/"native-reopen.json", HERE.parent/"packaging/envelopes.json",
             SIX/"fixed-height-match/native.json", SIX/"mission/field-registration.json",
             SIX/"leave-park/rehearse.py", SIX/"leave-park/native_park.py"]
    return dict(configuration=native["configuration"], pins={p.relative_to(ROOT).as_posix(): assembly.sha(p) for p in files},
                fieldSha256=field.FIRST_SHA, robotExchangeSolids=len(shapes), assessedFieldLeaves=len(leaves),
                supportSurfaces=sorted(supports), fieldSearchRadiusMm=field.SEARCH_MM, hullMm=hull,
                maxTranslationPerAxisMm=TRANSLATION_MM,maxHeadingErrorDeg=YAW_DEG,
                coverEdgeInsetMm=cover_shift, staticHullPaddingMm=hull_shift, zoneFieldIn=list(z),
                hangingCageRadiusMm=cage_radius, highestCageEyeMm=eye_height, cases=results, scope="Endpoint geometry with a stowed robot, registered parked module posture and static field. A separate cage margin assumes a six-degree centerline cone at each installed eye. Actual cage swing, traffic, travel and settlement through scoring time remain open.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("field_step", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()
    data = solve(args.field_step)
    args.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")
    for key, row in data["cases"].items():
        print(key, sum(row["guaranteedClothVolumeMm3"].values()), row["staticClearanceLowerMm"])
