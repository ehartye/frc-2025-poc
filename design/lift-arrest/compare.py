"""Check rocker and end-stop alternatives against independently reopened robot solids."""
import argparse
import json
import math
from pathlib import Path
from standoff import cad, digest, shared


def cylinder(x, y, z, radius, height):
    return cad.brep(["cyl", [x, y, z, 0, 0, 1, radius, height]])


def overlaps(a, b):
    return all(min(a[i+3], b[i+3])-max(a[i], b[i]) > 0.001 for i in range(3))


def contact(shape, shapes, bounds):
    box = cad.bounds(shape)
    hits = {}
    for name, other in shapes.items():
        if overlaps(box, bounds[name]):
            volume = shared(shape, other)
            if volume > 0.01:
                hits[name] = round(volume, 5)
    return hits


def tilted_plate(shapes, side, drop):
    # Pin ends are Y +/-252 mm. Rotate about the still-supported line at plate height.
    axis = cad.gp_Ax1(cad.gp_Pnt(0, -side*252, 400), cad.gp_Dir(1, 0, 0))
    tf = cad.gp_Trsf()
    tf.SetRotation(axis, -side * math.asin(drop / 504))
    label = "L" if side == 1 else "R"
    plate = shapes[f"backup-{label}-carriage-plate"]
    tilted = cad.BRepBuilderAPI_Transform(plate, tf, False).Shape()
    return shared(tilted, shapes[f"backup-{label}-service-rail-arm-368"])


def review(folder, coords):
    export = folder/"place.step"
    sha = digest(export)
    if sha != json.loads((folder/"review.json").read_bytes())["place"]["export_sha256"]:
        raise ValueError("STEP file differs from its review")
    frames = json.loads((folder/"part-frames.json").read_bytes())
    shapes = cad.read_named(export, [{"id": k} for k in frames])
    bounds = {k: cad.bounds(v) for k, v in shapes.items()}
    data = {"source": sha, "frames": digest(folder/"part-frames.json"), "count": len(shapes)}
    data["singleSidePlateOverlapMm3"] = {
        str(drop): {side: tilted_plate(shapes, sign, drop) for sign, side in [(1,"left"),(-1,"right")]}
        for drop in (4, 5, 5.679, 6.568)}
    reserves = {}
    for x, y in coords:
        key = f"{x},{y}"
        reserves[key] = {}
        for tag, sign in [("left",1),("right",-1)]:
            # Metric supplier drawing: 79.2 mm body + 17.5 mm free rod, 12.7 mm stroke.
            # The body and hex nut are conservative circular reserves.
            body = cylinder(x, sign*y, 313.5, 10, 79.2)
            nut = cylinder(x, sign*y, 363, 13.3, 8)
            rod = cylinder(x, sign*y, 392.7, 3.15, 7.3)
            release_pin = cad.brep(["cyl", [-171,sign*282,395,1,0,0,4,104]])
            candidates = {"barrel":body, "locknut":nut, "compressedRod":rod}
            reserves[key][tag] = {name: contact(s, shapes, bounds) for name,s in candidates.items()}
            reserves[key][tag]["servicePinOverlapMm3"] = sum(shared(s,release_pin) for s in candidates.values())
            raised_tip = cylinder(x, sign*y, 399, 3.2, 2)
            plate_name = "backup-" + ("L" if sign == 1 else "R") + "-carriage-plate"
            reserves[key][tag]["targetPadWitnessMm3"] = shared(raised_tip,shapes[plate_name])
    data["packagingMm"] = reserves
    return data


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--center", nargs=2, action="append", type=float, required=True)
    args = p.parse_args()
    args.output.write_text(json.dumps(review(args.folder,args.center),indent=2)+"\n", encoding="utf-8",newline="\n")
