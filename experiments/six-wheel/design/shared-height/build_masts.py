"""Build common mast stock and compare the two inspection configurations in native CAD."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LOCKS = HERE.parent / "fixed-height"
SOURCE = HERE.parent / "competition-package/shop-duplex/red"
sys.path.insert(0, str(LOCKS))
from native import cad, digest, replace_cut, shared, x_cyl
sys.path.insert(0, str(HERE.parent / "receiver-lift"))
from check_routing import write_step
from duplex import finalize_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps


def read(path):
    return json.loads(path.read_bytes())


def volume(shape):
    return abs(cad.volume(shape)[0])


def difference(a, b):
    """Sum of the volumes occupied by only one of two registered solids."""
    return abs(volume(a) + volume(b) - 2 * shared(a, b))


def section(shape, z):
    # Average over a thin horizontal slice centered on the widest diameter of the hole.
    thickness = .1
    slab = cad.brep(["box", [-100, -300, z - thickness / 2, 100, 600, thickness]])
    cut = BRepAlgoAPI_Common(shape, slab)
    if not cut.IsDone():
        raise ValueError("Could not section the mast")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(cut.Shape(), props)
    matrix = props.MatrixOfInertia()
    area = props.Mass() / thickness
    moments = {f"I{axis}_mm4": matrix.Value(i, i) / thickness - area * thickness**2 / 12
               for axis, i in (("x", 1), ("y", 2))}
    xmin, _, _, xmax, _, _ = cad.bounds(cut.Shape())
    cx = props.CentreOfMass().X()
    modulus = moments["Iy_mm4"] / max(xmax - cx, cx - xmin)
    return {"area_mm2": area, **moments, "fore_aft_modulus_mm3": modulus,
            "stress_MPa_per_100N_at_380mm": 100 * 380 / modulus}


def reopen(path, ids, expected_hash):
    if digest(path) != expected_hash:
        raise ValueError(f"Source revision changed: {path.name}")
    return cad.read_named(path, [{"id": name} for name in ids])


def geometry():
    register = read(LOCKS / "native.json")
    kits = {}
    for mode, case, file in [("high", "red", "locks.step"), ("low", "low-red-place", "low-locks.step")]:
        row = register[case]
        ids = list(row["editedVolumesMm3"]) + list(row["addedVolumesMm3"])
        kits[mode] = reopen(LOCKS / file, ids, row["exportSha256"])
    high, low = kits["high"], kits["low"]
    frames = read(SOURCE / "part-frames.json")
    low_stock = reopen(SOURCE / "place.step", frames, register["low-red-place"]["revision"])
    # Register low-place stock in the high retracted coordinate frame.
    travel = {"lift": 0, "middle": 185, "slide": 370}
    mast_shapes, holes, rail_delta, common = {}, {}, {}, {}
    for side, tag in ((1, "L"), (-1, "R")):
        name = f"backup-{tag}-drilled-mast"
        y = side * 222.5
        trial = {}
        for mode, missing_z in [("high", 422.5), ("low", 802.5)]:
            original = kits[mode][name]
            cut = {name: original}
            replace_cut(cut, name, x_cyl(-71, y, missing_z, 6.05, 52))
            trial[mode] = cut[name]
            probe = cad.brep(x_cyl(-71, y, missing_z, 6, 52))
            holes[f"{mode}-{tag}"] = {
                "single_height_mast_volume_mm3": volume(original),
                "unmodified_sleeve_path_interference_mm3": shared(original, probe),
                "new_sleeve_path_interference_mm3": shared(cut[name], probe),
                "removed_volume_mm3": volume(original) - volume(cut[name]),
                "before_section": section(original, missing_z),
                "after_section": section(cut[name], missing_z),
            }
        common[name] = difference(trial["high"], trial["low"])
        mast_shapes[name] = trial["high"]
        for z in (422.5, 802.5):
            rings = {}
            for x in (-70, -22):
                annulus = cad.brep(["cut", [x_cyl(x, y, z, 12.05, 2), x_cyl(x-1, y, z, 6.05, 4)]])
                rings[str(x)] = volume(annulus) - shared(mast_shapes[name], annulus)
            holes[f"{tag}-{z}"] = {"missing_annular_wall_volume_mm3": rings}
    for name, part in low.items():
        if name not in mast_shapes:
            common[name] = difference(cad.translated(part, [0, 0, 380]), high[name])
    for name in sorted(set(high) - set(low)):
        if name.startswith("stroke-"):
            continue
        stock = cad.translated(low_stock[name], [-travel[frames[name]], 0, 380])
        drilled = high[name]
        rail_delta[name] = {"removed_mm3": volume(stock) - volume(drilled),
                            "outside_low_stock_mm3": abs(volume(drilled) - shared(stock, drilled))}
    pins = {k: volume(v) for k, v in high.items() if k.startswith("stroke-")}
    if max(common.values()) > .01 or max(r["outside_low_stock_mm3"] for r in rail_delta.values()) > .01:
        raise ValueError("The registered solids are not interchangeable")
    out = HERE / "masts.step"
    write_step(out, mast_shapes)
    finalize_step(out)
    reopened = cad.read_named(out, [{"id": k} for k in mast_shapes])
    return {"sources": {p.relative_to(ROOT).as_posix(): digest(p) for p in
                        (LOCKS / "native.json", LOCKS / "locks.step", LOCKS / "low-locks.step",
                         SOURCE / "place.step", SOURCE / "part-frames.json")},
            "mast_step_sha256": digest(out), "critical_holes": holes, "common_solids_mm3": common,
            "rails": rail_delta, "high_only_pin_set_mm3": pins,
            "reopen_difference_mm3": {k: difference(s, reopened[k]) for k, s in mast_shapes.items()}}


if __name__ == "__main__":
    out = geometry()
    (HERE / "native.json").write_bytes((json.dumps(out, indent=2) + "\n").encode())
