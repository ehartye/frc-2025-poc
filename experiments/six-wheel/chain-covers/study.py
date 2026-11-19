"""Pack removable chain-cover skins into the released wheel bays, in millimeters."""
import argparse
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "drive-side-attachment"))
import joint as shop


END_AXLE_X = 247.65
ADJUST = 4.0
SPLIT_Z = 76.2
HALF_LENGTH = 292.0
BOTTOM = 34.0
TOP = 120.0
RADIUS = 6.0


def rounded_rectangle(sy, y0, y1, half_length, bottom, top, radius):
    """Solid prism with the four longitudinal-profile corners at constant radius."""
    shape = shop.box(-half_length + radius, half_length - radius, sy*y0, sy*y1, bottom, top)
    middle = shop.box(-half_length, half_length, sy*y0, sy*y1, bottom + radius, top - radius)
    shape = shop.fuse(shape, middle)
    for x in (-half_length + radius, half_length - radius):
        for z in (bottom + radius, top - radius):
            shape = shop.fuse(shape, shop.rod(x, sy, y0, y1, z, radius))
    return shape


def axle_opening(x, sy, travel, radius):
    """A continuous capsule around the 18 mm spacer through all take-up positions."""
    opening = shop.rod(x - travel, sy, 268, 278, SPLIT_Z, radius)
    if travel:
        bridge = shop.box(x-travel, x+travel, sy*268, sy*278, SPLIT_Z-radius, SPLIT_Z+radius)
        opening = shop.fuse(shop.fuse(opening, bridge),
                            shop.rod(x + travel, sy, 268, 278, SPLIT_Z, radius))
    return opening


def skins(side, sy, gauge, face_y, bore_radius):
    face_in = face_y - gauge/2
    band = rounded_rectangle(sy, 235.4, face_in, HALF_LENGTH, BOTTOM, TOP, RADIUS)
    inside = rounded_rectangle(sy, 235, face_in+1, HALF_LENGTH-gauge,
                               BOTTOM+gauge, TOP-gauge, RADIUS-gauge)
    surround = shop.cut(band, inside)
    face = rounded_rectangle(sy, face_in, face_y+gauge/2, HALF_LENGTH, BOTTOM, TOP, RADIUS)
    for x in (-END_AXLE_X, 0, END_AXLE_X):
        face = shop.cut(face, axle_opening(x, sy, ADJUST if x else 0, bore_radius))
    upper = shop.cut(face, shop.box(-300, 300, sy*268, sy*278, 30, SPLIT_Z))
    lower = shop.cut(face, shop.box(-300, 300, sy*268, sy*278, SPLIT_Z, 125))
    # An inner skirt closes the gap under the inner rail. It stops at the rail height.
    skirt = shop.box(-HALF_LENGTH+RADIUS, HALF_LENGTH-RADIUS, sy*235.4, sy*(235.4+gauge),
                     BOTTOM+gauge, 50)
    return {f"{name}-{side}": shape for name, shape in
            (("perimeter", surround), ("upper-face", upper), ("lower-face", lower), ("skirt", skirt))}


def redistribute_spacers(scene):
    """Move both front-loop sprockets together, preserving each axle's spacer stock volume."""
    changed = {}
    for side, sy in (("L", 1), ("R", -1)):
        for stem in ("chain-1-", "sprocket-1-a-", "sprocket-1-b-"):
            key = stem + side
            changed[key] = shop.cad.translated(scene[key], [0, -3*sy, 0])
        for pos, x, first in (("center", 0, 1), ("front", END_AXLE_X, 0)):
            for index in (first, first+1):
                key = f"spacer-{pos}-{index}-{side}"
                bounds = shop.cad.bounds(scene[key])
                low, high = sorted((sy*bounds[1], sy*bounds[4]))
                if index == first:
                    high -= 3
                else:
                    low -= 3
                changed[key] = shop.cut(shop.rod(x, sy, low, high, SPLIT_Z, 9),
                                        shop.rod(x, sy, low-1, high+1, SPLIT_Z, 7.1))
    return changed


def on_side(k, side):
    return k.endswith("-" + side)


def repack_contacts(original, revised):
    """Check every revised body against its installed neighbors in both native scenes."""
    installed = {**original, **revised}
    boxes = {k: shop.cad.bounds(v) for k, v in installed.items()}
    initial_boxes = {k: shop.cad.bounds(v) for k, v in original.items()}
    checked = set()
    overlaps = {}
    for name in revised:
        for other in installed:
            pair = tuple(sorted((name, other)))
            if name == other or pair in checked:
                continue
            if (shop.lower_bound(boxes[name], boxes[other]) > 0.001 and
                    shop.lower_bound(initial_boxes[name], initial_boxes[other]) > 0.001):
                continue
            checked.add(pair)
            old = shop.cad.intersection(original[name], original[other])
            new = shop.cad.intersection(installed[name], installed[other])
            if max(old, new) > 0.001:
                overlaps["/".join(pair)] = [round(old, 5), round(new, 5)]
    spacings = {}
    for side in ("L", "R"):
        for pos, end, index in (("center", "a", 1), ("front", "b", 0)):
            sprocket = installed[f"sprocket-1-{end}-{side}"]
            for neighbor in (index, index+1):
                key = f"spacer-{pos}-{neighbor}-{side}"
                spacings[key] = round(shop.distance(installed[key], sprocket), 5)
    return {"broadPhasePairs": len(checked), "nativeOverlapBeforeAfterMm3": overlaps,
            "adjacentSpacerToSprocketMm": spacings}


def study(artifacts, out):
    out.mkdir(parents=True, exist_ok=True)
    scene, pins = shop.load_source(artifacts, ROOT / "axle-retention")
    from fit import add_exchange
    joints = ROOT / "drive-side-attachment"
    receipt = shop.read(joints / "fit.json")
    step = joints / "inner-rail-joints.step"
    add_exchange(scene, step, receipt["volumeMm3"], receipt["stepSha256"])
    pins["drive-side-attachment/inner-rail-joints.step"] = shop.digest(step)
    mass_file = ROOT / "sp1-mount/optics-mass.json"
    pins["sp1-mount/optics-mass.json"] = shop.digest(mass_file)
    budget_g = 1000 * shop.read(mass_file)["representedMarginKg"][0]
    results = {"sourceSha256": pins, "sceneBodies": len(scene), "availableGrams": budget_g, "cases": {}}
    # Product-sheet typical density. No mass credit for omitted mounting holes or hardware.
    changed = redistribute_spacers(scene)
    revised_step = out / "front-loop-repack.step"
    shop.write_step(revised_step, changed)
    shop.finalize_step(revised_step)
    revised = shop.cad.read_named(revised_step, [{"id": k} for k in changed])
    results["repack"] = {"stepSha256": shop.digest(revised_step), "volumeChangesMm3":
                         {k: round(shop.cad.volume(v)[0]-shop.cad.volume(scene[k])[0], 6)
                          for k, v in revised.items()}}
    if abs(sum(results["repack"]["volumeChangesMm3"].values())) > 0.01:
        raise ValueError("Repacking altered net stock volume")
    results["repack"]["fit"] = repack_contacts(scene, revised)
    original_scene = scene
    for case_name, gauge, face_y, bore in (("original", 0.5, 275, 11),
                                           ("repacked-light", 0.5, 272.6876, 12),
                                           ("repacked-thick", 0.75, 272.6876, 12)):
        scene = original_scene if case_name == "original" else {**original_scene, **revised}
        parts = {k: v for side, sy in (("L", 1), ("R", -1))
                 for k, v in skins(side, sy, gauge, face_y, bore).items()}
        step = out / f"{case_name}.step"
        shop.write_step(step, parts)
        shop.finalize_step(step)
        reopened = shop.cad.read_named(step, [{"id": k} for k in parts])
        volumes = {k: round(shop.cad.volume(v)[0], 5) for k, v in reopened.items()}
        if max(abs(v-shop.cad.volume(parts[k])[0]) for k, v in volumes.items()) > 0.001:
            raise ValueError("Exchanged solids lost material")
        grams = sum(volumes.values())*0.0012
        case = {"gaugeMm": gauge, "faceCenterAbsYmm": face_y,
                "stepSha256": shop.digest(step), "volumeMm3": volumes, "sheetGrams": grams,
                "gramsLeftForAttachmentsAndAllOtherUnbookedMass": budget_g-grams,
                "installed": {}, "drive": {}, "continuousDriveLowerMm": {}, "negativeControls": {}}
        for name, shape in reopened.items():
            case["installed"][name] = shop.prove(shop.clearance(shape, scene))
        for side, sy in (("L", 1), ("R", -1)):
            moving = {k: v for k, v in scene.items() if on_side(k, side) and
                      k.startswith(("wheel-", "chain-", "sprocket-", "spacer-", "axle-"))}
            # Sample at 2 mm spacing. Each translated component is at most 1 mm
            # from a sampled pose. The different end axles may move independently.
            shifts = (-ADJUST, -2, 0, 2, ADJUST)
            for shift in shifts:
                shifted = {}
                for key, shape in moving.items():
                    fixed = ("-center-" in key or key.startswith(("sprocket-0-b-", "sprocket-1-a-")))
                    shifted[key] = shape if fixed else shop.cad.translated(shape, [shift, 0, 0])
                for name, shape in reopened.items():
                    if on_side(name, side):
                        case["drive"][f"{name}/{shift:g}"] = shop.prove(shop.clearance(shape, shifted))
            for name in reopened:
                if on_side(name, side):
                    sampled = min(case["drive"][f"{name}/{s:g}"]["nearestMm"] for s in shifts)
                    case["continuousDriveLowerMm"][name] = round(sampled - 1.001, 5)
            outer = reopened["upper-face-"+side]
            for key, delta, witness in (("inward", -4, "chain-1-"),
                                        ("outward", 4, "wheel-hardware-thread-center-")):
                hit = shop.cad.intersection(shop.cad.translated(outer, [0, sy*delta, 0]), scene[witness+side])
                if hit < 0.01:
                    raise ValueError(f"Control {key} failed to produce interference")
                case["negativeControls"][key+"-"+side] = round(hit, 5)
        results["cases"][case_name] = case
    shop.write(out / "study.json", results)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("--out", type=Path, default=HERE)
    args = parser.parse_args()
    study(args.artifacts.resolve(), args.out.resolve())
