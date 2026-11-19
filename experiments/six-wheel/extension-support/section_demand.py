"""Resolve pitch demand and sampled bending sections of the slotted middle channels."""

from pathlib import Path
import argparse
import math
import json
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from rail_geometry import HERE, ROOT, read, exchange, digest, cad, common
from tray import box
from static_fixture import support_plane
from rail_balance import GRAVITY, reaction


def section(shape, x):
    # The narrow slice is locally prismatic away from slot ends. Its mass Iyy
    # per width contains the small contribution of its own fore-aft thickness.
    w = 0.10
    cut = common(shape, box(x - w / 2, -250, 400, x + w / 2, 250, 510))
    prop = GProp_GProps()
    BRepGProp.VolumeProperties_s(cut, prop)
    area = prop.Mass() / w
    mid = prop.CentreOfMass().Z()
    lower, upper = cad.bounds(cut)[2::3]
    i = prop.MatrixOfInertia().Value(2, 2) / w - area * w * w / 12
    if area <= 0 or i <= 0:
        raise ValueError("Empty rail section")
    return {
        "areaMm2": area,
        "neutralZMm": mid,
        "inertiaMm4": i,
        "modulusMm3": i / max(upper - mid, mid - lower),
    }


def pin_demand(force, lever, diameter, bearing_length):
    return {
        "forceN": force,
        "shankDiaMm": diameter,
        "armMm": lever,
        "nominalBendingMPa": 32 * force * lever / (math.pi * diameter**3),
        "averageShearMPa": 4 * force / (math.pi * diameter**2),
        "dogBearingMPa": force / (4 * bearing_length),
    }


def evaluate(inner_loads, mid_loads, sections, native, pins):
    # All the load is placed on a single side for this fixture. The combined
    # reactants still balance weight and pitch if the two rails share it.
    s = 370
    r = native["contacts"]["L"][str(s)]

    def support(loads, key, keeper):
        return support_plane(
            reaction(loads, r[key]["downwardXMm"]), keeper, 10000, 10000
        )

    i = support(inner_loads, "innerOnMiddle", pins["innerCrosspinLocalXMm"] + s)
    m = support(inner_loads + mid_loads, "middleOnOuter", pins["outerDogFixedXMm"])
    # Loads on the physical middle member. Positive forces point downward.
    loads = [
        (i["rearXMm"], i["rearReactionN"]),
        (i["frontXMm"], i["frontReactionN"]),
        (m["rearXMm"], -m["rearReactionN"]),
        (m["frontXMm"], -m["frontReactionN"]),
        *mid_loads,
    ]
    if abs(sum(f for _, f in loads)) > 1e-7 or abs(sum(x * f for x, f in loads)) > 1e-5:
        raise ValueError("Middle stage does not balance")
    trials = {}
    for x, sect in sections.items():
        station = x + s / 2
        moment = sum((p - station) * force for p, force in loads if p > station)
        trials[x] = {
            "momentNmm": moment,
            "nominalBendingMPa": abs(moment) / sect["modulusMm3"],
        }
    # One nose on the loaded side carries its entire stage reaction, at the
    # maximum tip lever distance from the supported seat (groove for axle).
    return {
        "innerPin": pin_demand(max(0, -i["rearReactionN"]), 13.65 - 6.375, 3.5, 2.65),
        "outerDog": pin_demand(max(0, -m["rearReactionN"]), 15 - 11.35, 4.0, 2.65),
        "sections": trials,
    }


def run():
    native = read(HERE / "geometry.json")
    pins = read(HERE / "keepers.json")
    stock = exchange(HERE / "rail-fixture.step", pins["stepSha256"], pins["volumesMm3"])
    xs = [
        -145.0,
        -137.0,
        -100.0,
        -79.0,
        -40.0,
        -25.0,
        0.0,
        40.0,
        48.0,
        60.0,
        100.0,
        140.0,
    ]
    sections = {x: section(stock["slide-L-middle"], x) for x in xs}
    original = read(HERE / "balance.json")
    pipe_x = next(iter(original["cases"].values()))["pipeLocalXMm"]
    masses = {**native["movingStock"], **pins["movingStock"]}
    inner = [(pipe_x + 370, original["pipeMassKg"] * GRAVITY)]
    middle = []
    for a in masses.values():
        if a["stage"] == "inner":
            inner.append((a["stowedCenterXMm"] + 370, a["massKg"] * GRAVITY))
        else:
            middle.append((a["stowedCenterXMm"] + 185, a["massKg"] * GRAVITY))
    load_file = ROOT / "fixed-lift-omission/mechanics.json"
    full = read(load_file)["oneSideLockLoadKg"]
    return {
        "stepSha256": pins["stepSha256"],
        "sourceLoadSha256": digest(load_file),
        "nativeGrossSections": sections,
        "namedStockAndSeatedPipe": evaluate(inner, middle, sections, native, pins),
        "frontEdgeSingleSideControl": {
            "massKg": full,
            "loadWorldXMm": 535,
            **evaluate([(535, full * GRAVITY)], [], sections, native, pins),
        },
        "scope": "stationary, one-side load; demand only, sampled gross sections; each retaining feature takes its whole interface load",
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    p.parse_args().out.write_text(
        json.dumps(run(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
