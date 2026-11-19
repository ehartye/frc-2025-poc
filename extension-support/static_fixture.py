"""Calculate stationary vertical displacements under a declared load and keeper stiffness."""

from pathlib import Path
import argparse
import json
import math
from rail_balance import HERE, balance, digest, read


def support_plane(
    load, rear_keeper_x, floor_stiffness, keeper_stiffness, upward_gap=0.35
):
    """For forward tipping, use the rear keeper and front floor as active constraints."""
    if not all(
        math.isfinite(v)
        for v in [rear_keeper_x, floor_stiffness, keeper_stiffness, upward_gap]
    ):
        raise ValueError("Non-finite joint fixture")
    if min(floor_stiffness, keeper_stiffness) <= 0 or upward_gap < 0:
        raise ValueError("Invalid stiffness or running clearance")
    rear, front = load["contactXMm"]
    x, force = load["loadCenterXMm"], load["downwardN"]
    if x < rear:
        raise ValueError("This fixture does not establish rear-overhang support")
    if x <= front:
        a = rear
        rb = force * (x - rear) / (front - rear)
        ra = force - rb
        za = -1.0 - ra / floor_stiffness
        regime = "two lower lands"
    else:
        a = rear_keeper_x
        if not rear <= a < front:
            raise ValueError("Rear keeper lies outside the overlap")
        rb = force * (x - a) / (front - a)
        ra = force - rb
        za = upward_gap - ra / keeper_stiffness
        regime = "rear keeper and front floor"
    zb = -1.0 - rb / floor_stiffness
    return {
        "regime": regime,
        "rearXMm": a,
        "frontXMm": front,
        "rearReactionN": ra,
        "frontReactionN": rb,
        "rearZMm": za,
        "frontZMm": zb,
        "slope": (zb - za) / (front - a),
    }


def height(plane, x):
    return plane["rearZMm"] + plane["slope"] * (x - plane["rearXMm"])


def run():
    src = HERE / "balance.json"
    ledger = read(src)
    if ledger["nativeSha256"] != digest(HERE / "geometry.json"):
        raise ValueError("Different rail geometry")
    pins = read(HERE / "keepers.json")
    if pins["stepSha256"] != digest(HERE / "rail-fixture.step"):
        raise ValueError("Different keeper exchange")
    # Each contact line in the original two-stage hypothesis is 10 MN/m per
    # side. Splitting a line between the two fore/aft contacts, then combining
    # the two sides, gives 10 kN/mm per station. This preserves the seated
    # heave stiffness; it does not measure the flexible webs or the pin seats.
    corners = {"compliant": 1000.0, "nominal": 10000.0, "stiff": 100000.0}
    geometry = read(HERE / "geometry.json")
    geometry["movingStock"].update(pins["movingStock"])
    cases = {}
    for name, case in ledger["cases"].items():
        loads = {
            str(s): balance(geometry, s, case["pipeLocalXMm"], ledger["pipeMassKg"])
            for s in geometry["strokeMm"]
        }
        trials = {}
        for k, stiff in corners.items():
            stations = {}
            for s, r in loads.items():
                stroke = float(s)
                inner = support_plane(
                    r["innerOnMiddle"],
                    pins["innerCrosspinLocalXMm"] + stroke,
                    stiff,
                    stiff,
                )
                middle = support_plane(
                    r["middleOnOuter"], pins["outerDogFixedXMm"], stiff, stiff
                )
                heights = {
                    point: height(inner, stroke + x) + height(middle, stroke + x)
                    for point, x in {
                        "center": case["pipeLocalXMm"],
                        "frontEdge": 165.0,
                    }.items()
                }
                stations[s] = {
                    "inner": inner,
                    "middle": middle,
                    "absoluteZFromHorizontalDatumMm": heights,
                    "combinedSlope": inner["slope"] + middle["slope"],
                }
            for v in stations.values():
                v["frontChangeFromStowMm"] = (
                    v["absoluteZFromHorizontalDatumMm"]["frontEdge"]
                    - stations["0"]["absoluteZFromHorizontalDatumMm"]["frontEdge"]
                )
            trials[k] = stations
        cases[name] = trials
    return {
        "balanceSha256": digest(src),
        "keeperSha256": pins["stepSha256"],
        "stationStiffnessNPerMm": corners,
        "cases": cases,
        "assumptions": [
            "Stationary vertical equilibrium of named stock and pipe, symmetric parallel rails.",
            "Rigid rail spans and small-angle transformations. Positions refer to the undeformed floor witnesses.",
            "One pair of contact resultants per joint. No absorber force, inertia, friction, plate flexure, wear or housing preload.",
            "Input for comparing sensitivity. Dynamic motion and achieved scoring remain unverified.",
        ],
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.parse_args().output.write_text(
        json.dumps(run(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
