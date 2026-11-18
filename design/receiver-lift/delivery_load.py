"""Roll up the installed moving inventory by identity and re-screen the historical motor."""
import csv
import hashlib
import json
import math
from pathlib import Path

from size_lift import G, INCH, ROTOR_STACK_INERTIA, profile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FRAMES = {"lift", "middle", "slide", "variable-rope", "variable-draw-wire"}
# Paired intervals in kg for the specific bodies absent from both material BOMs.
EXTRAS = {
    "return-line-R": (.001, .010), "return-line-L": (.001, .010),
    "anchor-tab-R": (.011, .035), "anchor-tab-L": (.011, .035),
    "observer-moving-pad-0": (.001, .015), "observer-moving-pad-1": (.001, .015),
    "observer-moving-pad-2": (.001, .015), "qtrx-seat": (.001, .010), "qtrx-entry": (.001, .010),
    "shelf-harness-moving-anchor": (.005, .030), "lift-harness-moving-anchor": (.005, .030),
}


def bom(path, lo, hi):
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    result = {r["id"]: (float(r[lo]), float(r[hi])) for r in rows}
    if len(result) != len(rows):
        raise ValueError("duplicate material entry")
    return result


def audit(package=None):
    base_dir = ROOT / "design"
    package = package or base_dir / "competition-package"
    paths = [package / "part-frames.json", base_dir / "station-l1/bom.csv",
             base_dir / "receiver-drives/bom.csv", base_dir / "electrical-inspection/inspection.json",
             ROOT / "mission/candidate.json"]
    frames = json.loads(paths[0].read_bytes())
    if set(frames.values()) != FRAMES | {"base"}:
        raise ValueError("unexpected motion frame")
    original = bom(paths[1], "minMassKg", "maxMassKg")
    actuated = bom(paths[2], "lowMassKg", "highMassKg")
    budget = json.loads(paths[3].read_bytes())["projectMassAllowancesKg"]
    motion = json.loads(paths[4].read_bytes())["motion"]
    groups = {"retained_base_bom": original, "receiver_drive_bom": actuated, "added_body_allowance": EXTRAS}
    rows = {}
    for name, frame in frames.items():
        if frame == "base":
            continue
        match = [k for k, values in groups.items() if name in values]
        if len(match) != 1:
            raise ValueError("unresolved or multiply counted moving article: " + name)
        group, = match
        rows[name] = {"frame": frame, "basis": group, "kg": groups[group][name]}
    if not EXTRAS.keys() <= rows.keys():
        raise ValueError("unmatched supplemental body allowance")
    # Conservative allocations: deliberately charge the entire installed wiring allowance
    # against lift high, plus unmodeled moving fasteners. No addition to robot mass total.
    wire = budget["power cables, branches, moving service loops, leads and terminations"]
    unmodeled = {"moving_share_of_wire": (0, wire[1]), "misc_moving_fasteners": (0, .35)}
    dry = [sum(r["kg"][j] for r in rows.values()) + sum(v[j] for v in unmodeled.values()) for j in range(2)]
    loaded = [m + motion["pipeMass_kg"] for m in dry]
    radii = [(d + .125) * INCH / 2 for d in (.750, .787)]
    schedule = {}
    for name, travel, seconds, mass in [("receive-carry", .270, 2, loaded[1]), ("carry-place", .110, 1.5, loaded[1]),
                                       ("place-carry", .110, 1, dry[1]), ("carry-receive", .270, 2, dry[1]),
                                       ("one-second-return-control", .270, 1, dry[1])]:
        schedule[name] = [profile(travel, seconds, mass, r, 16, .78, ROTOR_STACK_INERTIA) for r in radii]
    return {"schema": "registered-receiver-delivery-load/1", "counts": {"all":len(frames),"moving":len(rows)},
            "inputs": {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            "groupsKg": {group:[sum(r["kg"][j] for r in rows.values() if r["basis"] == group) for j in range(2)]
                         for group in groups}, "bodyMassKg":rows, "unmodeledKg":unmodeled, "dryKg":dry, "loadedKg":loaded,
            "scheduled":schedule, "staticOutputNmHigh":loaded[1]*G*max(radii), "weighedMassKg":None}


if __name__ == "__main__":
    print(json.dumps(audit(), indent=2, allow_nan=False))
