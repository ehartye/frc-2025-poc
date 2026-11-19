"""Evaluate vertical force and pitch balance on the actually engaged rail floors."""

import argparse
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
GRAVITY = 9.80665


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reaction(loads, span):
    """Front/rear extreme resultants bound what any compressive contact can do."""
    rear, front = span
    if (
        rear >= front
        or not loads
        or any(not math.isfinite(x) or not math.isfinite(f) or f < 0 for x, f in loads)
    ):
        raise ValueError("Expected finite downward loads and a positive contact length")
    total = sum(f for _, f in loads)
    if total <= 0:
        raise ValueError("Expected a positive downward load")
    center = sum(x * f for x, f in loads) / total
    front_force = total * (center - rear) / (front - rear)
    rear_force = total - front_force
    unsupported_moment = total * max(0, rear - center, center - front) / 1000
    return {
        "downwardN": total,
        "loadCenterXMm": center,
        "contactXMm": span,
        "rearReactionN": rear_force,
        "frontReactionN": front_force,
        "minimumCounterMomentNm": unsupported_moment,
        "minimumHoldDownAtOppositeEndN": max(0, -rear_force, -front_force),
        "compressionOnlyAdmissible": unsupported_moment == 0,
    }


def balance(geometry, stroke, pipe_x, pipe_mass):
    if not math.isfinite(stroke) or str(stroke) not in geometry["contacts"]["L"]:
        raise ValueError("Reopen the native rail contacts for this stroke first")
    if not math.isfinite(pipe_mass) or pipe_mass <= 0 or not math.isfinite(pipe_x):
        raise ValueError("Expected a finite seated pipe state")
    inner = [(pipe_x + stroke, pipe_mass * GRAVITY)]
    middle = []
    for part in geometry["movingStock"].values():
        if part["stage"] == "inner":
            inner.append((part["stowedCenterXMm"] + stroke, part["massKg"] * GRAVITY))
        elif part["stage"] == "middle":
            middle.append(
                (part["stowedCenterXMm"] + stroke / 2, part["massKg"] * GRAVITY)
            )
        else:
            raise ValueError("Unknown load frame")
    result = {}
    for interface, loads in (
        ("innerOnMiddle", inner),
        ("middleOnOuter", inner + middle),
    ):
        # Use the union's fore/aft extrema for an optimistic necessary condition.
        # Unequal lateral load sharing or missing contact can require more restraint.
        spans = [
            geometry["contacts"][side][str(stroke)][interface]["downwardXMm"]
            for side in ("L", "R")
        ]
        result[interface] = reaction(
            loads, [min(s[0] for s in spans), max(s[1] for s in spans)]
        )
    return result


def report():
    geometry_file = HERE / "geometry.json"
    geo = read(geometry_file)
    for name, sha in geo["sources"].items():
        if digest(ROOT / name) != sha:
            raise ValueError(f"Stale native geometry: {name}")
    arrivals = ROOT / "split-guide-acquisition/restart.json"
    limits = ROOT / "integrated-slide/mass.json"
    pipe_mass_source = ROOT / "axial-spin/pipe_axial_contact.py"
    # Load the source project's pipe mass without duplicating the constant.
    import runpy

    pipe_mass = runpy.run_path(str(pipe_mass_source))["M"]
    cases = {}
    for original in read(arrivals)["cases"]:
        state = original.get("beforeFirstWheelService")
        if state is None or original["state"] != "IN_BAND":
            continue
        key = f"{original['side']}-{original['stepS']:g}"
        x = 1000 * state["p"][0]
        cases[key] = {
            "case": original["side"],
            "pipeLocalXMm": x,
            "poseSource": "seated station checkout used as a static fixture load",
            "strokes": {s: balance(geo, s, x, pipe_mass) for s in geo["strokeMm"]},
        }
    if len(cases) != 4:
        raise ValueError("Expected two signed seated cases at both numerical timesteps")
    return {
        "nativeSha256": digest(geometry_file),
        "stationRestartSha256": digest(arrivals),
        "pipeSourceSha256": digest(pipe_mass_source),
        "pipeMassKg": pipe_mass,
        "middleMassBoundSourceSha256": digest(limits),
        "middleStagePublishedMassKg": read(limits)["moving"]["middleMassKg"],
        "cases": cases,
        "supportScope": "quasistatic two-stage pitch; named stock and carried pipe; no credit for pinions, tensile guide stops or counterweights",
        "executedExtension": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.parse_args().output.write_text(
        json.dumps(report(), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
