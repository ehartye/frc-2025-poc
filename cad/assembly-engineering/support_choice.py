"""Compare centered and eccentric suspension against actual cage and chassis solids."""

import argparse
import itertools
import json
import math
from pathlib import Path
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf
from installed_audit import HERE, ROOT, bounds, box_gap, reopen, sha
from attachments import common
from clearance import hanging_inventory, move
from lift import CG_BOUNDS, MASS, G


def pivot(cg, x, travel):
    # Simple swivel on the top face of the cage bottom plate. Free body takes
    # moment-free equilibrium under gravity, with 0.1 kg chain/eye effects omitted.
    # The shallow plate stays at its field height. The commanded lift moves the
    # robot's local hook down. Full square hull includes the lower wheel contact.
    rh = 0.7874 - travel / 1000
    dx, dy = cg[0] - x, cg[1]
    radius = math.hypot(dx, dy)
    theta = math.atan2(radius, rh - cg[2])
    nx, ny = (-dx / radius, -dy / radius) if radius else (0, 0)
    floor = min(
        0.7874 + ((px - x) * nx + py * ny) * math.sin(theta) - rh * math.cos(theta)
        for px, py in itertools.product((-0.425, 0.425), repeat=2)
    )
    return dict(freeAngleDeg=math.degrees(theta), floorMm=floor * 1000)


def run(field):
    stations, shapes = hanging_inventory(field)
    issued = {
        k: s
        for k, g, _, _, _, s in reopen()
        if not (
            g
            in (
                "Shallow cage jack frame",
                "Left cage shoe",
                "Right cage shoe",
                "Lower plate clamp",
                "Cage crosshead",
            )
            or g.startswith("Cage")
        )
    }
    # Compare the existing swerve/receiver independently of the hypothetical
    # relocated climber. The front jack itself has different named motion groups.
    inventory = json.loads((ROOT / "cad/swerve-captive/appearances.json").read_bytes())
    base_ids = {r["id"] for r in inventory["solids"]}
    issued = {k: s for k, s in issued.items() if k in base_ids}
    assert len(issued) == 1001
    change = json.loads((HERE / "interfaces.json").read_bytes())
    from clearance import base

    additions = base.named(HERE / "station-and-jack-interfaces.step")
    exclude = set(change["fixedHeight"]["removed"])
    excluded_jack = {"Left rail saddle", "Right rail saddle"}
    issued = {
        k: s
        for k, s in (
            issued | {k: s for k, s in additions.items() if k not in excluded_jack}
        ).items()
        if k not in exclude
    }
    boxes = {k: bounds(s) for k, s in issued.items()}
    cases = {}
    for side, key, sgn in (("red", "NAUO260", -1), ("blue", "NAUO263", 1)):
        station = stations[key]
        fx, fz = station["axis_mm"]
        values = {}
        for cx, cy in (
            (300, 0),
            (200, 0),
            (150, 0),
            (100, 0),
            (50, 0),
            (0, 0),
            (0, 75),
            (0, -75),
        ):
            t = gp_Trsf()
            t.SetValues(sgn, 0, 0, cx - sgn * fx, 0, 0, -sgn, cy + sgn * fz, 0, 1, 0, 0)
            cage = shapes[station["cage"]].Moved(TopLoc_Location(t))
            states = {}
            for dz in (0, 100, 200, 319.4):
                shape = move(cage, (0, 0, -dz))
                b = bounds(shape)
                overlaps = {}
                for k, s in issued.items():
                    if box_gap(b, boxes[k]) > 0.001:
                        continue
                    v = common(shape, s)
                    if v > 0.01:
                        overlaps[k] = round(v, 3)
                states[str(dz)] = overlaps
            x = cx / 1000
            floor = [
                pivot((u, v - cy / 1000, w), x, 319.4)
                for u, v, w in itertools.product(*CG_BOUNDS)
            ]
            values[f"{cx},{cy}"] = dict(
                cageBody=station["cage"],
                intersectionsDuringSampledVerticalSweep=states,
                momentFreeFinalMinimumFloorMm=min(r["floorMm"] for r in floor),
                momentFreeFinalMaximumTiltDeg=max(r["freeAngleDeg"] for r in floor),
                maximumLevelMomentNm=MASS
                * G
                * max(
                    math.hypot(u - x, v - cy / 1000)
                    for u, v, w in itertools.product(*CG_BOUNDS)
                ),
            )
        cases[side] = values
    return dict(
        fieldSha256=sha(field),
        receiverStepSha256=sha(ROOT / "cad/swerve-captive/public-robot.step"),
        changedInstallationStepSha256=change["outputSha256"],
        robotBodies=len(issued),
        massKg=MASS,
        assumedCGBoundsM=CG_BOUNDS,
        cases=cases,
        scope="Four sampled vertical positions, level chassis. A positive material intersection rejects the aligned version. A clear sample alone does not qualify the sweep. Free-hinge results apply to a separate moment-free support and screen carpet only.",
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("field", type=Path)
    (HERE / "support-choice.json").write_bytes(
        (json.dumps(run(ap.parse_args().field), indent=2) + "\n").encode()
    )
