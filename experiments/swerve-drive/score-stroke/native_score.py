"""Intersect the R4 buttons, rails and a conditional scoring path with registered solids."""

import argparse
import importlib.util
import math
import sys
from pathlib import Path

from loaded_rack import ROOT, SIX, digest, read
import numpy as np
from dynamics_surface import SurfaceTrace

sys.path.insert(0, str(ROOT / "cad/full-swerve-r3"))
sys.path.insert(0, str(SIX))
assembly_spec = importlib.util.spec_from_file_location(
    "r4_assembly", ROOT / "cad/full-swerve-r3/build.py"
)
assembly = importlib.util.module_from_spec(assembly_spec)
assembly_spec.loader.exec_module(assembly)
cad, named, save = assembly.cad, assembly.named, assembly.save
from mission.check_branch_exclusion import FIRST_SHA, direct_children, name
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf

R4 = "abbd8ab83898349bbaddb93f1f0fd8f04bd976e59a910267d29ab1ad45a8c639"
LIP = 3453.050980729168


def move(body, values):
    transform = gp_Trsf()
    transform.SetValues(*values)
    return BRepBuilderAPI_Transform(body, transform, False).Shape()


def wall_from_field(path):
    document, table = cad.xcaf_doc()
    reader = cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(path)) != cad.IFSelect_RetDone or not reader.Transfer(
        document
    ):
        raise ValueError("Could not read field STEP")
    roots = cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    parent, ref = next(
        (a, b)
        for a, b in direct_children(table, roots.Value(1))
        if name(a) == "NAUO36" and name(b) == "GE-25300_Red"
    )
    child, part = next(
        (a, b)
        for a, b in direct_children(table, ref)
        if name(a) == "NAUO1053" and name(b) == "GE-25302"
    )
    return table.GetShape_s(part).Moved(
        table.GetLocation_s(parent).Multiplied(table.GetLocation_s(child))
    )


def cap_overlap(parts, trace):
    rows = []
    for stroke in (21.0, 21.25, 21.375, 21.5, 22.0, 24.0):
        t = np.interp(stroke / 1000, trace.stroke, trace.time)
        q = trace.q[np.searchsorted(trace.time, t)]
        for side, y in enumerate((-66.0, 42.0)):
            surface = trace.at(t, (118 - stroke) / 1000, y / 1000)
            dz = 1000 * (q[0] + 0.118 * q[1] - surface["verticalM"])
            cap = parts[f"receiver-absorber-cap-{side}"]
            row = dict(strokeMm=stroke, side=side, approachMm=dz)
            for key, down in (("issuedCommonMm3", 0.0), ("retracted2p5CommonMm3", 2.5)):
                placed = cad.translated(cap, [-stroke, 0, dz - down])
                row[key] = sum(
                    cad.intersection(placed, parts[k])
                    for k in ("r6-receiver-insert", "r6-polymer-facing")
                )
            rows.append(row)
    return rows


def rail_clearance(parts, trace, wall, sector, error):
    dx, dy, angle = error
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    rows = []
    for travel in (310, 320, 330, 340, 350, 360, 370):
        i = min(np.searchsorted(trace.stroke, travel / 1000), len(trace.q) - 1)
        if travel == 370:
            i = -1
        for setback in (0, 20, 40, 42, 45, 50, 60) if travel == 370 else (0,):
            dock = (
                c,
                -s,
                0,
                LIP - 490 + dx - setback,
                0,
                0,
                1,
                0,
                -s,
                -c,
                0,
                -sector - dy,
            )
            for mode in ("rigid", "loaded"):
                heave, pitch = (0.0, 0.0) if mode == "rigid" else trace.q[i, 4:6]
                u, v = math.cos(pitch), math.sin(pitch)
                shift = (
                    u,
                    0,
                    -v,
                    travel + 467 * v,
                    0,
                    1,
                    0,
                    0,
                    v,
                    0,
                    u,
                    1000 * heave + 467 * (1 - u),
                )
                distances, volumes = [], []
                for side in ("L", "R"):
                    body = move(move(parts[f"drilled-slide-{side}-inner"], shift), dock)
                    distance = BRepExtrema_DistShapeShape(body, wall)
                    if not distance.IsDone():
                        raise ValueError("Wall distance failed")
                    distances.append(distance.Value())
                    volumes.append(
                        cad.intersection(body, wall) if distances[-1] < 1.0e-5 else 0.0
                    )
                rows.append(
                    dict(
                        strokeMm=travel,
                        setbackMm=setback,
                        model=mode,
                        pitchRad=float(pitch),
                        heaveMm=float(1000 * heave),
                        gapMm=distances,
                        commonMm3=volumes,
                    )
                )
    return rows


def release_control(piece, bounds, trace=None, seconds=None):
    # Borrow the previous belt output as an explicit initial condition, never as an R4 event.
    path = SIX / "relieved-match" / f"{piece}-coupled-250.json"
    row = read(path)
    p, v, a, omega = (
        np.array(row[k]) for k in ("positionM", "velocityMps", "axis", "angularRadps")
    )
    # Free symmetric body has constant |da/dt|, including spin about the pipe axis.
    yaw = row["chassisHeadingRad"]
    edge_response = None
    if trace is not None:
        # Settled, fully extended, missing-button counterfactual. The measured
        # center is above the last shelf generator along its surface normal.
        _, local_y, _ = np.array(row["edgeRobotMm"]) / 1000
        local_x = 0.165  # Rim of the actual flat facing; the pipe center can project beyond it.
        edge_response = trace.at(
            trace.time[-1] if seconds is None else seconds, local_x, local_y
        )
        angle = math.atan(edge_response["slopeX"])
        fore = np.array([math.cos(yaw), math.sin(yaw)])
        p[:2] -= fore * 0.05715 * math.sin(angle)
        p[2] += edge_response["verticalM"] + 0.05715 * (math.cos(angle) - 1)
        forward = float(v[:2] @ fore)
        v[:2] += fore * forward * (math.cos(angle) - 1)
        v[2] += forward * math.sin(angle) + edge_response["verticalSpeedMps"]
    rate = np.linalg.norm(np.cross(omega, a))
    wall_front, wall_back = (bounds[0] - LIP) / 1000, (bounds[3] - LIP) / 1000
    top = bounds[4] / 1000
    horizon = 0.4
    ext = 0.05715 + 0.1508125 * np.minimum(1, np.abs(a) + rate * horizon)
    # The pipe also fits within a capsule around its generator. Its projection
    # fits in an XZ disk of this radius throughout the declared time horizon.
    # Unlike independent axis bounds, a single disk cannot occupy its bottom
    # and back extrema at once. Treat the measured wall as infinitely wide.
    radius = 0.05715 + 0.1508125 * min(
        1.0, float(np.linalg.norm(a[[0, 2]])) + rate * horizon
    )

    def crossing(z):
        return (v[2] + math.sqrt(v[2] ** 2 + 2 * 9.80665 * (p[2] - z))) / 9.80665

    t_early, t_late = crossing(top + ext[2]), crossing(top - ext[2])
    cases = []
    cases_to_run = [
        (370, d) for d in (0, 20, 40, 42, 45, 50, 52, 54, 56, 58, 60, 65, 70, 80, 100)
    ]
    if trace is None:
        cases_to_run += [(s, 0) for s in (21.25, 310, 320, 330, 340, 350, 360)] + [
            (21.25, -60)
        ]
    for stroke, setback in cases_to_run:
        x = p[0] + (stroke - 370) * math.cos(yaw) / 1000 - setback / 1000
        clearance = 1000 * (x + v[0] * t_early - ext[0] - wall_back)
        shortfall = 1000 * (wall_front - (x + v[0] * t_late + ext[0]))
        end = max(0.0, (wall_back + radius - x) / v[0]) + 0.001
        coupled = None
        if end <= horizon:
            times = np.linspace(0, end, int(math.ceil(end / 0.001)) + 1)
            xx, zz = x + v[0] * times, p[2] + v[2] * times - 0.5 * 9.80665 * times**2
            dx = np.maximum.reduce(
                [wall_front - xx, xx - wall_back, np.zeros(len(times))]
            )
            dz = np.maximum(0, zz - top)
            speed_bound = math.hypot(v[0], abs(v[2]) + 9.80665 * end)
            # Distance to a closed set is 1-Lipschitz. Charge half the maximum
            # sample spacing at the largest possible center speed on the arc.
            coupled = 1000 * (
                np.hypot(dx, dz).min() - radius - speed_bound * max(np.diff(times)) / 2
            )
        cases.append(
            dict(
                strokeMm=stroke,
                setbackMm=setback,
                farFaceMarginMm=clearance,
                nearFaceShortfallMm=shortfall,
                continuousCapsuleClearanceMm=coupled,
                overflight=(
                    "clear"
                    if clearance > 0 or coupled is not None and coupled > 0
                    else "falls short"
                    if shortfall > 0
                    else "indeterminate"
                ),
            )
        )
    if t_late > horizon or v[0] <= 0:
        raise ValueError("Projection bound used outside its integration interval")
    return dict(
        sourceSha256=digest(path),
        positionM=p.tolist(),
        velocityMps=v.tolist(),
        edge=edge_response,
        wallTopMm=top * 1000,
        wallXFromLipMm=[wall_front * 1000, wall_back * 1000],
        transitBoundsS=[t_early, t_late],
        tubeExtentUpperMm=(1000 * ext).tolist(),
        projectedCapsuleRadiusMm=1000 * radius,
        cases=cases,
    )


def run(field, traces, output):
    robot = ROOT / "cad/full-swerve-r4/full-robot.step"
    if digest(robot) != R4 or digest(field) != FIRST_SHA:
        raise ValueError("Unregistered field or robot")
    parts, wall = named(robot), wall_from_field(field)
    installed = read(SIX / "extension-support/keepers.json")
    for key, volume in installed["volumesMm3"].items():
        if not math.isclose(cad.volume(parts[key])[0], volume, abs_tol=0.01):
            raise ValueError(f"Different registered rail {key}")
    result = dict(
        robotSha256=R4,
        fieldSha256=FIRST_SHA,
        fieldPart="NAUO36/NAUO1053/GE-25302",
        railPartsVerified=len(installed["volumesMm3"]),
        caps={},
        rails={},
        release={},
    )
    nav = read(SIX / "relieved-match/navigation.json")["cases"]
    for piece, sector in (("coral-A", -180), ("coral-B", 220)):
        result["release"][piece] = release_control(piece, cad.bounds(wall))
        error = next(
            x["finalBodyErrorMmDeg"]
            for x in nav
            if x["pipe"] == piece and x["mode"] == "nominal"
        )
        for sign in ("plus", "minus"):
            stem = traces / f"{sign}-issued-31.25us"
            trace = SurfaceTrace(
                Path(str(stem) + ".npz"), read(Path(str(stem) + ".json"))["traceSha256"]
            )
            if piece == "coral-A":
                result["caps"][sign] = cap_overlap(parts, trace)
            stem = traces / f"{sign}-no-buttons-31.25us"
            trace = SurfaceTrace(
                Path(str(stem) + ".npz"), read(Path(str(stem) + ".json"))["traceSha256"]
            )
            result["rails"][f"{piece}-{sign}"] = rail_clearance(
                parts, trace, wall, sector, error
            )
            result["release"][f"{piece}-{sign}-loaded"] = release_control(
                piece, cad.bounds(wall), trace
            )
    save(output, result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("field", "traces", "output"):
        parser.add_argument(key, type=Path)
    args = parser.parse_args()
    run(args.field, args.traces, args.output)
