"""Sweep signed final approaches against installed stock, barriers and the other five robots."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from observer import ROOT, nav
import sys

sys.path.insert(0, str(ROOT / "design/station-l1"))
import build_assembly as cad
from field_screen import DRIVE
from field import screen

HERE = Path(__file__).resolve().parent


def check(assets, trials):
    baseline = ROOT / "observed-reef-a/machined-bounds.json"
    boxes = nav.read(baseline)["boxesMm"]
    guide = ROOT / "guide-mounts/guide-mounts.step"
    source = nav.read(ROOT / "guide-mounts/fit.json")
    if nav.sha(guide) != source["stepSha256"]:
        raise ValueError("Guide CAD changed")
    parts = cad.read_named(guide, [{"id": key} for key in source["volumesMm3"]])
    for key, shape in parts.items():
        boxes["mounted/" + key] = cad.bounds(shape)
    boxes["carried-pipe-enclosure"] = nav.read(ROOT / "sector-alignment/scene.json")[
        "carryBoxMm"
    ]
    names = list(boxes)
    limits = np.array(list(boxes.values()), float)
    radial = np.hypot(
        np.maximum(abs(limits[:, 0]), abs(limits[:, 3])),
        np.maximum(abs(limits[:, 1]), abs(limits[:, 4])),
    )
    allowance = 10 + 2 * radial * math.sin(math.radians(0.5))
    stock = nav.read(ROOT / "full-match/installed-stock.json")
    f, _, team = nav.route.route.field.scenario(
        nav.read(ROOT / "fixed-height-match/placements.json")["placements"],
        inboard=True,
    )[:3]
    x, z, y = stock["stock"]["NAUO201"]["native_center_mm"]
    bx, by = f.field_from_native(x / 25.4, y / 25.4)
    result = []
    for case in trials:
        side = case["side"]
        qsource = (
            assets
            / "reef-inertial-observer/sampled"
            / side
            / ("coral-A-" + case["mode"] + ".csv")
        )
        if nav.sha(qsource) != case["motionSHA256"]:
            raise ValueError("Changed wheel motion")
        q = np.loadtxt(qsource, delimiter=",", skiprows=1)
        cached = assets / "conditioned-match" / side / "slow/A/transit.npz"
        registration = nav.read(ROOT / "conditioned-match/slow-navigation.json")[side]
        if nav.sha(cached) != registration["transitSha256"]:
            raise ValueError("Unregistered prior transit")
        prior = np.load(cached)
        times = DRIVE + prior["times"][-1] + q[:, 0] / 1000
        motion = q[:, 1:].copy()
        motion[:, 0:2] /= 1000
        motion[:, 0] += nav.corrected().LIP[0] - 0.490
        motion[:, 1] += nav.corrected().LIP[1]
        motion[:, 2] += (
            round((prior["poses"][-1, 2] - motion[0, 2]) / (2 * math.pi)) * 2 * math.pi
        )
        if np.max(abs(motion[0] - prior["poses"][-1])) > 1e-7:
            raise ValueError("Chassis teleportation")
        arc = np.r_[1000 * np.linalg.norm(np.diff(motion[:, :2], axis=0), axis=1), 0.0]
        turns = np.r_[np.abs(np.diff(motion[:, 2])), 0.0]
        best = (math.inf, None)
        for idx, (qx, qy, h) in enumerate(motion):
            c, s = math.cos(h), math.sin(h)
            dx, dy = 1000 * (bx - qx), 1000 * (by - qy)
            here = [c * dx + s * dy, -s * dx + c * dy, z]
            gap = np.maximum(np.maximum(limits[:, :3] - here, here - limits[:, 3:]), 0)
            bound = (
                np.linalg.norm(gap, axis=1)
                - stock["ball_radius_mm"]
                - allowance
                - arc[idx]
                - radial * turns[idx]
                - 0.5
            )
            item = np.argmin(bound)
            if bound[item] < best[0]:
                best = float(bound[item]), names[item]
        scene = screen(f, times, motion, stock["stock"], team, times[-1])
        margin = 10 + 2 * math.hypot(425, 425) * math.sin(math.radians(0.5))
        movement = np.max(arc + math.hypot(425, 425) * turns)
        result.append(
            dict(
                side=side,
                mode=case["mode"],
                motionSha256=case["motionSHA256"],
                wallStartS=float(times[0]),
                wallStopS=float(times[-1]),
                boxes=len(boxes),
                guideCount=len(parts),
                guideSha256=nav.sha(guide),
                earlierBoundsSha256=nav.sha(baseline),
                installedBallMm=best[0],
                limitingBox=best[1],
                otherStockMm=min(
                    v for k, v in scene["stockMm"].items() if k != "NAUO201"
                )
                - margin
                - movement,
                barrierMm=scene["minimumBarrierMm"] - margin - movement,
                peerMm=scene["minimumPeerMm"] - margin - movement,
                maximumAngularAccelerationRadS2=float(
                    max(abs(np.diff(q[:, 3], n=2))) / 0.005**2
                ),
            )
        )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    args = parser.parse_args()
    trials = json.loads((HERE / "results.json").read_bytes())
    (HERE / "clearance.json").write_text(
        json.dumps(check(args.artifacts, trials), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
