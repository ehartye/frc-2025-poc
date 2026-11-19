"""Reconcile the source station and camera route with floor stock, peer traffic, and loaded bus."""

import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

from retention import HERE, ROOT, nav, path_for

sys.path.insert(0, str(ROOT / "station-landmark"))
from field import screen, parked_peers
from electrical import screen as electric

DRIVE = nav.read(ROOT / "relieved-match/first-feed.json")["scheduledDriveWallS"]


def brake(times, poses, when, config):
    """Cut the commanded path at the first 5 ms drive service, then decelerate the wheels."""
    i = np.searchsorted(times, when, side="left")
    if i == 0 or i == len(times):
        raise ValueError("Brake event outside moving route")
    t = times[: i + 1].tolist()
    states = poses[: i + 1].tolist()
    step_time = t[-1] - t[-2]
    # Wheel displacement is recovered in physical metres. Both axes see the
    # conservative 0.290 m/s² stand-off brake bound used at the first dock.
    track = config["effectiveTrackMm"] / 1000
    scale = np.array(config["wheelScale"])
    ds = nav.route.wheel_steps(poses[i - 1 : i + 1])[0].mean()
    turn = poses[i, 2] - poses[i - 1, 2]
    speed = (ds + np.array([-1, 1]) * track * turn / 2) / step_time
    initial = speed.copy()
    while np.max(abs(speed)) > 1.0e-7:
        targets = speed + np.clip(-speed, -0.290 * 0.005 * scale, 0.290 * 0.005 * scale)
        dl, dr = 0.5 * (speed + targets) * 0.005
        x, y, h = states[-1]
        turn = (dr - dl) / track
        travel = 0.5 * (dl + dr) * np.sinc(turn / (2 * math.pi))
        states.append(
            [
                x + travel * math.cos(h + 0.5 * turn),
                y + travel * math.sin(h + 0.5 * turn),
                h + turn,
            ]
        )
        t.append(t[-1] + 0.005)
        speed = targets
    return (
        np.array(t),
        np.array(states),
        {
            "firstServiceWallS": DRIVE + times[i],
            "stoppedWallS": DRIVE + t[-1],
            "wheelMpsAtInhibit": initial.tolist(),
            "centerTravelToRestMm": 1000
            * np.linalg.norm(
                np.diff(np.array(states[i:])[:, :2], axis=0), axis=1
            ).sum(),
        },
    )


def reconcile(assets, side, variant, inhibit=None):
    time, pose, trial = path_for(assets, side, variant)
    stop = None
    if inhibit is not None:
        config = nav.read(ROOT / "station-landmark/routes.json")["cases"][side]
        time, pose, stop = brake(time, pose, inhibit - DRIVE, config)
    ta = assets / "conditioned-match" / side / "corrected-opening.npz"
    if (
        nav.sha(ta)
        != nav.read(ROOT / "conditioned-match/opening.json")[side]["pathSha256"]
    ):
        raise ValueError("Different reached station")
    first = np.load(ta)
    if np.max(abs(first["pose"][-1, :2] - pose[0, :2])) > 1.0e-6:
        raise ValueError("Station did not join outbound movement")
    full = np.r_[first["t"], DRIVE + time, 156.0]
    positions = np.vstack((first["pose"], pose, pose[-1]))
    positions[:, 2] = np.unwrap(positions[:, 2])
    grid = np.arange(31201) * 0.005
    xy = np.column_stack([np.interp(grid, full, v) for v in positions.T])
    arcs = nav.route.wheel_steps(xy)
    events = {
        "coral-A": {
            "feed": 21.1,
            "seat": 22.97167263531428,
            "carry": DRIVE,
            "carryBeltSpeedBoundMps": 0.032,
        }
    }
    labels = np.select(
        (grid[:-1] < 21.1, grid[:-1] < DRIVE, grid[:-1] < DRIVE + time[-1]),
        ("opening", "station", "carry"),
        default="stop",
    )
    pack = {}
    for key, battery in (("screened", (11.8, 0.027)), ("weak", (11.8, 0.032))):
        r = electric(
            grid,
            arcs,
            labels,
            events,
            65.83312255,
            "observed score approach",
            battery,
            assets,
        )
        pack[key] = {
            k: r[k]
            for k in (
                "peakMotorPhaseA",
                "peakFrictionRequired",
                "minTerminalV",
                "firstReferenceInhibitWallS",
                "limitingWallS",
                "limitingBranch",
                "commonCurrentAtMinimumA",
                "branchBounds",
            )
        }
    plan = nav.read(ROOT / "fixed-height-match/placements.json")["placements"]
    field, _, team, _ = nav.route.route.field.scenario(plan, inboard=True)
    stock = nav.read(ROOT / "full-match/installed-stock.json")["stock"]
    g = screen(field, DRIVE + time, pose, stock, team, DRIVE + time[-1])
    stable = parked_peers(team, pose[-1], DRIVE + time[-1])
    return {
        "case": side,
        "sensor": variant,
        "inhibitWallS": inhibit,
        "braking": stop,
        "finalWallS": DRIVE + time[-1],
        "cameraDecisionWallS": DRIVE
        + time[-1]
        + (trial["decisionMs"] - trial["actualStopAfterStartMs"]) / 1000
        if trial["decisionMs"] is not None and inhibit is None
        else None,
        "routeSourceSha256": trial["motionSHA256"],
        "finalFieldXYHeading": pose[-1].tolist(),
        "clearance": g,
        "stationaryPeerMm": stable,
        "power": pack,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    p.add_argument("--aborts", action="store_true")
    a = p.parse_args()
    rows = []
    for side in ("leftSmall", "rightSmall"):
        if a.aborts:
            for label in (
                "quantized-stuck-cargo-nominal-active",
                "limited-quantized-stuck-cargo-nominal-active",
            ):
                source = HERE / (label + ".json")
                if not source.exists():
                    continue
                trial = next(r for r in nav.read(source) if r["wheelCase"] == side)
                if trial["firstFault"]:
                    tick = trial["firstFault"][0]
                    row = reconcile(
                        a.assets, side, "nominal", 21.571672635314284 + tick / 1000
                    )
                    row["faultSource"] = label
                    row["pipeTrajectorySha256"] = trial["trajectorySha256"]
                    rows.append(row)
        else:
            for typ in (
                "nominal",
                "wrong-face",
                "dropout",
                "ambiguous",
                "stale",
                "shared-survey-shift",
            ):
                rows.append(reconcile(a.assets, side, typ))
    dest = HERE / ("cargo-stops.json" if a.aborts else "field.json")
    dest.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf8", newline="\n")
