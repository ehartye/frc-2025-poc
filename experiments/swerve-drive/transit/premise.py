"""Replay the two moving optical vetoes with a finite robot-owned background."""

import argparse
import json
from pathlib import Path

import numpy as np

from register import read, sha, write
from inside_camera import setup
from fixed_contour import read_pair
import receiver_witness as camera


def backed_views(raw, cache, spans, shade=60):
    fixed, roof, guard = setup(raw, cache)
    camera.sensor.scenes.fixed_pose(camera.O)
    rays = camera.sensor.camera
    wheel = camera.load_wheel_mask(raw / "wheel-visibility.json")
    boxes = []
    for xmin, ymin, zmin, xmax, ymax, zmax in spans:
        t = (xmin - camera.O[0]) / rays.DX
        boxes.append((t, camera.O[2] + t * rays.DZ, ymin, ymax, zmin, zmax))
    views, masks = [], []
    for i, ((gray, depth), eye) in enumerate(zip(fixed, rays.EYES)):
        gray, depth = gray.copy(), depth.copy()
        for t, z, ymin, ymax, zmin, zmax in boxes:
            y = camera.O[1] + eye + t * rays.DY
            hit = (
                (t > 0)
                & (t < depth)
                & (y >= ymin)
                & (y <= ymax)
                & (z >= zmin)
                & (z <= zmax)
            )
            gray[hit] = shade
            depth[hit] = t[hit]
        views.append((gray, depth))
        masks.append(wheel["fixed"][i] | (depth >= 3499))
    return views, roof, {**guard, "fixed": masks}


def possible_overlaps(boxes, solids):
    return sorted(
        {
            s["id"]
            for s in solids
            for box in boxes
            if all(
                box[i] < s["boundsMm"][i + 3] and s["boundsMm"][i] < box[i + 3]
                for i in range(3)
            )
        }
    )


def captured(cargo, stamp):
    row = read(cargo / "contact.json")
    path = cargo / row["file"]
    if sha(path) != row["sha256"]:
        raise ValueError("Tube trace digest mismatch")
    track = np.load(path)
    relative = stamp - row["startWallS"]
    if not track[0, 0] - 1e-8 <= relative <= track[-1, 0] + 1e-8:
        raise ValueError("Exposure outside the acquired trace")
    state = np.array([np.interp(relative, track[:, 0], col) for col in track[:, 1:7].T])
    return (
        1000 * state[:3],
        state[3:] / np.linalg.norm(state[3:]),
        1000 * track[0, 1:4],
        row,
    )


def run(raw, transit, output):
    manifest = camera.PROJECT / "cad/full-swerve-r4/appearances.json"
    solids = read(manifest)["solids"]
    spans = [[355, -275, 190, 355.25, 275, 425]]
    narrow = [[355, y, 190, 355.25, y + 105, 425] for y in (-275, 170)]
    layouts = {"issued": [], "two-side-strips": narrow, "full-sheet": spans}
    background = {
        name: backed_views(raw, transit / "native-scene", boxes)
        for name, boxes in layouts.items()
    }
    background["low-contrast-sheet"] = backed_views(
        raw, transit / "native-scene", spans, shade=195
    )
    cases = []
    for sign in ("plus", "minus"):
        folder = transit / f"{sign}-31.25"
        clock = read(folder / "internal-clock.json")
        frame = next(f for f in clock["frames"] if not f["accepted"])
        center, axis, initial, trace = captured(
            folder / "cargo-probe", frame["captureWallS"]
        )
        trials = {}
        for name, (fixed, roof, mask) in background.items():
            trials[name] = camera.observe(
                fixed, roof, center, axis, wheel_mask=mask, fixed_reader=read_pair
            )
        if json.loads(json.dumps(trials["issued"])) != {
            k: frame[k] for k in trials["issued"]
        }:
            raise ValueError("Failed to reproduce the actual veto")
        cases.append(
            dict(
                case=sign,
                captureWallS=frame["captureWallS"],
                decisionWallS=clock["decisionWallS"],
                clockSha256=sha(folder / "internal-clock.json"),
                traceSha256=trace["sha256"],
                piece=trace["piece"],
                centerMm=center.tolist(),
                axis=axis.tolist(),
                travelSinceDepartureMm=float(np.linalg.norm(center - initial)),
                trueBottomRelativeMm=float(center[2] - 557.15),
                trials=trials,
            )
        )
    from custody_rays import ray_screen

    hard_path = transit / "minus-31.25/cargo-12/contact.json"
    hard = read(hard_path)
    write(
        output,
        dict(
            r4=read(manifest)["sourceSha256"],
            scene=sha(transit / "native-scene/scene.json"),
            unobservedDriveControl=dict(
                sourceSha256=sha(hard_path),
                hardStopWallS=hard["startWallS"] + hard["endRelativeS"],
                travelMm=hard["peakTravelMm"],
            ),
            nonImaging=ray_screen(cases, hard, solids),
            layouts={
                k: dict(
                    boxesMm=b, stowedBroadphaseCandidates=possible_overlaps(b, solids)
                )
                for k, b in layouts.items()
            },
            cases=cases,
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("raw", "transit", "output"):
        p.add_argument(n, type=Path)
    a = p.parse_args()
    run(a.raw, a.transit, a.output)
