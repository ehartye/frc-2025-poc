"""Confirm that each actual dock still supplies the matching pipe and modal checkpoint."""

import argparse
import json
from pathlib import Path
import sys
import numpy as np
from plant import ROOT, HERE, read, digest

sys.path.insert(0, str(ROOT / "conditioned-match"))
from station_drive import opening


def run(assets, revised=False, fault=None):
    name = ("revised-opening" if revised else "opening") + (
        "-" + fault if fault else ""
    )
    image = assets / "mounted-guide-vision/set-nominal-fixed.npz"
    stamp = read(ROOT / "mounted-guide-vision/setting-station.json")["set-nominal"][
        "register"
    ]["fixedBackgroundSha256"]
    from station_landmarks import Landing

    out = {}
    for side in ("leftSmall", "rightSmall"):
        strategy = Landing() if revised else None
        t, q, pose, status, when = opening(
            assets,
            side,
            pace=0.94,
            wheel_accel_mm_s2=290,
            registered_depth=(image, stamp),
            observation_policy=strategy,
            station_fault=fault,
        )
        baseline = assets / "conditioned-match" / side / "corrected-opening.npz"
        issued = read(ROOT / "conditioned-match/opening.json")[side]
        if digest(baseline) != issued["pathSha256"]:
            raise ValueError("Opening baseline changed")
        with np.load(baseline) as b:
            grid = b["t"].copy()
            samples = np.column_stack(
                [np.interp(grid, t, x) for x in (q[:, 0], q[:, 1], np.unwrap(q[:, 2]))]
            )
            same = bool(np.array_equal(samples, b["pose"]))
        cache = assets / "straight-guide-match"
        cache.mkdir(exist_ok=True, parents=True)
        path = cache / (side + "-" + name + ".npz")
        np.savez_compressed(path, t=grid, pose=samples)
        out[side] = {
            "pathSha256": digest(path),
            "fieldCameraSha256": stamp,
            "wheelReferenceSha256": digest(baseline),
            "sameTimeAndWheelPoses": same,
            "reachedMmDeg": pose,
            "sourceMmDeg": issued["stationMmDeg"],
            "maxDockDifference": float(
                np.max(abs(np.asarray(pose) - issued["stationMmDeg"]))
            ),
            "status": status,
            "stationWallS": when,
        }
        if strategy is not None:
            out[side].update(features=strategy.audit, poseFits=strategy.fit_events)
        if fault:
            out[side].update(fault=fault)
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("--revised", action="store_true")
    p.add_argument("--fault", choices=["noContrast", "late", "interrupt"])
    a = p.parse_args()
    result = run(a.assets, a.revised, a.fault)
    print([(k, v["status"], v["reachedMmDeg"]) for k, v in result.items()])
    (
        HERE
        / (
            ("revised-opening" if a.revised else "opening")
            + ("-" + a.fault if a.fault else "")
            + ".json"
        )
    ).write_text(json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n")
