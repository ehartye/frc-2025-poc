"""Compare two solver steps for the same signed routes, without interpolating camera events."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def source(path, expected):
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise ValueError("Different trajectory or image stream")
    return path


def load_run(assets, row):
    directory = assets / "observed-reef-a"
    trajectory = np.load(
        source(directory / (row["id"] + ".npy"), row["trajectorySha256"]), mmap_mode="r"
    )
    frames = json.loads(
        source(
            directory / (row["id"] + "-frames.json"), row["framesSha256"]
        ).read_bytes()
    )
    return trajectory, frames


def compare(assets):
    result = []
    for prefix in ("", "machined-roof-"):
        study = prefix + "limited-quantized-cargo-nominal-active"
        coarse, fine = [
            json.loads((HERE / (study + suffix + ".json")).read_bytes())
            for suffix in ("", "-fine")
        ]
        for first, second in zip(coarse, fine):
            for key in (
                "wheelCase",
                "piece",
                "sourceFeedSha256",
                "sourceFieldPathSha256",
                "elasticStateSha256",
                "actuatorAccelerationCapMps2",
            ):
                if first[key] != second[key]:
                    raise ValueError("Changed setup between solver steps")
            x, f = load_run(assets, first)
            y, g = load_run(assets, second)
            lo, hi = max(x[0, 0], y[0, 0]), min(x[-1, 0], y[-1, 0])
            grid = x[(x[:, 0] >= lo) & (x[:, 0] <= hi), 0]

            def states(data):
                return np.column_stack(
                    [np.interp(grid, data[:, 0], col) for col in data[:, 1:7].T]
                )

            a, b = states(x), states(y)
            dots = np.sum(a[:, 3:] * b[:, 3:], axis=1) / (
                np.linalg.norm(a[:, 3:], axis=1) * np.linalg.norm(b[:, 3:], axis=1)
            )
            result.append(
                {
                    "shelf": "machined" if prefix else "effective",
                    "wheelCase": first["wheelCase"],
                    "solverStepsUs": [1e6 * row["stepS"] for row in (first, second)],
                    "commonIntervalAfterStationS": [lo, hi],
                    "bothReachedSector": first["plannedEndpoint"]
                    and second["plannedEndpoint"],
                    "coarseFault": first["firstFault"],
                    "fineFault": second["firstFault"],
                    "sameDeliveredCaptureTimesAndCustodyStates": [
                        frame[:3] for frame in f
                    ]
                    == [frame[:3] for frame in g],
                    "coarseFrames": len(f),
                    "fineFrames": len(g),
                    "maxResampledCenterDifferenceMm": 1000
                    * np.linalg.norm(a[:, :3] - b[:, :3], axis=1).max(),
                    "maxResampledAxisDifferenceDeg": np.degrees(
                        np.arccos(np.clip(dots, -1, 1))
                    ).max(),
                    "coarseTrajectorySha256": first["trajectorySha256"],
                    "fineTrajectorySha256": second["trajectorySha256"],
                }
            )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    args = p.parse_args()
    (HERE / "refinement.json").write_text(
        json.dumps(compare(args.assets), indent=2) + "\n", encoding="utf8", newline="\n"
    )
