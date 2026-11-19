"""Export the attained slow loaded route on its continuous chassis clock."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(study, artifacts, cache):
    sys.path.insert(0, str(study.resolve()))
    from transport import trajectory

    src = study / "slow-navigation.json"
    raw = json.loads(src.read_bytes())
    result = {
        "slowNavigationSha256": sha(src),
        "powerSha256": sha(study / "slow-power.json"),
        "cases": [],
    }
    cache.mkdir(exist_ok=True, parents=True)
    for side, reg in raw.items():
        origin = artifacts / "conditioned-match" / side / "slow/A"
        if (
            sha(origin / "transit.npz") != reg["transitSha256"]
            or sha(origin / "coral-A-nominal.csv") != reg["motionSHA256"]
        ):
            raise ValueError("Different attained route")
        times, poses = trajectory(artifacts, side, "slow")
        dest = cache / (side + "-route.npz")
        np.savez(dest, times=times, poses=poses)
        result["cases"].append(
            {
                "case": side,
                "transitSha256": reg["transitSha256"],
                "visualSha256": reg["motionSHA256"],
                "durationS": times[-1],
                "routeSha256": sha(dest),
                "acceptedDock": reg["accepted"],
            }
        )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("study", "artifacts", "cache", "out"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.study, a.artifacts, a.cache), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
