"""Replay the captured-rail sizing model with the corrected-wheel, signed first pipe."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SIX = ROOT / "experiments/six-wheel"
sys.path.insert(0, str(SIX / "extension-support"))
from dynamics_run import trial


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(source, out, sign, dt, buttons):
    folder = source / (sign + "-15.625us")
    packet = folder / "service.json"
    state = folder / (sign + "-15.625us-service.npz")
    recorded = read(packet)
    if (
        recorded["outgoing"] != digest(state)
        or recorded["piece"] != "coral-A"
        or recorded["wallS"] != 27.035
    ):
        raise ValueError(
            "Require the first corrected-wheel service at the actual station"
        )
    with np.load(state, allow_pickle=False) as file:
        incoming = {key: file[key] for key in ("p", "a", "z", "zdot")}
    if not np.allclose(incoming["p"] * 1000, recorded["centerMm"], rtol=0, atol=1e-7):
        raise ValueError("Wrong starting pipe")
    label = f"{sign}-{'issued' if buttons else 'no-buttons'}-{dt:g}us"
    result = trial(incoming, label, out, dt * 1e-6, active_buttons=buttons)
    result.update(
        stationReceiptSha256=digest(packet),
        stationStateSha256=digest(state),
        stationCenterM=incoming["p"].tolist(),
        stationAxis=incoming["a"].tolist(),
        stationToDockMotion=None,
    )
    (out / (label + ".json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8"
    )


if __name__ == "__main__":
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("station_output", type=Path)
    args.add_argument("output", type=Path)
    args.add_argument("sign", choices=["plus", "minus"])
    args.add_argument("dt_us", type=float)
    args.add_argument("--no-buttons", action="store_true")
    a = args.parse_args()
    main(a.station_output, a.output, a.sign, a.dt_us, not a.no_buttons)
