"""Run the captured rail/continuous-cam fixture from the corrected pipe's actual modal amplitudes."""

from pathlib import Path
import argparse
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "score-stroke"))
from cam_model import activate_rack
from loaded_rack import read, digest
import dynamics
import dynamics_run as rack


def run(artifacts, side, mm, step):
    folder = artifacts / "low-placement/advance-25" / side
    observed = read(folder / "coral.json")
    statepath = folder / "coral.npz"
    if (
        not observed["allGranted"]
        or digest(statepath) != observed["state"]
        or abs(observed["endWallS"] - read(folder / "chassis.json")["endWallS"]) > 1e-7
    ):
        raise ValueError("No retained and observed stopped payload")
    cam = activate_rack()
    with np.load(statepath) as d:
        state = {k: d[k].copy() for k in ("p", "a", "v", "l", "z", "zdot", "plunger")}

    def schedule(_, dt):
        duration = 7.0
        t = np.arange(round(duration / dt) + 1) * dt
        s, v, a = dynamics.profile(t, 3.0)
        ret = t >= 4.0
        sr, vr, ar = dynamics.profile(t[ret] - 4.0, 3.0)
        s[ret], v[ret], a[ret] = 0.37 - sr, -vr, -ar
        scale = mm / 370.0
        return duration, t, s * scale, v * scale, a * scale

    rack.sample_motion = schedule
    name = f"rack-{mm:g}mm-{step:g}us"
    result = rack.trial(state, name, folder, step * 1e-6, motion=True, challenge=True)
    result.update(
        sourcePipe=observed["state"],
        piece=observed["piece"],
        wallStart=observed["endWallS"],
        cam=cam["candidateStepSha256"],
        strokeMm=mm,
        stepUs=step,
        sourceWindow="prescribed rack out for 3 s, dwell 1 s, return 3 s",
        scope="Loaded receiver flexibility reference; cargo constrained during this run. Terminal landing requires a separately integrated pipe.",
    )
    (folder / (name + ".json")).write_bytes(
        (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
    )
    print(result["traceSha256"])


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=("plus", "minus"))
    p.add_argument("--mm", type=float, default=337)
    p.add_argument("--step", type=float, default=31.25)
    a = p.parse_args()
    run(a.artifacts, a.side, a.mm, a.step)
