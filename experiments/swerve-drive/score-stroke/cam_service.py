"""Carry the redesigned receiver's genuine station state to the first wheel service tick."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys

# The first coarse run used the numerical library's default thread count.
# Load it under that same environment to meet the upstream bitwise restart check.
if "--source-default-blas" in sys.argv:
    import numpy  # noqa: F401 - initialize the configured BLAS before loading the replay
from cam_model import activate
from loaded_rack import ROOT, read


def run(raw, sign, step, *, gap_mm=0.1, destination=None):
    candidate = activate()
    motion = ROOT / "experiments/swerve-drive/motion"
    spec = importlib.util.spec_from_file_location(
        "signed_wheel_epoch", motion / "first_service.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    name = f"cam-station-{sign}-{step:g}us-{gap_mm:g}mm"
    feed = read(raw / (name + ".json"))
    if (
        feed["candidateStepSha256"] != candidate["candidateStepSha256"]
        or feed["state"] != "IN_BAND"
    ):
        raise ValueError("Candidate pickup was not accepted")
    destination = raw if destination is None else destination
    (destination / name).mkdir(parents=True, exist_ok=True)
    result = module.propagate(raw / name, feed, destination / name / "first-service")
    result.update(
        candidateStepSha256=candidate["candidateStepSha256"],
        registeredPlateTable=feed["registeredPlateTable"],
    )
    (destination / (name + "-service.json")).write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("sign", choices=("plus", "minus"))
    p.add_argument("step", type=float)
    p.add_argument("--source-default-blas", action="store_true")
    p.add_argument("--gap-mm", type=float, default=0.1)
    p.add_argument("--destination", type=Path)
    a = p.parse_args()
    run(a.raw, a.sign, a.step, gap_mm=a.gap_mm, destination=a.destination)
