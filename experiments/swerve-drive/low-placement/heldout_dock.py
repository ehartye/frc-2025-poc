"""Register the held-out signed route with its own tag pixels and target-bolt exposure."""

import argparse
from pathlib import Path
import json
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cam-scoring"))
from reef_camera import camera_input, sha, TAG_ERROR


def run(artifacts):
    source = artifacts / "cam-scoring"
    target = artifacts / "low-placement/holdout"
    with np.load(source / "camera.npz") as data:
        ctx = {key: data[key] for key in data.files}
    signed = json.loads((source / "reef-optics.json").read_bytes())
    if signed["fieldScene"] != sha(source / "camera.npz"):
        raise ValueError("Different fixed cameras or field registration")
    ctx["faces"] = [ctx.pop("face0"), ctx.pop("face1")]
    survey = camera_input.fix.mono.run(
        artifacts,
        scene=ctx,
        routes={"minus": target / "minus/route"},
        pixel_error=TAG_ERROR,
    )
    data = target / "reef-survey.json"
    data.write_bytes((json.dumps(survey, indent=2) + "\n").encode())
    camera_input.run(
        artifacts, scene=ctx, survey=data, output=target, tag_error=TAG_ERROR
    )
    print(sha(data), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    run(p.parse_args().artifacts)
