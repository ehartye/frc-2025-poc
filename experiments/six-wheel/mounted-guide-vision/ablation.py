"""Render non-buildable optical removal controls for individual guide components."""

import argparse
import json
from pathlib import Path
import numpy as np

from station import HERE, ROOT, mesh, sensor, O, digest, read


def run(assets):
    baseline = assets / "observed-reef-a/machined-fixed.npz"
    if (
        digest(baseline)
        != read(ROOT / "observed-reef-a/station-bridge.json")["leftSmall"][
            "machinedFieldBackgroundSha256"
        ]
    ):
        raise ValueError("Changed source robot and harness")
    native = sensor.background(baseline)
    kit, _sha, _setting = mesh("nominal")
    sensor.scenes.camera = sensor.camera
    sensor.scenes.fixed_pose(O)
    result = {}
    for name, omit in (
        ("omit-lower-left-arm", ["guide-L-687-mount-arm"]),
        ("omit-left-backing", ["guide-L-backing"]),
        ("omit-both", ["guide-L-687-mount-arm", "guide-L-backing"]),
    ):
        selected = [p for p in kit["parts"] if p[0] not in omit]
        if len(selected) != len(kit["parts"]) - len(omit):
            raise ValueError("Missing control article")
        layer = sensor.camera.images_for_pose({"parts": selected})
        pair = sensor.composite(native, layer)
        path = assets / "mounted-guide-vision" / (name + "-fixed.npz")
        np.savez_compressed(
            path,
            left=pair[0][0],
            left_z=pair[0][1],
            right=pair[1][0],
            right_z=pair[1][1],
        )
        result[name] = {
            "register": {
                "fixedBackgroundSha256": digest(path),
                "omitted": omit,
                "guideStepSha256": _sha,
                "sourceRobotSha256": digest(baseline),
                "buildable": False,
            }
        }
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("assets", type=Path)
    a = p.parse_args()
    (HERE / "ablation-sources.json").write_text(
        json.dumps(run(a.assets), indent=2) + "\n", encoding="utf8", newline="\n"
    )
