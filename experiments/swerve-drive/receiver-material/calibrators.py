"""Identify stationary robot-owned radiometric references outside the sensing volume."""

import argparse

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "receiver-contrast"))
from finish import select_views as select_views, witness

# The pipe must fit in this deliberately enlarged sensing volume to use the self-calibration.
LO = np.array([-35.0, -205.0, 475.0])
HI = np.array([212.0, 205.0, 665.0])


def off_volume(camera, eye, depth):
    origin = camera.ORIGIN + [0, camera.EYES[eye], 0]
    near, far = np.full(depth.shape, -np.inf), np.full(depth.shape, np.inf)
    for i, ray in enumerate((camera.DX, camera.DY, camera.DZ)):
        ray = np.asarray(ray)
        with np.errstate(divide="ignore", invalid="ignore"):
            a = (LO[i] - origin[i]) / ray
            b = (HI[i] - origin[i]) / ray
        parallel = abs(ray) < 1e-12
        near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(a, b)))
        far = np.minimum(far, np.where(parallel, np.inf, np.maximum(a, b)))
        if not LO[i] <= origin[i] <= HI[i]:
            far = np.where(parallel, -np.inf, far)
    hits = (far >= np.maximum(0, near)) & (np.maximum(0, near) <= depth + 3.0)
    return ~hits


def build(geometry, mask_register, output):
    parts = witness.source_parts()
    masked = witness.load_wheel_mask(mask_register)
    output.mkdir(exist_ok=True, parents=True)
    record = {}
    surfs = witness.read(geometry / "surfaces.json")
    for head in ("fixed", "roof"):
        only = witness.render(parts, head)
        for sign in sorted(surfs["docks"]):
            with np.load(geometry / (sign + ".npz")) as background:
                for eye in range(2):
                    z = background[f"fz{eye}" if head == "fixed" else f"z{eye}"]
                    robot = only[eye][1]
                    valid = (
                        (robot < 3490) & (abs(robot - z) < 0.002) & ~masked[head][eye]
                    )
                    valid &= off_volume(witness.sensor.camera, eye, z)
                    record[f"{sign}-{head}{eye}"] = valid
    path = output / "calibrators.npz"
    np.savez_compressed(path, **record)
    info = {
        "sourceRobotSha256": surfs["robotSha256"],
        "sourceFieldSha256": surfs["fieldSha256"],
        "wheelMaskSha256": witness.digest(mask_register),
        "guardBoxMm": [LO.tolist(), HI.tolist()],
        "pixels": {k: int(m.sum()) for k, m in record.items()},
        "maskSha256": witness.digest(path),
    }
    (output / "calibrators.json").write_text(
        json.dumps(info, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    print(info)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("geometry", "wheel", "output"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    build(a.geometry, a.wheel, a.output)
