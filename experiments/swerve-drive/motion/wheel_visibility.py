"""Check whether rotating wheel-module envelopes can enter either R4 custody camera view."""

import argparse
import itertools
from pathlib import Path
import numpy as np
from loaded_departure import write


def load_wheel_mask(path):
    from receiver_witness import read, digest, PROJECT, HERE

    register = read(path)
    archive = path.with_suffix(".npz")
    if (
        register["maskSha256"] != digest(archive)
        or register["publicStepSha256"]
        != digest(PROJECT / "cad/full-swerve-r4/full-robot.step")
        or register["source"] != digest(HERE.parent / "packaging/envelopes.json")
    ):
        raise ValueError("Different supplier sweep, robot or uncertainty mask")
    with np.load(archive, allow_pickle=False) as data:
        return {
            head: [data[head + str(eye)].copy() for eye in range(2)]
            for head in ("roof", "fixed")
        }


def mask_fixed_contour(images, background, masks, whole_scanlines=False):
    masked_images, masked_background = [], []
    for image, base, mask in zip(images, background, masks):
        if whole_scanlines:
            blocked = mask.any(axis=1)[:, None]
        else:
            # The issued fit uses a single contiguous run per scanline. Drop a
            # row if uncertainty reaches the run or its three-pixel edge witness.
            changed = (abs(image.astype(int) - base.astype(int)) > 36) & ~mask
            x = np.arange(image.shape[1])[None, :]
            left = np.where(changed, x, image.shape[1] + 4).min(axis=1)[:, None]
            right = np.where(changed, x, -5).max(axis=1)[:, None]
            vulnerable = (mask & (x >= left - 3) & (x <= right + 3)).any(axis=1)[
                :, None
            ]
            blocked = mask | vulnerable
        masked_images.append(np.where(blocked, 32, image))
        masked_background.append(np.where(blocked, 32, base))
    return masked_images, masked_background


def check(mask_path):
    import receiver_witness as camera

    bounds = camera.HERE.parent / "packaging/envelopes.json"
    boxes = camera.read(bounds)["cornerBoundsMm"]
    refs = camera.source_parts()
    tris = np.array(
        [
            [0, 2, 3],
            [0, 3, 1],
            [4, 5, 7],
            [4, 7, 6],
            [0, 1, 5],
            [0, 5, 4],
            [2, 6, 7],
            [2, 7, 3],
            [0, 4, 6],
            [0, 6, 2],
            [1, 3, 7],
            [1, 7, 5],
        ]
    )
    parts = []
    for name, box in boxes.items():
        vertices = [
            tuple(box[k + 3 * side[k]] for k in range(3))
            for side in itertools.product(range(2), repeat=3)
        ]
        parts.append(
            [
                name,
                "wheel",
                "#ffffff",
                np.round(np.array(vertices) * 10).ravel().tolist(),
                tris.ravel().tolist(),
            ]
        )
    out = {}
    masks = {}
    stationary = [part for part in refs if "-display-" not in part[0]]
    if len(refs) - len(stationary) != 24:
        raise ValueError("Expected six depicted module parts per corner")
    for head in ("fixed", "roof"):
        robot = camera.render(refs, head)
        fixed_geometry = camera.render(stationary, head)
        envelope = camera.sensor.camera.images_for_pose({"parts": parts})
        out[head] = []
        for eye, ((_, shown_z), (_, rz), (_, ez)) in enumerate(
            zip(robot, fixed_geometry, envelope)
        ):
            rays = ez < 3499
            # A depicted forward module may extend beyond the supplier sweep.
            possible = (rays & (ez <= rz + 0.1)) | (shown_z < rz - 0.01)
            import cv2

            mask = cv2.dilate(
                possible.astype(np.uint8), np.ones((11, 11), np.uint8)
            ).astype(bool)
            masks[head + str(eye)] = mask

            y, x = np.nonzero(possible)
            out[head].append(
                {
                    "raysToModuleEnvelope": int(np.count_nonzero(rays)),
                    "excludedPixels": int(mask.sum()),
                    "possiblyVisiblePixels": len(x),
                    "possibleRect": None
                    if len(x) == 0
                    else [int(x.min()), int(y.min()), int(x.max()), int(y.max())],
                    "minimumBehindOpaqueRobotMm": None
                    if not rays.any()
                    else float(np.min((ez - rz)[rays])),
                }
            )
    np.savez_compressed(mask_path, **masks)
    return dict(
        maskSha256=camera.digest(mask_path),
        publicStepSha256=camera.digest(
            camera.PROJECT / "cad/full-swerve-r4/full-robot.step"
        ),
        envelopePlusDepictedModule=True,
        neighborhoodRadiusPx=5,
        source=camera.digest(bounds),
        solids=len(boxes),
        allowanceMm=0.1,
        heads=out,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    write(args.output, check(args.output.with_suffix(".npz")))
