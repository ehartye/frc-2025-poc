"""Overlay the native station on the already complete match-drilled robot view."""

import math
import numpy as np
from camera import sensor


def scene(field, delta, robot):
    dx, dy, angle = delta
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    mesh = []
    for key, vs, ts in field:
        x = 444.525 - dx - vs[:, 0]
        y = -dy - vs[:, 1]
        pts = np.column_stack((c * x + s * y, -s * x + c * y, vs[:, 2]))
        mesh.append(
            [
                key,
                "station",
                "#808080" if "/GE-" in key else "#ababab",
                np.rint(pts * 10).astype(int).ravel().tolist(),
                ts,
            ]
        )
    fg = sensor.camera.images_for_pose({"parts": mesh})
    return [
        (np.where(fz < rz, gr, g0), np.minimum(fz, rz))
        for (gr, fz), (g0, rz) in zip(fg, robot)
    ]
