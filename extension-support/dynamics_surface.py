"""Read a registered dynamic fixture surface at a rack-local material point."""

import argparse
import json
from pathlib import Path

import numpy as np

from dynamics import digest, plate


class SurfaceTrace:
    def __init__(self, path, expected):
        if digest(path) != expected:
            raise ValueError("Surface trace digest differs from the case receipt")
        with np.load(path, allow_pickle=False) as data:
            self.time = data["timeS"]
            self.stroke = data["strokeM"]
            self.q, self.v = data["q"], data["v"]
            self.shapes = data["plateShape"]
        if self.q.shape != self.v.shape or self.q.shape != (
            len(self.time),
            6 + self.shapes.shape[1],
        ):
            raise ValueError("Inconsistent physical coordinate inventory")

    def at(self, seconds, x, y):
        """At an inner-stage-local point. Velocities hold this material point fixed."""
        if not np.isfinite([seconds, x, y]).all() or not self.time[0] <= seconds <= self.time[-1]:
            raise ValueError("Time is outside the retained fixture trajectory")
        if not -0.100 <= x <= 0.165 or not -0.180 <= y <= 0.180:
            raise ValueError("Point lies outside the plate basis")
        j = min(len(self.time) - 2, max(0, np.searchsorted(self.time, seconds, side="right") - 1))
        interval = self.time[j + 1] - self.time[j]
        f = (seconds - self.time[j]) / interval
        # Cubic endpoint interpolation is exact at both samples. To preserve
        # every integration tick request --sample-us equal to --dt-us.
        q0, q1 = self.q[j : j + 2]
        v0, v1 = self.v[j : j + 2]
        q = (1 - 3 * f * f + 2 * f**3) * q0 + (3 * f * f - 2 * f**3) * q1
        q += interval * ((f - 2 * f * f + f**3) * v0 + (f**3 - f * f) * v1)
        v = (
            (6 * f - 6 * f * f) / interval * (q1 - q0)
            + (1 - 4 * f + 3 * f * f) * v0
            + (3 * f * f - 2 * f) * v1
        )

        def basis(dx, dy):
            return (plate.basis(np.array([x]), np.array([y]), 13, 20, dx, dy) @ self.shapes)[0]

        b = basis(0, 0)
        travel = float(np.interp(seconds, self.time, self.stroke))
        return dict(
            timeS=seconds,
            strokeM=travel,
            chassisXM=x + travel,
            localYM=y,
            verticalM=float(q[4] + x * q[5] + b @ q[6:]),
            verticalSpeedMps=float(v[4] + x * v[5] + b @ v[6:]),
            slopeX=float(q[5] + basis(1, 0) @ q[6:]),
            slopeY=float(basis(0, 1) @ q[6:]),
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trace", type=Path)
    p.add_argument("sha256")
    for k in ("seconds", "x", "y"):
        p.add_argument(k, type=float)
    a = p.parse_args()
    print(json.dumps(SurfaceTrace(a.trace, a.sha256).at(a.seconds, a.x, a.y), indent=2))
