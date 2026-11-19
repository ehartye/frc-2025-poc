"""Bound the pipe's positive-Y support against the native arm along each registered pickup."""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

from arm_route import ARM, HERE, ROOT, carriage, j

sys.path.insert(0, str(ROOT / "axial-spin"))
from pipe_axial_contact import HALF, R


def run(raw):
    arm = carriage.selected(HERE, "route.step", "revision.json", "stepSha256", "volumesMm3")[ARM]
    inner = float(j.cad.bounds(arm)[1])
    register = HERE / "mechanics/joined.json"
    trials = {}
    for row in j.read(register)["cases"]:
        name = row["id"]
        path = raw / name / (name + ".npy")
        if j.digest(path) != row["trajectorySha256"]:
            raise ValueError("Unregistered pipe trajectory")
        trace = np.load(path, mmap_mode="r")
        axes = trace[:, 4:7]
        if not np.isfinite(trace).all() or np.max(np.abs(np.linalg.norm(axes, axis=1)-1.)) > 1e-7:
            raise ValueError("Invalid sampled pipe pose")
        ay = np.clip(axes[:, 1], -1., 1.)
        support = trace[:, 2] + 1000*(HALF*np.abs(ay) + R*np.sqrt(1.-ay**2))
        # Along a linear center and shortest-arc axis interpolation, bound every
        # intermediate surface point from the preceding node by the full step.
        angle = np.arctan2(np.linalg.norm(np.cross(axes[:-1],axes[1:]),axis=1),
                           np.sum(axes[:-1]*axes[1:],axis=1))
        pad = np.abs(np.diff(trace[:, 2])) + 1000*math.hypot(HALF,R)*angle
        sampled = inner - float(support.max())
        lower = min(sampled, float(np.min(inner-support[:-1]-pad)))
        trials[name] = {"pipeSha256":row["trajectorySha256"],"poses":len(trace),
                        "sampledGapMm":sampled,"interpolatedGapLowerMm":lower}
    return {"stepSha256":j.digest(HERE/"route.step"),"joinedSha256":j.digest(register),
            "pipeGeometrySourceSha256":j.digest(ROOT/"axial-spin/pipe_axial_contact.py"),
            "minimumArmYMm":inner,"pipeRadiusMm":R*1000,"pipeHalfLengthMm":HALF*1000,"trials":trials}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_trials",type=Path)
    j.write(HERE/"pipe-gap.json",run(parser.parse_args().raw_trials))
