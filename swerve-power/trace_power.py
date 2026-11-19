"""Join actual emitted drive/steer phase demands to retained protected branches."""

import argparse
from pathlib import Path
import numpy as np
from electrical import HERE, IDS, PACKS, other_watts, source, branches, DRIVE_PHASE_R, STEER_PHASE_R
from screen import sha, write


def assess(path, registered, state="stowed", pack="screened"):
    if sha(path) != registered:
        raise ValueError("Unregistered motion")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")
    if data.size % 4 or data.size < 4:
        raise ValueError("Partial clock")
    r = data.reshape((-1, 4))
    wall = r["wall_s"][:, 0]
    if (
        not np.all(r["id"] == np.array(IDS)[None, :])
        or not np.all(r["wall_s"] == wall[:, None])
        or np.any(np.diff(wall) <= 0)
    ):
        raise ValueError("Missing, reordered, duplicate or asynchronous corner")
    w = other_watts((len(wall),), state)
    requirements = {}
    phases = {}
    for axis, resistance in (("drive", DRIVE_PHASE_R), ("steer", STEER_PHASE_R)):
        current = r[axis + "_phase_a"]
        v = r[axis + "_voltage_v"] + current * resistance
        if not np.isfinite(current).all() or not np.isfinite(v).all():
            raise ValueError("Invalid motor data")
        for j, name in enumerate(IDS):
            key = name + axis.title()
            w[key] = 0.7 + np.abs(current[:, j] * v[:, j]) / 0.90
            phases[key] = current[:, j]
            requirements[key] = np.abs(v[:, j])
    names, bus, term, amps = source(w, PACKS[pack])
    br = branches()
    failed = ~np.isfinite(bus) | (np.min(term, axis=1) < 9.5)

    def peak(values, maximum=True):
        good = np.flatnonzero(np.isfinite(values))
        if not len(good):
            return None
        i = good[np.argmax(values[good]) if maximum else np.argmin(values[good])]
        return [float(values[i]), float(wall[i])]

    rows = {}
    for j, k in enumerate(names):
        failed |= amps[:, j] > br[k]["breakerA"]
        row = rows[k] = dict(minTerminalV=peak(term[:, j], False), maxBranchA=peak(amps[:, j]))
        if k in requirements:
            margin = term[:, j] - requirements[k]
            cap = 40 if k.endswith("Drive") else 20
            failed |= (margin < 0) | (abs(phases[k]) > cap + 1e-8)
            row.update(minimumMotorHeadroomV=peak(margin, False), maxPhaseA=peak(abs(phases[k])))
    bad = np.flatnonzero(failed)
    return dict(
        sha256=registered,
        samples=len(wall),
        pack=pack,
        otherLoads=state,
        minBusV=peak(bus, False),
        peakPackA=peak(amps.sum(axis=1)),
        perBranch=rows,
        firstBlockedWallS=float(wall[bad[0]]) if len(bad) else None,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("clock", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--sha256", required=True)
    p.add_argument("--receiver", choices=["stowed", "loaded", "actuating"], default="stowed")
    a = p.parse_args()
    write(a.out, dict(trials=[assess(a.clock, a.sha256, a.receiver, k) for k in PACKS]))
