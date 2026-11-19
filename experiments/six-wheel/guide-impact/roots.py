"""Resolve individual mounting spring reactions on the independently saved coupled motion."""

import argparse
import json
from pathlib import Path
import numpy as np
from beam import Beam, ROOT, digest, guide, support_source
from backing import GuideResponse


def recover(row, raw, kit):
    folder = raw / row["id"]
    original = folder / "guide.npy"
    path = folder / (row["id"] + ".npy")
    if (
        digest(path) != row["trajectorySha256"]
        or digest(original) != row["guideHistorySha256"]
    ):
        raise ValueError("Different contact event")
    x = np.load(path, mmap_mode="r")
    beam = Beam(kit, "folded", row["guideModel"][1], arm_fraction=row["guideModel"][2])
    g = guide.Guide(
        throat_mm=-0.3 + row["outwardPerSideMm"],
        stiffness_n_mm=10,
        damping_n_s_m=200,
        friction=0.1,
        sectors=128,
        radial_bins=4,
        x_width_mm=40,
        upper_mm=813,
        x_offset_mm=20,
    )
    pipe = guide.GuidedPipe(row["poseMmDeg"], 0.0, g)
    response = GuideResponse(beam)
    z = np.array([r[0] for r in beam.attachment]) / 1000
    phi = beam.at(z)[0]
    k = np.array([r[2] for r in beam.attachment])
    source = support_source(kit)
    arm_lengths = (
        np.array([source.arm(mm)["lengthMm"] for mm in source.STATIONS]) / 1000
    )
    a = x[:, 6]
    rad = guide.feed.HALF * abs(a) + guide.feed.R * np.sqrt(np.maximum(0, 1 - a * a))
    within = (x[:, 3] / 1000 + rad >= 0.625) & (x[:, 3] / 1000 - rad <= 0.813)
    data = []
    for i in np.flatnonzero(within[:-1]):
        r = x[i]
        pipe.time = (r[0] - 100) / 1000
        pipe.p = r[1:4] / 1000
        pipe.a = r[4:7].copy()
        pipe.v = r[7:10].copy()
        w = r[10:13]
        pipe.l = (
            guide.feed.IP * w + (guide.feed.I - guide.feed.IP) * (w @ pipe.a) * pipe.a
        )
        response.step(pipe, (x[i + 1, 0] - r[0]) / 1000)
        if response.first_loaded is not None:
            displacement = response.q @ phi.T
            reaction = displacement * k
            data.append(
                [
                    response.last,
                    *reaction.ravel(),
                    *(displacement * 1000).ravel(),
                    *0.5 * np.sum(reaction * displacement, axis=1),
                ]
            )
    h = np.load(original)
    reconstructed = np.array(response.trace)
    if h.shape != reconstructed.shape:
        raise ValueError("Contact window differs")
    diff = np.max(abs(h - reconstructed), axis=0)
    if diff[1:3].max() > 1e-7 or diff[9:11].max() > 1e-10 or diff[-2:].max() > 1e-10:
        raise ValueError(f"Contact work or force differs: {diff.tolist()}")
    data = np.asarray(data)
    file = folder / "root-reactions.npy"
    np.save(file, data)
    force = data[:, 1:5].reshape(-1, 2, 2)
    # The arm's static direction is transverse. These are spring loads and gross lever moments.
    # Modal damping is specified globally and has no uniquely allocated mast reaction.
    return {
        "id": row["id"],
        "rawRootHistory": digest(file),
        "sourceTrajectory": digest(path),
        "peakPositiveSpringN": force.max(axis=0).tolist(),
        "peakNegativeSpringN": force.min(axis=0).tolist(),
        "absoluteLeverMomentNm": (np.max(abs(force), axis=0) * arm_lengths).tolist(),
        "peakTwoSpringStrainEnergyJ": data[:, -2:].max(axis=0).tolist(),
        "maxGuideReplayForceErrorN": diff[1:3].max(),
        "maxGuideReplayTravelErrorMm": diff[9:11].max() * 1000,
        "maxGuideReplayWorkErrorJ": diff[-2:].max(),
    }


def run(reg, raw, kit):
    r = json.loads(reg.read_bytes())
    rows = [
        x
        for x in r["cases"]
        if x["stepUs"] == 31.25
        and x["layout"] == "folded"
        and x["guideModel"][1] == 1800
        and not x["carriageSupport"]
    ]
    return {
        "input": digest(reg),
        "springOrder": ["-Y 687", "-Y 787", "+Y 687", "+Y 787"],
        "cases": [recover(x, raw, kit) for x in rows],
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("reg", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--kit", type=Path, default=ROOT / "guide-mounts")
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.reg, a.raw, a.kit), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
