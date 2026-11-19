"""Select the real terminal pipe exposures and bound free guide motion for an optical replay."""

import argparse
import json
from pathlib import Path
import numpy as np
from beam import Beam, ROOT, digest, guide

EXPOSURES = (5360, 5372, 5389, 5406, 5423, 5440, 5457, 5460)


def run(register, kit, raw):
    data = json.loads(register.read_bytes())
    packets = []
    for row in data["cases"]:
        if (
            row["stepUs"] != 31.25
            or row["layout"] != "folded"
            or not row["eligibleForStationImageReplay"]
        ):
            continue
        name = row["id"]
        root = raw / name
        file = root / (name + ".npy")
        gf = root / "guide.npy"
        if (
            digest(file) != row["trajectorySha256"]
            or digest(gf) != row["guideHistorySha256"]
        ):
            raise ValueError("Changed pipe/guide record")
        trace = np.load(file, mmap_mode="r")
        h = np.load(gf, mmap_mode="r")
        last = h[-1, 0]
        remaining = trace[trace[:, 0] > 100 + 1000 * last + 1e-7]
        axis_z = remaining[:, 6]
        upper = (
            remaining[:, 3] / 1000
            + guide.feed.HALF * abs(axis_z)
            + guide.feed.R * np.sqrt(np.maximum(0.0, 1 - axis_z**2))
        )
        if len(upper) == 0 or upper.max() >= 0.625:
            raise ValueError("Free guide decay has no non-contact bound")
        layout, cutoff, fraction = row["guideModel"]
        beam = Beam(kit, layout, cutoff, arm_fraction=fraction)
        # Cauchy-Schwarz per modal initial displacement/velocity. Linear
        # interpolation of each Phi(z) bounds the weighted norm by its node maxima.
        compliance = np.max(np.sum((beam.phi / beam.w) ** 2, axis=1)) ** 0.5
        bound = (
            1000 * compliance * np.sqrt(2 * h[-1, 11:13]) / np.sqrt(1 - beam.damping)
        )
        samples = []
        for tick in EXPOSURES:
            i = int(np.argmin(abs(trace[:, 0] - tick)))
            if abs(trace[i, 0] - tick) > 1e-6:
                raise ValueError("No exposure at requested time")
            decay = np.exp(-beam.damping * beam.w.min() * ((tick - 100) / 1000 - last))
            samples.append(
                {
                    "javaMs": tick,
                    "centerMm": trace[i, 1:4].tolist(),
                    "axis": trace[i, 4:7].tolist(),
                    "velocityMps": trace[i, 7:10].tolist(),
                    "omegaRadS": trace[i, 10:13].tolist(),
                    "allFaceModeledAbsUpperMm": (bound * decay).tolist(),
                }
            )
        tail = trace[trace[:, 0] >= 5360 - 1e-7]
        out = root / "last-100ms.npy"
        np.save(out, tail)
        packets.append(
            {
                "id": name,
                "case": row["case"],
                "guideModel": row["guideModel"],
                "outwardPerSideMm": row["outwardPerSideMm"],
                "carriageSupport": row["carriageSupport"],
                "trajectorySha256": row["trajectorySha256"],
                "modeStateSha256": row["stateSha256"],
                "guideHistorySha256": row["guideHistorySha256"],
                "last100msSha256": digest(out),
                "last100msRows": len(tail),
                "lastGeometricGuideStepLipS": float(last),
                "maxPipeTopAfterGuideMm": float(1000 * upper.max()),
                "frames": samples,
                "actualOpticalCustody": None,
            }
        )
    return {
        "source": digest(register),
        "frameClock": "Java milliseconds; chute lip at Java 100",
        "packets": packets,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("register", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--kit", type=Path, default=ROOT / "guide-mounts")
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.register, a.kit, a.raw), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
