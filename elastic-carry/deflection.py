"""Bound sampled vertical motion and slope across the two-rail plate interpolation grid."""

import argparse
import json
from pathlib import Path
import numpy as np
from checkpoint import HERE, coupled, digest


def run(folder):
    table, omega, _ = coupled.tables(True, 2300.0, 0.0, True)
    maximum = np.max(np.abs(table), axis=(0, 1))
    rows = []
    for r in json.loads((HERE / "drive.json").read_bytes()):
        raw = folder / "carry" / f"{r['case']}-{round(r['stepS'] * 1e9)}ns.npy"
        if digest(raw) != r["traceSha256"]:
            raise ValueError("Different forecast")
        d = np.load(raw, mmap_mode="r")
        coeff = d[:, 13 : 13 + len(omega)]
        rate = d[:, 13 + len(omega) : 13 + 2 * len(omega)]
        # Every interpolated value lies in the convex hull of its four tabulated nodes.
        # Bound each complete plate at each recorded timestamp; the sample spacing
        # is independent of the finer contact-integration step.
        limit = np.max(abs(coeff) @ maximum.T, axis=0)
        speed = np.max(abs(rate) @ maximum.T, axis=0)
        rows.append(
            {
                "case": r["case"],
                "stepS": r["stepS"],
                "traceSha256": digest(raw),
                "sampleIntervalMs": 1,
                "globalVerticalUpperMm": 1000 * limit[0],
                "slopeXUpper": limit[1],
                "slopeYUpper": limit[2],
                "verticalSpeedUpperMmPerS": 1000 * speed[0],
            }
        )
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.folder), indent=2) + "\n", encoding="utf8", newline="\n"
    )
