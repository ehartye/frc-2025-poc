"""Simultaneous receiver resultant at the rear locks and both mast feet."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from response import SPAN, Y0, basis, forced, plate, trace

COLUMNS = [
    "fromFirstPulseS",
    "leftRailDownN",
    "rightRailDownN",
    "chassisFxN",
    "chassisFyN",
    "chassisFzN",
    "atLockMxNm",
    "atLockMyNm",
    "atLockMzNm",
    "atFeetMxNm",
    "atFeetMyNm",
    "atFeetMzNm",
]


def run(raw, source, out):
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for hybrid in (False, True):
        nx, ny = 13, 20
        mode, w, inertia = plate(hybrid, nx, ny, 2.2)
        for ref in json.loads(source.read_bytes()):
            if ref["case"] not in ("leftSmall", "rightSmall"):
                continue
            file = raw / (ref["case"] + "-stations.csv")
            idx, d = trace(file, ref)
            dt = ref["dt"]
            count = int(np.ceil(0.12 / dt)) + idx.max() + 1
            down = -d[:, 7]
            geom = np.column_stack(
                (
                    down,
                    down * (d[:, 3] * 0.001 - Y0) / SPAN,
                    down * (0.001 * d[:, 2] + 0.087),
                )
            )
            applied = np.zeros((count, 3))
            np.add.at(applied, idx, geom)
            q = np.zeros((count, len(w)))
            np.add.at(
                q,
                idx,
                down[:, None] * (basis(d[:, 2] / 1000, d[:, 3] / 1000, nx, ny) @ mode),
            )
            _, acc = forced(q, w, 0.02, dt)
            down_total, left, pitch = (applied - acc @ inertia.T).T
            right = down_total - left
            # Full in-plane reaction transmitted at contact with zero in-plane
            # inertia. Applied horizontal wrenches use each station's true height.
            z = d[:, 4] * 0.001
            x = d[:, 2] * 0.001
            y = d[:, 3] * 0.001
            fx, fy = d[:, 5], d[:, 6]
            other = np.zeros((count, 8))
            np.add.at(
                other,
                idx,
                np.column_stack(
                    (
                        fx,
                        fy,
                        -(z - 0.4225) * fy,
                        (z - 0.4225) * fx,
                        (x + 0.087) * fy - y * fx,
                        -(z - 0.154) * fy,
                        (z - 0.154) * fx,
                        (x + 0.045) * fy - y * fx,
                    )
                ),
            )
            hx, hy, lr, lp, ly, fr, fp, fyaw = other.T
            roll = -0.18 * (left - right)
            data = np.column_stack(
                (
                    np.arange(count) * dt,
                    left,
                    right,
                    hx,
                    hy,
                    -down_total,
                    roll + lr,
                    pitch + lp,
                    ly,
                    roll + fr,
                    pitch - 0.042 * down_total + fp,
                    fyaw,
                )
            )
            label = ("insert" if hybrid else "stock") + "-" + ref["case"]
            target = out / (label + ".csv")
            np.savetxt(
                target,
                data,
                fmt="%.9g",
                delimiter=",",
                header=",".join(COLUMNS),
                comments="",
            )
            points = {}
            for i, key in enumerate(COLUMNS[1:], 1):
                for op, fn in (("min", np.argmin), ("max", np.argmax)):
                    points[key + "-" + op] = data[fn(data[:, i])].tolist()
            rows.append(
                {
                    "case": label,
                    "totalSamples": len(data),
                    "stepS": dt,
                    "loadSha256": ref["stationsSha256"],
                    "file": target.name,
                    "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
                    "coincident": points,
                }
            )
    return {
        "columns": COLUMNS,
        "lockReferenceXYZmm": [-87, 0, 422.5],
        "footReferenceXYZmm": [-45, 0, 154],
        "positiveRailValueIsDown": True,
        "physicalMastLoad": None,
        "cases": rows,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("raw", "source", "out", "receipt"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    r = run(a.raw, a.source, a.out)
    a.receipt.write_text(json.dumps(r, indent=2) + "\n", encoding="utf8", newline="\n")
