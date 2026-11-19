"""Sample the two moving native apertures at their 20 ms cadence in transit."""

import argparse
import json
from pathlib import Path
import numpy as np

from checkpoint import HERE, digest
from drive import restore


def observe(folder, case, dt):
    dep = next(
        r
        for r in json.loads((HERE / "departure.json").read_bytes())["cases"]
        if r["case"] == case
    )
    pipe = restore(folder / "checkpoints" / (case + "-state.npz"), dep)
    raw = folder / "carry" / f"{case}-{round(dt * 1e9)}ns.npy"
    data = np.load(raw, mmap_mode="r")
    count = len(pipe.z)
    changes = []
    first = None
    for ms in range(0, math_floor_ms(data[-1, 0]) + 1, 20):
        ridx = round(ms)
        if ridx >= len(data) or abs(data[ridx, 0] - ms / 1000) > 1e-7:
            raise ValueError("No synchronous pipe/plate sample at controller tick")
        r = data[ridx]
        pipe.p = r[1:4]
        pipe.a = r[4:7]
        pipe.z = r[13 : 13 + count]
        hits = list(map(bool, pipe.ports()))
        if not changes or hits != changes[-1]["entrySeat"]:
            changes.append({"fromDepartureMs": ms, "entrySeat": hits})
        if not all(hits) and first is None:
            first = ms
    return {
        "case": case,
        "stepS": dt,
        "stateSha256": dep["stateSha256"],
        "rawSha256": digest(raw),
        "firstMissingFromDepartureMs": first,
        "changes": changes,
    }


def math_floor_ms(t):
    return int(np.floor(t * 1000 + 1e-7))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--steps", type=float, nargs="+", default=[0.000125, 0.0000625])
    a = p.parse_args()
    rows = [
        observe(a.folder, c, dt) for dt in a.steps for c in ("leftSmall", "rightSmall")
    ]
    a.out.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf8", newline="\n")
