"""Keep the failed physical tube and actuator moving under the Java neutral demand."""

import argparse, json
from dataclasses import replace
from pathlib import Path
import numpy as np
from restart import restore, seed
from split_checkpoint import HERE, coupled
from split_controller import digest


def follow(path):
    rows = []
    for r in json.loads(path.read_bytes())["cases"]:
        if r["state"] != "FAULT" or r["portTicks"][-1][-1] != 0:
            raise ValueError("Expected inhibited command")
        s = r["terminal"]
        p = restore(r)
        seed(p, s)
        save = coupled.feed.cfg
        try:
            coupled.feed.cfg = replace(
                save,
                slices=10,
                belt_mu=0.0 if r["id"] == "lost-traction" else save.belt_mu,
            )
            start = p.time
            until = start + 0.20
            zero = start if p.belt == 0 else None
            furthest = 0.0
            vmax = 0.0
            while until - p.time > 1e-11 and not p.loss and not p.hard:
                p.moves(0.0, min(r["stepS"], until - p.time))
                furthest = max(
                    furthest, float(np.linalg.norm(p.p - np.asarray(s["p"])))
                )
                vmax = max(vmax, float(np.linalg.norm(p.v)))
                if zero is None and p.belt == 0:
                    zero = p.time
            result = p.capture(0.0)
        finally:
            coupled.feed.cfg = save
        rows.append(
            {
                "disturbance": r["id"],
                "side": r["side"],
                "stepS": r["stepS"],
                "faultJavaMs": s["javaMs"],
                "faultReason": r["fault"],
                "beltSpeedAtFaultMps": s["actualBeltMps"],
                "neutralToBeltStopMs": None if zero is None else (zero - start) * 1000,
                "observedAfterFaultMs": 1000 * (p.time - start),
                "maximumPipeDisplacementMm": 1000 * furthest,
                "peakPipeSpeedMps": vmax,
                "terminal": result,
                "softwareState": "FAULT",
                "qualifiedForTravel": False,
            }
        )
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("out", type=Path)
    ns = p.parse_args()
    inputs = [HERE / "locked.json", HERE / "traction.json"]
    ns.out.write_text(
        json.dumps(
            {
                "inputs": {f.name: digest(f) for f in inputs},
                "cases": [r for f in inputs for r in follow(f)],
            },
            indent=2,
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )
