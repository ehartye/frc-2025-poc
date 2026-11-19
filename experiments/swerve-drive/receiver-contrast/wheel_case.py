"""Challenge the passive facing with the four modules' conservative all-azimuth image masks."""

import argparse
import json
from pathlib import Path

import numpy as np
from finish import select_views, witness
from native_scene import inputs


def run(cache, docks, register):
    feeds, rows, sources = inputs(docks)
    wheel = witness.load_wheel_mask(register)
    result = []
    for row in rows:
        name = row["case"]
        feed = next(
            f for f in feeds if (f["case"], f["plantStepUs"]) == (name, row["stepUs"])
        )
        surf = witness.read(cache / "surfaces.json")["docks"][name]
        if (
            surf["dockMmMmDeg"] != row["dockMmDeg"]
            or feed["pipeTrajectorySha256"] != row["trace"]
        ):
            raise ValueError("Changed attained pose or trace")
        modes = {}
        for mode, gray, whole in (
            ("bulk-black", 80, False),
            ("matched", 220, False),
            ("whole-scanline", 80, True),
        ):
            fixed, roof, _ = select_views(cache / (name + ".npz"), gray)
            mask = dict(wheel, wholeScanlines=whole)
            frames = []
            for point in row["frames"]:
                obs = witness.observe(
                    fixed,
                    roof,
                    np.asarray(point["trueCenterMm"]),
                    np.asarray(point["trueAxis"]),
                    wheel_mask=mask,
                )
                frames.append(dict(captureJavaMs=point["captureJavaMs"], **obs))
            modes[mode] = {
                "frames": frames,
                "gate15ms": witness.service_gate(frames, feed, 15),
                "gate45ms": witness.service_gate(frames, feed, 45),
            }
        result.append(
            dict(
                case=name,
                stepUs=row["stepUs"],
                trajectorySha256=row["trace"],
                treatments=modes,
            )
        )
    return dict(
        wheelMaskRegisterSha256=witness.digest(register),
        sourceObservationsSha256=sources,
        cases=result,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("cache", type=Path)
    p.add_argument("docks", type=Path)
    p.add_argument("register", type=Path)
    p.add_argument("result", type=Path)
    a = p.parse_args()
    a.result.write_text(
        json.dumps(run(a.cache, a.docks, a.register), indent=2) + "\n",
        encoding="utf8",
        newline="\n",
    )
