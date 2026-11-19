"""Reopen the machined support to check the abrupt entry predicted by the sampled section."""

import argparse
import json
from pathlib import Path
import numpy as np
from rail_geometry import ROOT, button_contact, exchange, read
from tray import FACE, TRAY, cad, common, digest, unite, volume


def run(trace, receipt):
    row = read(receipt)
    if digest(trace) != row["traceSha256"]:
        raise ValueError("Different dynamic trajectory")
    kit = ROOT / "shelf-structure"
    machined = read(kit / "installed.json")
    parts = exchange(
        kit / "stow-receiver.step", machined["manufacturingSha256"], machined["volumeMm3"]
    )
    stock = unite(parts[FACE], parts[TRAY])
    pin = read(kit / "damper-fit.json")
    top = pin["supplierButtonTopMm"]
    from dynamics_surface import SurfaceTrace

    body = SurfaceTrace(trace, row["traceSha256"])
    results = []
    for s in (21.0, 21.25, 21.375, 21.5, 22.0, 24.0):
        t = float(np.interp(s / 1000, body.stroke, body.time))
        j = min(np.searchsorted(body.time, t), len(body.q) - 1)
        root_height = float(1000 * (body.q[j, 0] + 0.118 * body.q[j, 1]))
        for side in pin["results"]:
            x, y = side["xMm"], side["yMm"]
            dx = (x - s) / 1000
            fixture = body.at(t, dx, y / 1000)
            drop = root_height - fixture["verticalM"] * 1000
            delta = button_contact(stock, s, x, y, top)
            # Translate the straight-sided button in the shelf frame. Its
            # nominal top and disk radius are the original geometry witness.
            # Local slope/curvature across the disk and cap rounding remain open.
            probe = cad.brep(["cyl", [x - s, y, top + drop - 5, 0, 0, 1, 3.2, 5]])
            overlap = volume(common(stock, probe))
            results.append(
                dict(
                    strokeMm=s,
                    seconds=t,
                    localCenterMm=[x - s, y],
                    sectionGapMm=delta["firstGapMm"],
                    fittedApproachMm=drop,
                    commonWithFreeFlatButtonMm3=overlap,
                )
            )
    for y in (-66.0, 42.0):
        same = [r for r in results if r["localCenterMm"][1] == y]
        if (
            same[0]["commonWithFreeFlatButtonMm3"] > 1e-7
            or same[-1]["commonWithFreeFlatButtonMm3"] < 0.01
        ):
            raise ValueError("Native flat-cap adverse control did not reproduce")
    return dict(
        traceSha256=digest(trace),
        shelfSha256=machined["manufacturingSha256"],
        cases=results,
        source="Original flat disk proxy on the issued damper axis; installed upper cap profile, transverse strength and contact dynamics unqualified",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("trace", "receipt", "output"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    a.output.write_bytes(
        (json.dumps(run(a.trace, a.receipt), indent=2, allow_nan=False) + "\n").encode()
    )
