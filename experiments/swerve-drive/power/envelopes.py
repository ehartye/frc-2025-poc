"""Publish original stowed stationary body and cable occupancy for scene comparison."""

import json
import numpy as np
from electrical import HERE
from screen import sha, write
from cable_routes import fillet, segbox


def boxes():
    fit = json.loads((HERE / "controller-fit.json").read_bytes())
    wire = json.loads((HERE / "paths.json").read_bytes())
    result = {}
    for site, v in fit["corners"].items():
        for k, data in v["fit"].items():
            if k.endswith("-service"):
                continue
            result[site + "/" + k] = data["boundsMm"]
    for group in ("groupedSupplyTargets", "motorAndSensorTargets"):
        for k, r in wire[group].items():
            if r.get("unresolved") or r["lowerGapMm"] <= 0:
                raise ValueError("Unqualified centerline " + k)
            pts = fillet([np.array(v) for v in r["polylineVerticesMm"]], r["bendRadiusMm"])
            for i, (a, b) in enumerate(zip(pts, pts[1:])):
                if np.linalg.norm(b - a) < 1e-6:
                    continue
                pad = r["radiusMm"] + 0.06
                result["wire/" + k + "/" + str(i)] = (
                    segbox(a, b) + np.r_[np.full(3, -pad), np.full(3, pad)]
                ).tolist()
    return dict(
        stationaryBoundsMm=result,
        shapes=len(result),
        frame="robot mm; part boxes and conservative capsule-segment AABBs",
        coreSources={p: sha(HERE / p) for p in ("controller-fit.json", "paths.json")},
        omitted=[
            "connector mating tails",
            "individual signal breakouts",
            "final fastener heads and clips",
        ],
    )


if __name__ == "__main__":
    write(HERE / "envelopes.json", boxes())
