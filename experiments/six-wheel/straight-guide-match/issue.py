"""Check the issued opening, finite disable and hold outcome on the full match clock."""

import argparse, json, math, sys
from pathlib import Path
import numpy as np
from plant import ROOT, HERE, read, digest

sys.path.insert(0, str(ROOT / "conditioned-match"))
from supply import n, electric
from field import screen, parked_peers


def run(assets):
    stock = read(ROOT / "full-match/installed-stock.json")["stock"]
    scenarios = read(ROOT / "fixed-height-match/placements.json")["placements"]
    f, _, team, _ = n.route.route.field.scenario(scenarios, inboard=True)
    out = {}
    for variant in (
        "opening",
        "revised-opening",
        "revised-opening-late",
        "revised-opening-interrupt",
        "revised-opening-noContrast",
    ):
        for side, entry in read(HERE / (variant + ".json")).items():
            path = assets / "straight-guide-match" / (side + "-" + variant + ".npz")
            if digest(path) != entry["pathSha256"]:
                raise ValueError("Unexpected opening path")
            with np.load(path) as data:
                t, p = data["t"], data["pose"]
            motions = n.route.wheel_steps(p)
            power = {}
            for label, pack in (("screened", (11.8, 0.027)), ("weak", (11.8, 0.032))):
                r = electric(
                    t,
                    motions,
                    np.full(len(motions), "opening"),
                    {},
                    65.83312255,
                    "straight guide",
                    pack,
                    assets,
                )
                power[label] = {
                    k: r[k]
                    for k in (
                        "peakMotorPhaseA",
                        "minTerminalV",
                        "firstReferenceInhibitWallS",
                        "peakFrictionRequired",
                    )
                }
            field = screen(f, t, p, stock, team, t[-1])
            disabled = p[(t >= 15) & (t <= 18)]
            center = p[np.searchsorted(t, 15)]
            tape = (690.875 - 144 - 65.5 - 88) * f.IN
            extent = 0.425 * (abs(math.cos(center[2])) + abs(math.sin(center[2])))
            out[side + "-" + variant] = dict(
                power=power,
                stationWallS=entry["stationWallS"],
                sourceSha256=entry["pathSha256"],
                minBarrierMm=field["minimumBarrierMm"],
                minStockMm=field["minimumStockMm"],
                minPeerMm=field["minimumPeerMm"],
                stationHoldPeerMm=min(
                    parked_peers(team, p[-1], max(t[-1], 18)).values()
                ),
                leaveTapeMm=float((center[0] - extent - tape) * 1000),
                disabledTranslationMm=float(
                    1000
                    * np.linalg.norm(np.diff(disabled[:, :2], axis=0), axis=1).sum()
                ),
                disabledYawDeg=float(np.rad2deg(abs(np.diff(disabled[:, 2])).sum())),
                heldUntilWallS=153,
                finalPose=p[-1].tolist(),
            )
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    args = p.parse_args()
    (HERE / "issued-opening.json").write_text(
        json.dumps(run(args.assets), indent=2) + "\n", encoding="utf8", newline="\n"
    )
