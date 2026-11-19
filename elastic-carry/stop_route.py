"""Check each arrested chassis pose against the field, moving peers and shared supply."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "conditioned-match"))
from occupancy import n, screen, parked_peers
from supply import electric


def run(assets):
    folder = assets / "elastic-carry"
    rows = json.loads((HERE / "stop.json").read_bytes()) + json.loads(
        (HERE / "stop-late.json").read_bytes()
    )
    plan = n.read(ROOT / "fixed-height-match/placements.json")["placements"]
    field, _, team, _ = n.route.route.field.scenario(plan, inboard=True)
    stock = n.read(ROOT / "full-match/installed-stock.json")["stock"]
    drive_wall = n.read(ROOT / "relieved-match/first-feed.json")["scheduledDriveWallS"]
    results = []
    for r in rows:
        if r["stepS"] != 0.0000625:
            continue
        side = r["case"]
        source = (
            folder
            / "stop"
            / f"{side}-{r['monitor']}-{r['assumedActuationLatencyMs']}ms-62500ns.npz"
        )
        if hashlib.sha256(source.read_bytes()).hexdigest() != r["pathSha256"]:
            raise ValueError("Changed wheel stop")
        brake = np.load(source)
        until = np.searchsorted(brake["wall"], r["wheelStopWallS"] + 1e-8)
        t = brake["wall"][:until]
        pose = brake["poses"][:until]
        gap = screen(field, t, pose, stock, team, t[-1])
        hold = parked_peers(team, pose[-1], t[-1])
        opening = np.load(assets / "conditioned-match" / side / "corrected-opening.npz")
        route = np.load(folder / "routes" / (side + "-route.npz"))
        old_t = drive_wall + route["times"]
        use = old_t < t[0] - 1e-9
        time = np.r_[opening["t"], old_t[use], t, 156.0]
        points = np.vstack((opening["pose"], route["poses"][use], pose, pose[-1]))
        points[:, 2] = np.unwrap(points[:, 2])
        grid = np.arange(31201) * 0.005
        achieved = np.column_stack([np.interp(grid, time, col) for col in points.T])
        labels = np.select(
            (grid[:-1] < opening["t"][-1], grid[:-1] < drive_wall, grid[:-1] < t[-1]),
            ("opening approach", "station hold", "loaded drive"),
            default="stationary",
        )
        steps = n.route.wheel_steps(achieved)
        dep = next(
            q
            for q in n.read(ROOT / "conditioned-match/elastic.json")["cases"]
            if q.get("machinedTray") and q["case"] == side and q["stepS"] == 0.00003125
        )
        events = {
            "coral-A": {
                "seat": 21.571672635314284 + dep["firstBandJavaMs"] / 1000,
                "feed": 21.10,
                "carry": drive_wall,
            }
        }
        packs = {}
        for name, source_pack in (("screened", (11.8, 0.027)), ("weak", (11.8, 0.032))):
            power = electric(
                grid,
                steps,
                labels,
                events,
                52.16312255 + 6.17 + 7.5,
                "conditioned",
                source_pack,
                assets,
            )
            packs[name] = {
                k: power[k]
                for k in (
                    "onePipeKg",
                    "minTerminalV",
                    "peakMotorPhaseA",
                    "peakFrictionRequired",
                    "firstReferenceInhibitWallS",
                    "floorBelowOrUnsolvedS",
                )
            }
        results.append(
            {
                "case": side,
                "monitor": r["monitor"],
                "latencyMs": r["assumedActuationLatencyMs"],
                "posePathSha256": r["pathSha256"],
                "minBrakeBarrierMm": gap["minimumBarrierMm"],
                "minBrakeStockMm": gap["minimumStockMm"],
                "minBrakePeerMm": gap["minimumPeerMm"],
                "minParkedPeerMm": min(hold.values()),
                "finalChassisXYTheta": pose[-1].tolist(),
                "sourceCases": packs,
            }
        )
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.artifacts), indent=2) + "\n", encoding="utf8", newline="\n"
    )
