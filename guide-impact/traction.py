"""Screen finite drive traction and reopen the post-guide belt contact on accepted and faulted states."""

import argparse
import json
from pathlib import Path
from dataclasses import replace
import numpy as np
from beam import ROOT, digest, guide


def run_case(r, raw):
    feed = guide.feed
    folder = raw / r["id"]
    file = folder / (r["id"] + ".npy")
    if digest(file) != r["trajectorySha256"]:
        raise ValueError("Unknown tube")
    x = np.load(file, mmap_mode="r")
    cfg = replace(feed.cfg, slices=10)
    old = feed.cfg
    feed.cfg = cfg
    try:
        p = feed.Pipe(r["poseMmDeg"], 0.0, True)
        # Positive normal at each end is normalized by max(actual area, preload area).
        # Bound all possible lateral speeds and the simultaneous guide impulse before the belt step.
        ax = x[:, 4]
        az = x[:, 6]
        rx = feed.HALF * abs(ax) + feed.R * np.sqrt(np.maximum(0, 1 - ax * ax))
        rz = feed.HALF * abs(az) + feed.R * np.sqrt(np.maximum(0, 1 - az * az))
        vel = abs(x[:, 8]) + abs(x[:, 12]) * rx + abs(x[:, 10]) * rz
        radial = np.hypot(feed.HALF, feed.R)
        guide_impulse = (
            (1.0 + 0.1) * sum(r["guide"]["peakSideNormalN"]) * r["stepUs"] * 1e-6
        )
        guide_allowance = guide_impulse * (
            1 / feed.M + radial**2 / min(feed.I, feed.IP)
        )
        if r["hardStop"]:
            raise ValueError("Hard limit invalidates the remaining bound")
        lateral = float(vel.max()) + guide_allowance
        bound = 2 * cfg.belt_mu * (p.preload + 2 * p.outward + 37.52 * lateral)
        commands = r["portTransitions"]

        def command(tick):
            return next(z[4] for z in reversed(commands) if z[0] <= tick)

        # Reconstruct the exact rate-limited belt setpoint at each desktop tick.
        start = {}
        belt = 0.0
        for tick in range(0, r["lastJavaMs"] + 20, 20):
            start[tick] = belt
            belt += np.clip(
                command(tick) - belt, -0.020 * cfg.belt_accel, 0.020 * cfg.belt_accel
            )
        active_end = 100 + 1000 * r["guide"]["lastForceS"]
        start_ms = int(np.ceil(active_end)) + 1
        sample = []
        for ms in range(start_ms, r["lastJavaMs"]):
            i = int(np.searchsorted(x[:, 0], ms - 1e-7))
            if abs(x[i, 0] - ms) > 1e-6:
                raise ValueError("Missing belt sample")
            v = x[i, 7:10]
            w = x[i, 10:13]
            p.p = x[i, 1:4] / 1000
            p.a = x[i, 4:7]
            tick = 20 * (ms // 20)
            target = command(tick)
            # The force acts after updating belt speed for the next integration step.
            duration = (ms - tick) / 1000 + r["stepUs"] * 1e-6
            belt = start[tick] + np.clip(
                target - start[tick],
                -cfg.belt_accel * duration,
                cfg.belt_accel * duration,
            )
            force_x = normal = 0.0
            for side in (-1, 1):
                cells = p.footprint(side)
                area = sum(s for _, s in cells)
                for arm, part in cells:
                    travel = 1000 * (side * (p.p[1] + arm[1]) - feed.HALF)
                    if travel < -p.inward:
                        continue
                    vv = v + np.cross(w, arm)
                    n = (
                        max(0, p.preload + 2 * travel + 37.52 * side * vv[1])
                        * part
                        / max(cfg.area_for_preload, area)
                    )
                    slip = vv[0] - belt
                    fx = (
                        -cfg.belt_mu
                        * n
                        * slip
                        / max(cfg.slip_scale, np.hypot(slip, vv[2]))
                    )
                    force_x += fx
                    normal += n
            sample.append([ms, force_x, normal, belt, force_x * belt])
        sample = np.asarray(sample)
        if not len(sample) or abs(sample[:, 1]).max() > bound:
            raise ValueError("Invalid traction")
        out = folder / "belt-1ms.npy"
        np.save(out, sample)
        return {
            "id": r["id"],
            "sourceTrajectory": digest(file),
            "samples": len(sample),
            "sampledFromJavaMs": start_ms,
            "sampleHistory": digest(out),
            "postGuideSampledTotalXN": [sample[:, 1].min(), sample[:, 1].max()],
            "postGuideSampledTotalNormalMaxN": sample[:, 2].max(),
            "postGuideSampledDrivePowerRangeW": [
                sample[:, 4].min(),
                sample[:, 4].max(),
            ],
            "completeTrajectoryAbsXUpperN": bound,
            "lateralGuideVelocityAllowanceMps": guide_allowance,
            "beltMu": cfg.belt_mu,
            "preloadPerEndN": p.preload,
            "verifiedMotorForceLimitN": None,
        }
    finally:
        feed.cfg = old


def run(reg, raw):
    allrows = json.loads(reg.read_bytes())["cases"]
    rows = [
        r
        for r in allrows
        if r["stepUs"] == 31.25
        and r["layout"] == "folded"
        and not r["outwardPerSideMm"]
        and r["guideModel"][1] == 1800
    ]
    return {
        "register": digest(reg),
        "sourceBeltPlant": digest(ROOT / "return-station/feed.py"),
        "cases": [run_case(r, raw) for r in rows],
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("reg", type=Path)
    p.add_argument("raw", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.reg, a.raw), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
