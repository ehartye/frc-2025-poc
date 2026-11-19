"""Recover each original lateral cell load and compute its static work on the new arm."""

import numpy as np
import joined as model
from joined import prior, screen
from arm_contact import ContactResponse


def displacement(z, f, low, compliance):
    # F is signed OUTWARD lateral force at each loaded ring cell. Roots are simple
    # vertical-position supports; their translations and the folded spine share work.
    total = f.sum()
    if total <= 1e-8:
        return
    hi = 787.0
    rlo = f @ (hi - z) / (hi - low)
    rhi = total - rlo
    support_energy_twice = rlo * rlo * compliance + rhi * rhi * screen.UPPER
    slope = np.hypot(1.0, 0.46)
    points = np.r_[z, low, hi, 625.0, 813.0] * slope
    idx = np.argsort(points)
    positions = points[idx]
    jumps = np.r_[-f, rlo, rhi, 0.0, 0.0][idx]
    dx = np.diff(positions)
    shear = np.cumsum(jumps)[:-1]
    end_moment = np.cumsum(shear * dx)
    moment = np.r_[0.0, end_moment[:-1]]
    if abs(end_moment[-1]) > 1.0e-8:
        raise ValueError("Unbalanced whole backing")
    integral = np.sum(dx * (moment**2 + moment * end_moment + end_moment**2) / 3)
    # Virtual work at every 1 mm of face height, including unloaded free tips.
    # H(p)=integral[(s-p)*M(s),s>=p]. Piecewise linear moment permits exact integration.
    rigidity = screen.E * prior.bending_inertia(4)
    area = dx * (moment + end_moment) / 2
    first = positions[:-1] * area + moment * dx**2 / 2 + shear * dx**3 / 3
    area_tail = np.r_[np.cumsum(area[::-1])[::-1], 0.0]
    first_tail = np.r_[np.cumsum(first[::-1])[::-1], 0.0]
    grid = np.arange(625.0, 814.0)
    probe = grid * slope
    i = np.clip(np.searchsorted(positions, probe, side="right") - 1, 0, len(dx) - 1)
    length = positions[i + 1] - probe
    moment_at = moment[i] + shear[i] * (probe - positions[i])
    influence = (
        first_tail[i + 1]
        - probe * area_tail[i + 1]
        + moment_at * length**2 / 2
        + shear[i] * length**3 / 3
    )
    wlo = (hi - grid) / (hi - low)
    whi = 1 - wlo
    profile = (
        wlo * rlo * compliance
        + whi * rhi * screen.UPPER
        + (wlo * influence[int(low - 625)] + whi * influence[int(hi - 625)] - influence)
        / rigidity
    )
    worst = np.argmax(abs(profile))
    interpolation_error = slope**2 * np.max(abs(end_moment)) / (8 * rigidity)
    work_average = (
        support_energy_twice + integral / (screen.E * prior.bending_inertia(4))
    ) / total
    return dict(
        averageMm=work_average,
        rigidSpineMm=support_energy_twice / total,
        lowerN=rlo,
        upperN=rhi,
        forceN=total,
        copMm=f @ z / total,
        signedWorstFaceMm=profile[worst],
        worstFaceZMm=grid[worst],
        allFaceTravelUpperMm=abs(profile[worst]) + interpolation_error,
        lowerFreeTipMm=profile[0],
        upperFreeTipMm=profile[-1],
    )


def run(input_raw, outdir):
    reg = model.read(model.ROOT / "guide-impact/results.json")["cases"]
    out = {}
    for row in [
        r
        for r in reg
        if r["layout"] == "folded"
        and r["stepUs"] == 31.25
        and r["id"].endswith("serial-carriage-recover")
    ]:
        case = row["case"]
        key = row["id"]
        raw = input_raw / key
        beam = prior.Beam(model.ROOT / "guide-mounts", "folded")
        pipe = prior.guide.GuidedPipe(
            row["poseMmDeg"],
            0.0,
            prior.guide.Guide(
                throat_mm=-0.3,
                stiffness_n_mm=10,
                damping_n_s_m=200,
                friction=0.1,
                sectors=128,
                radial_bins=4,
                x_width_mm=40,
                upper_mm=813,
                x_offset_mm=20,
            ),
        )
        source = raw / (key + ".npy")
        if (
            prior.digest(source) != row["trajectorySha256"]
            or prior.digest(raw / "guide.npy") != row["guideHistorySha256"]
        ):
            raise ValueError("Different registered contact history")
        x = np.load(source, mmap_mode="r")
        h = np.load(raw / "guide.npy")
        response = ContactResponse(beam)
        configs = {
            "front": (687.0, screen.segments("front-return")["compliance"]),
            "raised": (760.0, screen.segments("rise-+73")["compliance"]),
            "straight": (687.0, screen.ref.arm(687.0)["totalMmPerN"]),
        }
        peak = {}

        def capture(t, j, z, f):
            if f.sum() <= 1e-8:
                return
            side_configs = (
                configs if j == 1 else {"negativeYStraight": configs["straight"]}
            )
            for label, (lo, c) in side_configs.items():
                d = displacement(z, f, lo, c)
                d.update(javaMs=100 + 1000 * t)
                book = peak.setdefault(label, {})
                for m in (
                    "averageMm",
                    "rigidSpineMm",
                    "lowerN",
                    "upperN",
                    "allFaceTravelUpperMm",
                ):
                    if m not in book or abs(d[m]) > abs(book[m][m]):
                        book[m] = d

        response.capture = capture
        a = x[:, 6]
        rad = prior.guide.feed.HALF * abs(a) + prior.guide.feed.R * np.sqrt(
            np.maximum(0.0, 1 - a * a)
        )
        within = (x[:, 3] / 1000 + rad >= 0.625) & (x[:, 3] / 1000 - rad <= 0.813)
        for i in np.flatnonzero(within[:-1]):
            r = x[i]
            pipe.time = (r[0] - 100) / 1000
            pipe.p = r[1:4] / 1000
            pipe.a = r[4:7].copy()
            pipe.v = r[7:10].copy()
            w = r[10:13]
            pipe.l = (
                prior.guide.feed.IP * w
                + (prior.guide.feed.I - prior.guide.feed.IP) * (w @ pipe.a) * pipe.a
            )
            response.step(pipe, (x[i + 1, 0] - r[0]) * 0.001)
        rec = np.asarray(response.trace)
        discrepancy = np.max(abs(rec - h), axis=0)
        if discrepancy[1:3].max() > 1e-7 or discrepancy[9:11].max() > 1e-10:
            raise ValueError("Different distributed trajectory")
        out[case] = dict(
            history=row["guideHistorySha256"],
            trajectory=row["trajectorySha256"],
            maxNormalErrorN=discrepancy[1:3].max(),
            maxGuideDisplacementErrorM=discrepancy[9:11].max(),
            peak=peak,
        )
        print(
            case,
            [
                (k, v["averageMm"]["averageMm"], v["rigidSpineMm"]["rigidSpineMm"])
                for k, v in peak.items()
            ],
            flush=True,
        )
    model.replay.write(outdir / "distributed.json", out)
