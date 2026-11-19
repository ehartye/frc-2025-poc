"""Register prior/successor enrollments, terminal states and six true exposure poses."""

import json
import numpy as np
import joined as model
from reconstruct import check

DIGEST = model.prior.digest
EXPOSURES = (5360, 5372, 5389, 5406, 5423, 5440, 5457, 5460)


def run(raw, config, window_dir):
    upstream = model.ROOT / "guide-impact/results.json"
    original = model.read(upstream)
    arms = model.Pair(model.ROOT / "guide-mounts", "front-return", 1800, 0.236, config)
    rows, packets = [], []
    for rfile in sorted(raw.glob("*/result.json")):
        folder = rfile.parent
        r = model.read(rfile)
        parent = next(
            s
            for s in original["cases"]
            if s["case"] == r["case"]
            and s["stepUs"] == r["stepUs"]
            and s["layout"] == "folded"
            and s["id"].endswith("serial-carriage-recover")
        )
        if (
            parent["programSha256"] != r["programSha256"]
            or parent["poseMmDeg"] != r["poseMmDeg"]
            or parent["carriageSupport"] != r["carriageSupport"]
            or r["binding"] != arms.receipt()
        ):
            raise ValueError("Different approach, program, carriage or guide")
        paths = (
            folder / (r["id"] + ".npy"),
            folder / "guide.npy",
            folder / "guide-modes.npy",
            folder / "state.npz",
        )
        if [DIGEST(p) for p in paths] != [
            r["trajectorySha256"],
            r["guideHistorySha256"],
            r["guide"]["guideModesSha256"],
            r["stateSha256"],
        ]:
            raise ValueError("Different source trajectory, guide, modes or state")
        pipe, guide, modes = [np.load(p, mmap_mode="r") for p in paths[:3]]
        np.testing.assert_array_equal(guide[:, 0], modes[:, 0])
        nq = arms.w.size
        q, v = [modes[-1, i : i + nq].reshape(arms.w.shape) for i in (1, 1 + nq)]
        last = float(modes[-1, 0])
        tail = pipe[pipe[:, 0] > 100 + 1000 * last + 1e-7]
        z = tail[:, 6]
        max_top = max(
            tail[:, 3]
            + 1000
            * (
                model.prior.guide.feed.HALF * abs(z)
                + model.prior.guide.feed.R * np.sqrt(np.maximum(0, 1 - z * z))
            )
        )
        if max_top >= 625:
            raise ValueError("Unbounded guide motion")
        state = np.load(paths[-1], allow_pickle=False)
        final_q, final_v = arms.advance(
            float(state["time"]) - last, q, v, np.zeros_like(q)
        )
        for name, value in [
            ("guide_q", final_q),
            ("guide_v", final_v),
            ("guide_w", arms.w),
            ("p", pipe[-1, 1:4] / 1000),
            ("a", pipe[-1, 4:7]),
            ("v", pipe[-1, 7:10]),
        ]:
            np.testing.assert_allclose(value, state[name], rtol=1e-12, atol=1e-20)
        obs = {
            k: r[k]
            for k in (
                "id",
                "case",
                "poseMmDeg",
                "stepUs",
                "lastJavaMs",
                "lastState",
                "firstBandMs",
                "portTransitions",
                "beltMps",
                "loss",
                "hardStop",
                "finalPositionMm",
                "finalAxis",
                "programSha256",
                "trajectorySha256",
                "guideHistorySha256",
                "stateSha256",
                "peakShelfNormalN",
                "damperWorkJ",
                "guide",
                "carriageSupport",
            )
        }
        obs["original"] = {
            k: parent[k]
            for k in (
                "id",
                "lastState",
                "firstBandMs",
                "lastJavaMs",
                "trajectorySha256",
                "guideHistorySha256",
                "stateSha256",
            )
        }
        obs["contactReplay"] = check(arms, r["poseMmDeg"], pipe, guide, modes)
        # Native NPZ fields use the issued solver's SI units. Preserve all advancing
        # coordinates and velocities so another stage can start at this same instant.
        obs["terminalMechanicalState"] = {
            k: state[k].tolist()
            for k in (
                "p",
                "a",
                "v",
                "l",
                "z",
                "zdot",
                "plunger",
                "time",
                "belt",
                "freq",
                "casePose",
                "guide_q",
                "guide_v",
                "guide_w",
            )
        }
        obs["terminalMechanicalState"].update(
            touch=json.loads(str(state["touch"])),
            first_belts=[
                None if np.isnan(t) else float(t) for t in state["first_belts"]
            ],
            guideLastTimeS=float(state["time"]),
            hard=r["hardStop"],
            loss=r["loss"],
            heldJavaCommandMps=r["portTransitions"][-1][4],
            shelfDisplacementSlopeTable=r["shelfDisplacementSlopeTable"],
            guideDisplacementTable=r["guideDisplacementTable"],
            guideSlopeTable=r["guideSlopeTable"],
        )
        forces = []
        for side, arm in enumerate(arms.sides):
            phi = arm.at(np.array([0.687, 0.787]))[0]
            n = len(arm.w)
            amp = modes[:, 1 : 1 + nq].reshape(-1, 2, nq // 2)[:, side, :n]
            forces.append((amp @ phi.T) * np.array(arm.attachment)[:, 2])
        obs["modeSpringForceMinMaxN"] = [
            {"min": f.min(axis=0).tolist(), "max": f.max(axis=0).tolist()}
            for f in forces
        ]
        obs["eligibleForStationImageReplay"] = (
            r["lastJavaMs"] == 5460
            and r["lastState"] == "IN_BAND"
            and r["beltMps"] == 0
            and not r["loss"]
            and not r["hardStop"]
        )
        rows.append(obs)
        if not obs["eligibleForStationImageReplay"]:
            continue
        samples = []
        for tick in EXPOSURES:
            i = int(np.argmin(abs(pipe[:, 0] - tick)))
            if abs(pipe[i, 0] - tick) > 1e-6:
                raise ValueError("Missing exact camera exposure")
            dt = (tick - 100) * 0.001 - last
            coord, vel = arms.advance(dt, q, v, np.zeros_like(q))
            # exp(-zeta*w_min*t) and modal energy give upper face amplitude.
            compliance = (
                np.max(np.sum((arms.phi / arms.w[:, None, :]) ** 2, axis=2), axis=1)
                ** 0.5
            )
            bound = (
                1000
                * compliance
                * np.sqrt(2 * guide[-1, 11:13])
                / np.sqrt(1 - arms.damping)
                * np.exp(-arms.damping * arms.w.min(axis=1) * dt)
            )
            samples.append(
                {
                    "javaMs": tick,
                    "centerMm": pipe[i, 1:4].tolist(),
                    "axis": pipe[i, 4:7].tolist(),
                    "velocityMps": pipe[i, 7:10].tolist(),
                    "omegaRadS": pipe[i, 10:13].tolist(),
                    "allFaceModeledAbsUpperMm": bound.tolist(),
                    "guideQ": coord.tolist(),
                    "guideV": vel.tolist(),
                }
            )
        final_window = pipe[pipe[:, 0] >= 5360 - 1e-7]
        count = 1 + round(100000 / r["stepUs"])
        index = np.rint((final_window[:, 0] - 5360) * 1000 / r["stepUs"]).astype(int)
        if not np.array_equal(np.unique(index), np.arange(count)):
            raise ValueError("Incomplete camera window")
        packet_dir = window_dir / folder.name
        packet_dir.mkdir(parents=True, exist_ok=True)
        file = packet_dir / "last-100ms.npy"
        np.save(file, final_window)
        packets.append(
            dict(
                id=r["id"],
                case=r["case"],
                stepUs=r["stepUs"],
                programSha256=r["programSha256"],
                guideModel=["front-return", 1800, 0.236],
                outwardPerSideMm=0.0,
                carriageSupport=r["carriageSupport"],
                trajectorySha256=r["trajectorySha256"],
                modeStateSha256=r["stateSha256"],
                guideHistorySha256=r["guideHistorySha256"],
                modalHistorySha256=DIGEST(paths[2]),
                last100msSha256=DIGEST(file),
                last100msRows=len(final_window),
                nominalGridRows=count,
                extraBoundarySubstepAtJavaMs=final_window[1:, 0][
                    np.diff(index) == 0
                ].tolist(),
                lastGeometricGuideStepLipS=last,
                maxPipeTopAfterGuideMm=max_top,
                frames=samples,
                actualOpticalCustody=None,
            )
        )
    if sorted((r["case"], r["stepUs"]) for r in rows) != [
        (side, dt) for side in ("leftSmall", "rightSmall") for dt in (15.625, 31.25)
    ]:
        raise ValueError(
            "The final register requires both signed poses at both solver steps"
        )
    return dict(
        original=DIGEST(upstream),
        route=arms.source,
        model=arms.receipt(),
        cases=rows,
        physicalCustody=None,
    ), dict(
        original=DIGEST(upstream),
        route=arms.source,
        frameClock="Java milliseconds; physical chute lip at Java 100",
        packets=packets,
    )
