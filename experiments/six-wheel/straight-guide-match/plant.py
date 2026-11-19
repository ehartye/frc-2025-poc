"""Restore the flexible shelf, both bearing stages, pipe, guide backings and dampers."""

from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def read(path):
    return json.loads(path.read_bytes())


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class Selection:
    def __init__(self, micros, attained=False):
        self.attained = attained
        self.micros = micros
        self.label = ("measured-start" if attained else "arrived") + f"-{micros:g}us"
        self.stage_peak_mm = 0
        self.pupil_guard_exceeded = None

    def restore(self, assets, side):
        sys.path.insert(0, str(ROOT / "guide-impact"))
        spec = importlib.util.spec_from_file_location(
            "mounted_arrival", ROOT / "guide-impact/replay.py"
        )
        arrival = importlib.util.module_from_spec(spec)
        prior = sys.modules.pop("rehearse", None)
        try:
            spec.loader.exec_module(arrival)
            spec = importlib.util.spec_from_file_location(
                "match_carriage", ROOT / "compliant-carriage/model.py"
            )
            carriage = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(carriage)
        finally:
            if prior is not None:
                sys.modules["rehearse"] = prior
        source = read(ROOT / "guide-impact/results.json")
        identity = (
            f"{side}-folded-{self.micros:g}us-1800Hz-0.236-5460-serial-carriage-recover"
        )
        if self.attained:
            row = read(HERE / f"feed-{side}-{self.micros:g}.json")
            opening = read(HERE / "revised-opening.json")[side]
            x, y, h = opening["reachedMmDeg"]
            if row["id"] != identity or row["poseMmDeg"] != [-x, -y, h]:
                raise ValueError("Feed and attained wheel pose disagree")
            folder = assets / "straight-guide-match/attained-feed" / identity
        else:
            row = next(r for r in source["cases"] if r["id"] == identity)
            folder = assets / "guide-impact" / identity
        raw = folder / (identity + ".npy")
        start = folder / "state.npz"
        if (
            digest(start) != row["stateSha256"]
            or digest(raw) != row["trajectorySha256"]
        ):
            raise ValueError("Changed station arrival")
        beam = arrival.Beam(ROOT / "guide-mounts", "folded", 1800, arm_fraction=0.236)
        if beam.receipt() != source["guideModels"]["folded/1800/0.236"]:
            raise ValueError("Changed guide model")

        class Pipe(arrival.MovingGuides, carriage.FlexiblePipe):
            pass

        Pipe.settings = (1.0, 5e6, 1.0, 0.002, 0.0001, 1.0, 2)
        coupled = arrival.coupled
        pipe = Pipe(
            row["poseMmDeg"],
            0.0,
            replace(coupled.GUIDE, sectors=128, radial_bins=4, upper_mm=813),
            coupled.Shelf(400000, 4, stations=48),
            True,
            2300.0,
            0.02,
            0.0,
            1500.0,
            (-0.066, 0.042),
            0.0001,
            True,
        )
        pipe.guides = arrival.GuideResponse(beam)
        with np.load(start, allow_pickle=False) as state:
            if not np.array_equal(state["freq"], pipe.freq) or not np.array_equal(
                state["guide_w"], beam.w
            ):
                raise ValueError("Changed shelf or guide modes")
            for key in ("p", "a", "v", "l", "z", "zdot", "plunger"):
                setattr(pipe, key, state[key].copy())
            pipe.time = float(state["time"])
            pipe.belt = float(state["belt"])
            pipe.touch = json.loads(str(state["touch"]))
            pipe.first_belts = [
                None if np.isnan(t) else float(t) for t in state["first_belts"]
            ]
            pipe.guides.q = state["guide_q"].copy()
            pipe.guides.v = state["guide_v"].copy()
        pipe.guides.last = pipe.time
        self.restore_state = {
            "pipeTime": pipe.time,
            "shelfStates": len(pipe.z),
            "guideModes": list(pipe.guides.q.shape),
            "beltMps": pipe.belt,
        }
        return pipe, row, row, raw

    def observe_boundary(self, pipe, t):
        motion = max(abs(pipe.model["relative"] @ pipe.z)) * 1000
        self.stage_peak_mm = max(self.stage_peak_mm, motion)
        if motion > 0.05 and self.pupil_guard_exceeded is None:
            self.pupil_guard_exceeded = float(t)

    def terminal(self, path, pipe, actuator, targets, pending, last_good):
        # Keep all generalized coordinates and the velocity/observer state for a
        # downstream model that provides its own stroke-dependent bearing geometry.
        arrays = {
            k: getattr(pipe, k)
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
            )
        }
        arrays.update(
            guide_q=pipe.guides.q,
            guide_v=pipe.guides.v,
            guide_last=pipe.guides.last,
            targets=targets,
            last_good=last_good,
            requested_ticks=np.array(
                [[tick, stamp, *target] for tick, stamp, status, target in pending]
            ).reshape(-1, 4),
            pending_status=np.array(
                [status for _, _, status, _ in pending], dtype="U32"
            ),
        )
        for key in (
            "speed",
            "actual",
            "goal",
            "command",
            "age",
            "steps",
            "follow_fault",
        ):
            arrays["idler_" + key] = getattr(actuator, key)
        np.savez(path, **arrays)
        return digest(path)

    def path(self, assets, side, variant):
        from retention import nav

        trial = next(
            r
            for r in read(ROOT / "reef-inertial-observer/results.json")
            if r["side"] == side and r["mode"] == variant
        )
        stage = assets / "conditioned-match" / side / "slow/A/transit.npz"
        path = (
            assets
            / "reef-inertial-observer/sampled"
            / side
            / ("coral-A-" + variant + ".csv")
        )
        transit_sha = read(ROOT / "conditioned-match/slow-navigation.json")[side][
            "transitSha256"
        ]
        if self.attained:
            trial = next(
                r
                for r in read(HERE / "anchored-route.json")
                if r["side"] == side and r["mode"] == variant
            )
            folder = assets / "straight-guide-match/anchored-route" / side
            path = folder / ("coral-A-" + variant + ".csv")
            stage = folder / "transit.npz"
            transit_sha = trial["transitSha256"]
        if digest(path) != trial["motionSHA256"] or digest(stage) != transit_sha:
            raise ValueError("Changed drive source")
        wheel = np.loadtxt(path, delimiter=",", skiprows=1)
        wheel[:, 1:3] /= 1000
        wheel[:, 1] += nav.corrected().LIP[0] - 0.490
        wheel[:, 2] += nav.corrected().LIP[1]
        with np.load(stage) as earlier:
            times = earlier["times"].copy()
            pose = earlier["poses"].copy()
        wheel[:, 3] += round((pose[-1, 2] - wheel[0, 3]) / (2 * np.pi)) * 2 * np.pi
        if np.max(abs(pose[-1] - wheel[0, 1:])) > 1e-7:
            raise ValueError("Discontinuous route")
        return (
            np.r_[times, times[-1] + wheel[1:, 0] / 1000],
            np.vstack((pose, wheel[1:, 1:])),
            trial,
        )
