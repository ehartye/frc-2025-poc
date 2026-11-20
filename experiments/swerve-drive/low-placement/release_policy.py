"""Compare a slower leading or trailing belt on the same confirmed extended CORAL."""

import argparse
import json
from pathlib import Path

import delivery


def run(artifacts, side, dt, slower_side, resume):
    original = delivery.captivity.restore_station
    fractions = [0.4 if label == slower_side else 1.0 for label in ("right", "left")]
    calls = 0
    max_speed = [0.0, 0.0]

    def restore(*args):
        pipe, step, context = original(*args)
        raw = pipe.belt_surface_speeds

        def speeds(target, dt):
            nonlocal calls
            rates = raw(target, dt)
            selected = tuple(r * f for r, f in zip(rates, fractions))
            if target:
                calls += 1
                for i, v in enumerate(selected):
                    max_speed[i] = max(max_speed[i], abs(v))
            return selected

        pipe.belt_surface_speeds = speeds
        return pipe, step, context

    label = "slow-" + slower_side
    delivery.captivity.restore_station = restore
    try:
        delivery.run(artifacts, side, 337.0, dt, label, 7.2, resume, True)
    finally:
        delivery.captivity.restore_station = original
    path = (
        artifacts
        / "low-placement/advance-25"
        / side
        / f"joined-337mm-{dt:g}us-{label}.json"
    )
    record = delivery.read(path)
    if calls == 0:
        raise ValueError("Independent belt rate not applied")
    record["beltPolicy"] = dict(
        rightLeftSpeedFractions=fractions,
        appliedTicks=calls,
        peakRightLeftSurfaceMps=max_speed,
        source=delivery.digest(Path(__file__)),
    )
    path.write_bytes((json.dumps(record, indent=2) + "\n").encode())


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=("minus", "plus"))
    p.add_argument("slower", choices=("left", "right"))
    p.add_argument("--dt-us", type=float, default=62.5)
    p.add_argument("--from-extension", action="store_true")
    a = p.parse_args()
    run(a.artifacts, a.side, a.dt_us, a.slower, a.from_extension)
