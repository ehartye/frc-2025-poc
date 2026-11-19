"""Find simultaneous acceleration boundaries for the chosen four-module hardware."""

import numpy as np
from electrical import (
    HERE,
    IDS,
    branches,
    DRIVE_SUPPLY_R,
    STEER_SUPPLY_R,
    DRIVE_PHASE_R,
    STEER_PHASE_R,
)
from screen import result, write


def line(
    accels,
    speed,
    rate=0.0,
    yaw_acceleration=0.0,
    mass=65.0,
    pack="screened",
    receiver="stowed",
    adverse=False,
):
    aa = np.asarray(accels, dtype=float)

    def four(x):
        return np.broadcast_to(np.asarray(x), (4, len(aa))).T.copy()

    # Four matched contact shares on a straight/strafe segment. Rolling
    # resistance hypothesis 2% of weight, including at forward launch.
    expected = mass * (aa + 0.02 * 9.80665) / 4
    return result(
        np.arange(len(aa)) * 0.005,
        four(expected),
        four(0.0),
        four(mass * 9.80665 / 4),
        four(speed),
        four(aa),
        four(rate),
        four(yaw_acceleration),
        pack=pack,
        receiver=receiver,
        adverse=adverse,
    )


def limits(speed, rate, alpha, pack, receiver, adverse=False):
    # Rows are independent design points. The numeric wall index is only a
    # lookup into acceleration, never elapsed match time.
    grid = np.arange(0, 5.0001, 0.025)
    summary, _ = line(grid, speed, rate, alpha, pack=pack, receiver=receiver, adverse=adverse)
    first = summary["firstInhibitWallS"]
    last = len(grid) - 1 if first is None else round(first / 0.005) - 1
    return dict(
        speedMps=speed,
        steerRateRadS=rate,
        steerAccelRadS2=alpha,
        pack=pack,
        receiver=receiver,
        adverse=adverse,
        highestContinuousTestedAccelMps2=None if last < 0 else float(grid[last]),
        firstFailingAccelMps2=None if first is None else float(grid[last + 1]),
        firstFailedGates=[
            k for k, v in summary["firstByGate"].items() if first is not None and v == first
        ],
    )


def main():
    hardware = branches()
    write(
        HERE / "branches.json",
        dict(
            order=IDS,
            branches=hardware,
            maxLoadedSupplyLoopOhm={"drive": DRIVE_SUPPLY_R, "steer": STEER_SUPPLY_R},
            effectivePhaseLoopOhm={"drive": DRIVE_PHASE_R, "steer": STEER_PHASE_R},
            configuration="Proposed topology and hot lead budgets, eight separate controllers; service/routing validation pending.",
        ),
    )
    combos = [
        (v, 0.0, 0.0, p, "stowed", False)
        for p in ("nominal", "screened", "weak")
        for v in (0, 1, 2, 3)
    ]
    combos.extend(
        [(1.0, 4.0, 25.0, "screened", r, False) for r in ("stowed", "loaded", "actuating")]
    )
    combos.append((1.0, 4.0, 25.0, "screened", "stowed", True))
    write(
        HERE / "corners.json",
        dict(
            massKg=65.0,
            equalCornerLoads=True,
            stepMps2=0.025,
            trials=[limits(*row) for row in combos],
            physicalMassKg=None,
        ),
    )


if __name__ == "__main__":
    main()
