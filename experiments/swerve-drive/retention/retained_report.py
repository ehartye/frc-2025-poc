"""Join same-pipe 5 ms measurements to the prior capture and both stopped chassis records."""

from argparse import ArgumentParser
from pathlib import Path
import json
import math

import numpy as np
from screen import HERE, sha, read


def permission(mm, ports, switches, age_s=0.0, prior_identity=True):
    # The finite pipe end planes intercept the two gauges at the same X/Z.
    # At tilt <=3 degrees their separation is 301.625/cos(tilt) mm.
    # Subtract the closed 296 mm free-tip spacing, then charge both gauges.
    expected_low = 301.625 - 296.0 - 0.4
    expected_high = 301.625 / math.cos(math.radians(3)) - 296.0 + 0.4
    return bool(
        prior_identity
        and 0 <= age_s <= 0.010
        and all(ports)
        and all(switches)
        and 0.65 <= min(mm)
        and max(mm) <= 7.5
        and expected_low <= sum(mm) <= expected_high
        and 0.45 * (max(mm) + 0.20) + 0.12 <= 3.50
    )


def run(artifacts):
    cases = {}
    for side in ("plus", "minus"):
        folder = artifacts / "captive-observer" / side
        source = read(folder / "receipt.json")
        prior = read(HERE / "carry.json")[side]["pipe"]
        if (
            source["source"] != prior["terminalPipe"]
            or source["sourceRoute"] != prior["wheelTrajectory"]
        ):
            raise ValueError("Changed source identity")
        previous = artifacts / "retained-coral" / side / "carry/pipe.npy"
        if (
            sha(previous) != prior["cargoTrace"]
            or sha(folder / "trace.npy") != source["trace"]
        ):
            raise ValueError("Changed capture or continuation")
        if sha(folder / "state.npz") != source["checkpoint"]:
            raise ValueError("Changed mechanical restart")
        data = np.load(folder / "trace.npy", allow_pickle=False)
        # The continuation explicitly preserves both CLOSED commands. The independent
        # switch-open fault below changes one sensed state, not the pipe position.
        allowed = [permission(row[13:15], row[15:17], [True, True]) for row in data]
        early = np.load(previous, allow_pickle=False)
        kept = source["start"]["readings"]
        mm = np.array([row[1] for row in kept])
        faults = dict(
            empty=permission([0, 0], [False, False], [True, True]),
            emptyBothPortsStuck=permission([0, 0], [True, True], [True, True]),
            biasedGauge=permission(mm + [0, 5], [True, True], [True, True]),
            oneMissingBeam=permission(mm, [True, False], [True, True]),
            oneOpenYoke=permission(mm, [True, True], [True, False]),
            oldFrame=permission(mm, [True, True], [True, True], 0.020),
            unverifiedIdentity=permission(
                mm, [True, True], [True, True], prior_identity=False
            ),
            shoulderOverload=permission([8, 1], [True, True], [True, True]),
        )
        if not all(allowed) or any(faults.values()):
            raise ValueError("Optical, geometric or force bounds violated")
        cases[side] = dict(
            source=source,
            path=source["sourceRoute"],
            physicalRestartSha256=source["checkpoint"],
            priorTraceSha256=sha(previous),
            beforeOriginalRangeStopServices=len(early),
            postInterlockServices=len(data),
            allContinuationServicesGranted=all(allowed),
            radialToAxialProbeTiltMaxDeg=float(
                np.degrees(np.arccos(np.clip(abs(data[:, 5]), 0, 1))).max()
            ),
            observedSumMm=[
                float(data[:, 13:15].sum(1).min()),
                float(data[:, 13:15].sum(1).max()),
            ],
            firstTwoIndependentGatesUnderFiveMmOffset=source["faults"][
                "calibration-offset"
            ],
            fusedFaultPermissions=faults,
        )
    return dict(
        cases=cases,
        requiredEndFaceTiltDeg=3,
        individualDisplacementErrorMm=0.20,
        clockS=0.005,
        sourcePiece="coral-A",
        matchedScore=0,
    )


if __name__ == "__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(run(args.artifacts), indent=2) + "\n", encoding="utf-8"
    )
