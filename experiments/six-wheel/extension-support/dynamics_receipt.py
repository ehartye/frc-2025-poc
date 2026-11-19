"""Reconcile the issued signed extension controls with their binary trajectories."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def cases():
    for side in ("leftSmall", "rightSmall"):
        for dt in (62.5, 31.25, 15.625):
            yield side, "nominal", dt
        for dt in (62.5, 31.25):
            yield side, "no-buttons", dt
        for variant in ("retracted-slow", "retracted-slower"):
            yield side, variant, 62.5
    for side, variants in (
        ("leftSmall", ("legacy", "bare", "wide")),
        ("rightSmall", ("soft", "forward-cg", "no-keepers")),
    ):
        for variant in variants:
            yield side, variant, 62.5


def report(folder):
    keep = read(HERE / "keepers.json")
    source = ROOT / "split-guide-acquisition/restart.json"
    motion = read(HERE / "dynamics-motion.json")
    files = [
        "extension-support/keepers.json",
        "extension-support/geometry.json",
        "extension-support/balance.json",
        "extension-support/rail-fixture.step",
        "extension-support/dynamics-motion.json",
        "extension-support/dynamics-witness.json",
        "extension-support/dynamics.py",
        "extension-support/dynamics_run.py",
        "extension-support/dynamics_input.py",
        "extension-support/dynamics_surface.py",
        "extension-support/dynamics_witness.py",
        "extension-support/dynamics_motion.py",
        "compliant-carriage/model.py",
        "compliant-carriage/boundary.json",
        "shelf-structure/response.py",
        "shelf-structure/coupled.py",
        "shelf-structure/installed.py",
        "shelf-structure/tray.py",
        "split-guide-acquisition/restart.json",
        "integrated-slide/mass.json",
        "fixed-lift-omission/mechanics.json",
    ]
    for p, sha in motion["sources"].items():
        if digest(ROOT / p) != sha:
            raise ValueError("Stale motion data: " + p)
    rows = {}
    for side, variant, dt in cases():
        key = f"{side}-{variant}-{dt:g}us"
        raw = folder / (key + ".npz")
        row = read(folder / (key + ".json"))
        if (
            row["stateSourceSha256"] != digest(source)
            or row["keeperStepSha256"] != keep["stepSha256"]
            or digest(raw) != row["traceSha256"]
            or row["label"] != key
        ):
            raise ValueError("Unregistered run " + key)
        with np.load(raw, allow_pickle=False) as series:
            if series["q"].shape[1] != 19 or len(series["timeS"]) < 1000:
                raise ValueError("Invalid structural coordinate inventory")
        button = row["activeButtons"]
        rows[key] = dict(
            timeStepUs=dt,
            durationS=row["durationS"],
            buttonAxialModel=button,
            inputCase=side,
            localPipeCenterM=row["localPipePositionM"],
            tubeAxis=row["tubeAxis"],
            traceSha256=row["traceSha256"],
            sampledEveryUs=row["sampledEveryUs"],
            firstPossibleSideStrike=row["firstPossibleEdgeState"] if button else None,
            peakPossibleSideStrike=row["peakPossibleEdgeState"] if button else None,
            peakGeometricRetractionMm=row["geometricRetractionFromFreeTopMm"],
            peakRailN=row["peakRailN"],
            peakButtonN=row["peakButtonN"],
            lowestMeanNormal=row["minimumMeanNormalState"],
            firstMeanTension=row["firstNegativeMeanNormalS"],
            lowestStationNormalMps2=row["minimumLocalNormalMps2"],
            startProbeMm=row["startProbeMm"],
            finalProbeMm=row["finalProbeMm"],
            finalChangeMm=row["finalChangeFromOwnStartMm"],
            peakSlopeRad=row["peakSlopeRad"],
            projectedLegacyDisturbance=row["projectedLegacyDisturbance"],
            discardedAntisymmetricAmplitudeMm=row["ignoredRollDisturbanceMm"],
            prescribedPeakAccelerationMps2=row["peakPrescribedAccelerationMps2"],
            lastStrokeMm=row["lastStrokeMm"],
            largeRotationStopS=row["firstLargeRotationS"],
        )
    for side in ("leftSmall", "rightSmall"):
        clocks = [rows[f"{side}-nominal-{dt:g}us"] for dt in (62.5, 31.25, 15.625)]
        if not all(
            20 < r["firstPossibleSideStrike"]["strokeMm"] < 22
            and 152 < r["firstMeanTension"]["strokeMm"] < 155
            and r["lowestMeanNormal"]["requiredMeanNormalMps2"] < 0
            and r["largeRotationStopS"] is None
            and r["lastStrokeMm"] == 370
            for r in clocks
        ):
            raise ValueError("Numerical clocks do not agree on the adverse cases")
        if np.ptp([r["finalChangeMm"][-1] for r in clocks]) > 0.1:
            raise ValueError("Nose response failed clock comparison")
        rates = [
            rows[f"{side}-{v}-62.5us"] for v in ("no-buttons", "retracted-slow", "retracted-slower")
        ]
        if any(r["firstMeanTension"] is None for r in rates):
            raise ValueError("Duration control changed")
    witness = read(HERE / "dynamics-witness.json")
    if witness["traceSha256"] != rows["leftSmall-nominal-62.5us"]["traceSha256"]:
        raise ValueError("Different witness trace")
    return dict(
        sources={p: digest(ROOT / p) for p in files},
        nativeStepSha256=keep["stepSha256"],
        orderedRailReactions=[
            "inner rear floor",
            "inner front floor",
            "inner upper crosspin",
            "middle rear floor",
            "middle front floor",
            "middle upper dog",
        ],
        probeLocalXY="rear (-100,0), signed input pipe X at Y0, nose (165,0), mm",
        cases=rows,
        wholeTubeCustody=None,
        achievedPlacement=None,
        scope="Prescribed stroke, held horizontal pipe coordinates and a distributed vertical mass. Figures after the first side interference or required tensile support are counterfactual fixture demands.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("traces", type=Path)
    p.add_argument("output", type=Path, nargs="?")
    p.add_argument(
        "--run-cases",
        action="store_true",
        help="Generate only the twenty raw runs into the declared trace directory",
    )
    a = p.parse_args()
    if a.run_cases:
        import subprocess, sys

        for side, variant, dt in cases():
            subprocess.run(
                [
                    sys.executable,
                    str(HERE / "dynamics_run.py"),
                    str(a.traces),
                    "--case",
                    side,
                    "--variant",
                    variant,
                    "--dt-us",
                    str(dt),
                ],
                check=True,
            )
        raise SystemExit(0)
    if a.output is None:
        p.error("Give an output receipt or --run-cases")
    a.output.write_bytes((json.dumps(report(a.traces), indent=2, allow_nan=False) + "\n").encode())
