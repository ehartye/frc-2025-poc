"""Continue the measured swerve shaft and gyro stop on the original match and traffic clock."""

import argparse
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "transit"))
import field_transit
from register import read, sha, write


def run(artifacts, side):
    work = artifacts / "retained-coral" / side
    wheel = read(work / "stop/route.json")
    stop = read(HERE.parent / "transit/stops.json")["cases"][side]
    cargo = stop["cargo"]
    source = (
        artifacts
        / "swerve-transit"
        / f"{side}-31.25"
        / "internal-stop-contact"
        / cargo["file"]
    ).with_suffix(".npz")
    if not (
        sha(source) == cargo["terminalStateSha256"]
        and wheel["stopFeedback"] == stop["stopFeedback"]
        and sha(work / "stop/path.npz") == cargo["sourceRoute"]
        and cargo["piece"] == "coral-A"
    ):
        raise ValueError("The retained pipe or measured stop changed")
    folder = work / "continued"
    basis = dict(
        wheel=work / "stop/end-wheels.npz",
        pipe=source,
        startWallS=wheel["endWallS"],
        nextWaypoint=len(wheel["waypoints"]),
        holdS=0.5,
        reefSetbackM=0.054,
        maxSpeedMps=0.48,
        maxAccelerationMps2=0.28,
        maxYawAccelerationRadS2=0.38,
    )
    if sha(basis["wheel"]) != wheel["terminalWheels"]:
        raise ValueError("Reconstructed shaft state does not match the registered stop")
    result = field_transit.run(
        artifacts / "loaded-swerve", folder, side, 31.25, continuation=basis
    )
    result["continuation"] = dict(
        (k, v) for k, v in basis.items() if k not in ("wheel", "pipe")
    )
    result["authorization"] = (
        "Wheel-only candidate. Requires independent occupied-pocket and endpoint observations."
    )
    write(folder / "route.json", result)


if __name__ == "__main__":
    q = argparse.ArgumentParser(description=__doc__)
    q.add_argument("artifacts", type=Path)
    q.add_argument("side", choices=("plus", "minus"))
    args = q.parse_args()
    run(args.artifacts, args.side)
