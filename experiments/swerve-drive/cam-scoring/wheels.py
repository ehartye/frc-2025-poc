"""Start the loaded four-module route from the changed cam's actual stopped station wheels."""

from argparse import ArgumentParser
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "transit"))
import field_transit as F
from register import read, sha, write
from departure_drive import run as station_wheels


def run(artifacts, side, stage):
    raw = artifacts / "cam-scoring" / side
    raw.mkdir(exist_ok=True, parents=True)
    r4 = artifacts / "r4-score-stroke"
    name = f"cam-station-{side}-31.25us-0.1mm"
    service = read(r4 / (name + "-service.json"))
    optics = read(artifacts / "cam-scoring/optics.json")[side]
    if (
        optics["counts"] != {"65": 6, "215": 0}
        or optics["piece"] != "coral-A"
        or optics["pipe"] != service["incoming"]
    ):
        raise ValueError(
            "Exact station frames do not support a fresh identified capture"
        )
    if stage == "station":
        record = station_wheels(
            side, artifacts / "swerve-energy/reciprocal", 0, raw, service
        )
        write(raw / "station-wheels.json", record)
        return record
    ready = read(raw / "station-piece.json")
    before = read(raw / "station-wheels.json")
    cargo = raw / "station-piece.npz"
    w = raw / (side + "-0-wheels.npz")
    if not (
        ready["allGranted"]
        and ready["before"] == service["outgoing"]
        and ready["wheel"] == before["trajectory"]
        and abs(ready["endWallS"] - (before["startWallS"] + before["elapsedS"])) < 1e-8
        and ready["state"] == sha(cargo)
    ):
        raise ValueError("Fresh contact, two-yoke capture or wheel handoff unavailable")
    started = (
        {"pipe": "coral-A"},
        {"wheel": before, "capturedPipe": ready["state"]},
        w,
        cargo,
    )
    plan = dict(
        wheel=w,
        pipe=cargo,
        nextWaypoint=0,
        holdS=0.0,
        reefSetbackM=0.054,
        startWallS=ready["endWallS"],
        maxSpeedMps=0.48,
        maxAccelerationMps2=0.28,
        maxYawAccelerationRadS2=0.38,
    )
    out = raw / "route"
    result = F.run(None, out, side, 31.25, continuation=plan, registered_start=started)
    result.update(
        station=service["incoming"],
        candidateCam=service["candidateStepSha256"],
        sourceMatchRegister=sha(raw / "station-wheels.json"),
        initialCustody=sha(raw / "station-piece.json"),
        stationObservation=sha(artifacts / "cam-scoring/optics.json"),
    )
    write(out / "route.json", result)
    return result


if __name__ == "__main__":
    p = ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("side", choices=["plus", "minus"])
    p.add_argument("stage", choices=["station", "route"])
    args = p.parse_args()
    r = run(args.artifacts, args.side, args.stage)
    print(r.get("endWallS"), r.get("field", {}).get("minimumPeerMm"))
