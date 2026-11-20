"""Verify the ordered source and clocks and summarize both completed development carries."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def load(p):
    return json.loads(p.read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def check(condition, reason):
    if not condition:
        raise ValueError(reason)


def collect(artifacts):
    all_cases = {}
    root = artifacts / "cam-scoring"
    camera = load(root / "camera.json")
    field = load(root / "reef-optics.json")
    challenge = load(root / "holdout.json")
    check(
        challenge["scene"]
        == field["fieldScene"]
        == camera["bytes"]
        == sha(root / "camera.npz"),
        "Wrong camera raster",
    )
    for side in ("plus", "minus"):
        folder = root / side
        optics = load(root / "optics.json")[side]
        wheels = load(folder / "station-wheels.json")
        route = load(folder / "route/route.json")
        source = load(
            artifacts
            / "r4-score-stroke"
            / f"cam-station-{side}-31.25us-0.1mm-service.json"
        )
        stages = {}
        for stage in ("station", "route"):
            piece = load(folder / (stage + "-piece.json"))
            hist = folder / (stage + "-piece.npy")
            state = folder / (stage + "-piece.npz")
            check(
                piece["trace"] == sha(hist) and piece["state"] == sha(state),
                "Changed pipe history or checkpoint",
            )
            check(
                piece["piece"] == source["piece"] == route["piece"] == "coral-A",
                "Replaced cargo",
            )
            check(
                piece["cam"]
                == source["candidateStepSha256"]
                == route["candidateCam"]
                == optics["cam"],
                "Different shelf",
            )
            with open(hist, "rb") as f:
                tick = np.load(f)
            good = tick[:, -1].astype(bool)
            confirmed = next(
                (
                    float(tick[i, 0])
                    for i in range(4, len(good))
                    if all(good[i - 4 : i + 1])
                ),
                None,
            )
            check(
                piece["services"] == len(tick)
                and abs(piece["endWallS"] - piece["startWallS"] - tick[-1, 0]) < 1e-7,
                "Wrong duration",
            )
            stages[stage] = dict(
                **piece, grantedServices=int(good.sum()), firstFiveConfirmedS=confirmed
            )
        first, last = stages["station"], stages["route"]
        check(
            first["before"] == source["outgoing"] == wheels["serviceState"]
            and wheels["trajectory"] == first["wheel"],
            "Wrong first service",
        )
        check(
            first["state"] == last["before"] == route["sourcePipe"],
            "Missing station-to-carry connection",
        )
        check(
            abs(first["endWallS"] - route["startWallS"]) < 1e-7
            and abs(last["startWallS"] - route["startWallS"]) < 1e-7,
            "Discontinuous departure time",
        )
        check(
            last["wheel"] == route["trajectory"] == sha(folder / "route/path.npz"),
            "Different wheel path",
        )
        completed = bool(
            last["allGranted"]
            and abs(last["endWallS"] - route["endWallS"]) < 1e-7
            and route["stopFeedback"]["confirmed"]
        )
        eye = camera["cases"][side]
        check(eye["source"] == route["trajectory"], "Reef scene and route diverged")
        audits = eye["cornerReview"]
        all_cases[side] = dict(
            stationOptics=optics["counts"],
            emptyAcceptedByFinish={
                k: v["empty"] for k, v in optics["observations"].items()
            },
            incomingBody=source["incoming"],
            firstWheelServiceState=source["outgoing"],
            stages=stages,
            carriedToMeasuredRoughStop=completed,
            route={
                k: route[k]
                for k in (
                    "startWallS",
                    "endWallS",
                    "pathLengthM",
                    "sourceWheel",
                    "terminalWheels",
                    "sourceFieldStock",
                    "trajectory",
                    "stopFeedback",
                    "battery",
                    "field",
                )
            },
            dock=dict(
                trueMmMmRad=eye["pose"],
                observed=eye["optical"],
                accepted=sum(r["accepted"] for r in audits),
                exposures=len(audits),
                largestAcceptedCornerErrorPx=max(
                    r["actualMaxCornerErrorPx"] for r in audits if r["accepted"]
                ),
            ),
            positionTargetMmMmRad=[-54, -180, 0],
            releaseAuthorized=False,
            matchDeliveryPoints=0,
        )
    out = dict(
        configuration="fixed-low-straight-arm-cam-capture",
        developmentCases=all_cases,
        stationContactStepUs=31.25,
        stationaryContactGapMm=0.1,
        firstService="cam-station-{sign}-31.25us-0.1mm-service.json",
        stationaryObservations=sha(root / "optics.json"),
        reefSource=field,
        heldOut={
            k: v
            for k, v in challenge.items()
            if k not in ("observations", "falseGrants")
        },
        falseGrants=len(challenge["falseGrants"]),
        holdoutByTreatment={
            t: dict(
                individualEyes=len(rows),
                accepted=sum(r.get("accepted", False) for r in rows),
            )
            for t in ("dim", "bright", "subpixel", "wrong-tag")
            if (rows := [r for r in challenge["observations"] if r["treatment"] == t])
        },
        matchDeliveryPoints=0,
    )
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    args = p.parse_args()
    data = collect(args.artifacts)
    (HERE / "results.json").write_text(
        json.dumps(data, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    print(
        {
            k: v["carriedToMeasuredRoughStop"]
            for k, v in data["developmentCases"].items()
        }
    )
