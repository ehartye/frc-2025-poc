"""Register the optical calibration, causal correction, retained cargo and distinct cam inputs."""

from pathlib import Path
from argparse import ArgumentParser
import json
import hashlib

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(artifacts):
    folder = artifacts / "reef-correction"
    receipt = {
        "camera": read(folder / "camera.json")["bytes"],
        "sourceSurvey": read(folder / "camera.json")["sourceSurvey"],
        "residualLimitPx": 0.18,
        "perCornerBudgetPx": 0.25,
        "cases": {},
        "matchDeliveryCredit": 0,
    }
    camera = read(folder / "camera.json")
    old = read(HERE.parent / "motion/loaded-feed.json")
    cam = read(HERE.parent / "score-stroke/candidate.json")
    for side in ("plus", "minus"):
        src = folder / side
        chassis = read(src / "chassis.json")
        cargo = read(src / "coral.json")
        native = read(src / "clearance.json")
        if (
            chassis["route"] != cargo["wheels"]
            or abs(cargo["endWallS"] - chassis["endWallS"]) > 1e-7
        ):
            raise ValueError("State or epoch mismatch")
        controls = [
            read(src / (mode + ".json")) for mode in ("lost-packets", "missing-eye")
        ]
        rows = chassis["cornerReview"]
        original = next(
            r
            for r in old["attempts"]
            if r["case"] == side and r["plantStepUs"] == 31.25
        )
        changed = cam["feeds"][f"cam-station-{side}-31.25us-0.1mm"]
        if changed["pipeStateSha256"] == original["pipeStateSha256"]:
            raise ValueError("The changed cam needs its own source branch")
        receipt["cases"][side] = dict(
            acceptedTagFrames=sum(r["accepted"] for r in rows),
            decodedFrames=len(rows),
            worstUsedCornerPx=max(
                r["actualMaxCornerErrorPx"] for r in rows if r["accepted"]
            ),
            worstCulledCornerPx=max(
                (r["actualMaxCornerErrorPx"] for r in rows if not r["accepted"]),
                default=None,
            ),
            exposures=rows,
            correctedInitialBound=camera["cases"][side]["optical"][
                "positionMmHeadingRad"
            ],
            startWallS=chassis["startWallS"],
            endWallS=chassis["endWallS"],
            shaftSource=chassis["sourceWheels"],
            visionTicks=chassis["newFrames"],
            terminalMmMmRad=chassis["terminalMmMmRad"],
            enclosure=chassis["enclosure"],
            route=chassis["route"],
            physicalStop=chassis["stopped"],
            minStowedFieldMm={
                k: chassis["field"][k]
                for k in ("minimumBarrierMm", "minimumStockMm", "minimumPeerMm")
            },
            power=chassis["power"],
            cargo=cargo,
            adverseControls=controls,
            conditionalWallClearance=native,
            stationSources=dict(
                originalState=original["pipeStateSha256"],
                originalTrace=original["pipeTrajectorySha256"],
                camState=changed["pipeStateSha256"],
                camTrace=changed["pipeTrajectorySha256"],
                camGeometry=changed["candidateStepSha256"],
            ),
        )
    return receipt


if __name__ == "__main__":
    p = ArgumentParser(description=__doc__)
    p.add_argument("artifacts", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_text(json.dumps(run(a.artifacts), indent=2) + "\n", encoding="utf8")
