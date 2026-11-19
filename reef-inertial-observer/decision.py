"""Report desktop interlocks without granting physical scoring authority."""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def load(name):
    return json.loads((HERE / name).read_bytes())


def evaluate():
    result = []
    traces = {(r["side"], r["mode"]): r for r in load("clearance.json")}
    custody = json.loads((ROOT / "mounted-guide-vision/handoff.json").read_bytes())[
        "nominal"
    ]
    for trial in load("results.json"):
        side, mode = trial["side"], trial["mode"]
        route = traces[side, mode]
        if route["motionSha256"] != trial["motionSHA256"]:
            raise ValueError("Unmatched controller path")
        station = [
            r
            for r in custody
            if r["wheelCase"] == side
            and r["guard"] == "guarded"
            and r["source"].endswith("serial-carriage-recover")
        ]
        if len(station) != 2:
            raise ValueError("Missing signed station time refinement")
        stop = min(
            route[k] for k in ("installedBallMm", "otherStockMm", "barrierMm", "peerMm")
        )
        conditions = {
            "stationGuardBothRefinements": all(r["accepted"] for r in station),
            "fieldControllerAccepted": trial["accepted"],
            "fixtureEnclosesTrueFieldPose": not trial["truthOutsideDeclaredCalibration"]
            if trial["accepted"]
            else None,
            "registeredSweepClear": stop > 0,
            "angularAccelerationInsideScreen": route["maximumAngularAccelerationRadS2"]
            <= 1.5,
        }
        result.append(
            {
                "wheelCase": side,
                "fieldInput": mode,
                **conditions,
                "wallDecisionS": (
                    route["wallStartS"] + trial["decisionMs"] / 1000
                    if trial["accepted"]
                    else None
                ),
                "wallStoppedS": route["wallStopS"],
                "minimumFieldClearanceMm": stop,
                "independentAsInstalledFieldAndPupilSurvey": None,
                "carriedPipeCustodyOnThisWheelHistory": None,
                "scoringPermission": False,
                "supportedPipePoints": 0,
            }
        )
    return result


if __name__ == "__main__":
    (HERE / "decisions.json").write_text(
        json.dumps(evaluate(), indent=2) + "\n", encoding="utf8", newline="\n"
    )
