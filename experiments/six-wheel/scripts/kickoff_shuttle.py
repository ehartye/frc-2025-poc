"""Replay two one-piece shuttles from the recorded 2025 kickoff rules model."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(name):
    return json.loads((ROOT / "kickoff" / name).read_text(encoding="utf-8"))


def evaluate(model, session, trial):
    phase = session["phase"]
    target = next(t for t in model["scoring"]["targets"] if t["name"] == trial["target"])
    phase_index = next(i for i, p in enumerate(model["scoring"]["phases"]) if p["name"] == phase)
    route = next(r for r in model["routeHypotheses"] if r["phase"] == phase
                 and r["piece"] == target["piece"] and r["destination"] == target["goal"])
    distance = sum(math.dist(p, q) for p, q in zip(route["routeIn"], route["routeIn"][1:]))
    if not math.isclose(distance, route["lengthIn"]):
        raise ValueError("Stored route length disagrees with the displayed waypoints")
    transit = distance / session["assumedTransitInPerSecond"]
    service = trial["pickupSeconds"] + trial["releaseSeconds"]
    count = trial["allocatedAtSource"]
    setup = session["startingSetupSeconds"]
    deadline = session["windowSeconds"]
    # The robot starts at A. The first delivery has no empty return; every subsequent one does.
    finishes = [setup + (2*n-1)*transit + n*service for n in range(1, count+1)]
    credited = sum(t <= deadline for t in finishes)
    goal = trial["goal"]
    travel_budget = deadline - setup - goal*service
    return {
        "id": trial["id"], "target": target["name"], "routeLengthIn": distance,
        "finishTimesSeconds": finishes, "piecesByDeadline": credited,
        "pointsIfEachScores": credited*target["points"][phase_index],
        "goal": goal, "goalFits": goal <= credited,
        "minimumMeanSpeedForGoalInPerSecond": (2*goal-1)*distance/travel_budget
            if goal <= count and travel_budget > 0 else None,
        "maxServiceSecondsPerPieceForGoal": (deadline-setup-(2*goal-1)*transit)/goal,
        "inferredTraffic": route["traffic"]
    }


def main():
    model = read("rules-v0.json")
    session = read("trials.json")
    if session["edition"] != model["edition"]:
        raise ValueError("Trial and field model editions differ")
    result = [evaluate(model, session, t) for t in session["trials"]]
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
