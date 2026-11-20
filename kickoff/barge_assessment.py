"""Translate the lab's active seconds to the release-day match clock and tally the BARGE RP."""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def summarize():
    raw = json.loads((HERE / "match-series.json").read_bytes())
    rules = json.loads((HERE / "rules-v0.json").read_bytes())["scoring"]["barge"]
    out = []
    for game in raw["matches"]:
        ledger = game["game"]["endgame"]
        points = ledger["terminalScore"]
        wall_cutoff = ledger["activeCutoffS"] + rules["transitionS"]
        out.append(
            dict(
                match=game["id"],
                ordinary=ledger["ordinaryScore"],
                barge=points,
                total=game["game"]["score"],
                bargeQualificationRP={
                    k: int(v >= rules["bargeRankingThreshold"])
                    for k, v in points.items()
                },
                poweredCutoffWallS=wall_cutoff,
                assessmentWallS=ledger["assessmentTimeS"] + rules["transitionS"],
                attempts=[
                    {
                        k: r[k]
                        for k in (
                            "robot",
                            "action",
                            "target",
                            "startS",
                            "arrivedAtS",
                            "completedAtS",
                            "outcome",
                            "points",
                        )
                    }
                    for r in ledger["attempts"]
                ],
            )
        )
    return dict(
        profileSha256=raw["profileSha256"],
        transitionS=rules["transitionS"],
        threshold=rules["bargeRankingThreshold"],
        matches=out,
        scope="Prescribed qualification peers and final witnesses, no foul adjustments. Parked robots remain at rest from TELEOP cutoff through the recorded assessment. The project robot has no awarded climb.",
    )


if __name__ == "__main__":
    (HERE / "barge-assessment.json").write_bytes(
        (json.dumps(summarize(), indent=2) + "\n").encode()
    )
