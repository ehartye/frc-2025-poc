"""Generate a self-contained student comparison from the two measured wheel histories."""

import argparse
import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def build(assets):
    data = {}
    decisions = {
        (r["wheelCase"], r["fieldInput"]): r
        for r in json.loads((HERE / "decisions.json").read_bytes())
    }
    for row in json.loads((HERE / "results.json").read_bytes()):
        side, mode = row["side"], row["mode"]
        case = decisions[side, mode]
        observations = {r[0]: r[1:] for r in row["observations"]}
        source = (
            assets
            / "reef-inertial-observer/sampled"
            / side
            / ("coral-A-" + mode + ".csv")
        )
        frames = []
        with source.open(encoding="utf8") as f:
            for point in csv.DictReader(f):
                stamp = int(point["ms"])
                if stamp % 35:
                    continue
                frames.append(
                    [
                        stamp,
                        round(float(point["x_mm"]), 1),
                        round(float(point["y_mm"]) + 180, 1),
                        *(observations.get(stamp, [0, None])),
                    ]
                )
        data[side, mode] = {
            "frames": frames,
            "accepted": case["fieldControllerAccepted"],
            "correct": case["fixtureEnclosesTrueFieldPose"],
            "decision": row["decisionMs"],
            "bound": row["acceptanceDiskMmYawDeg"],
            "reason": row["reason"],
        }
    page = (HERE / "viewer.template.html").read_text(encoding="utf8")
    payload = {
        side: {mode: val for (s, mode), val in data.items() if s == side}
        for side in ("leftSmall", "rightSmall")
    }
    (HERE / "viewer.html").write_text(
        page.replace("DATA_PAYLOAD", json.dumps(payload, separators=(",", ":"))),
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", type=Path)
    build(parser.parse_args().artifacts)
