"""Summarize the prescribed placement image/packet controls independently of achieved travel."""

import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(folder):
    camera = read(folder / "controller.json")
    field = read(folder / "roof-field.json")
    reports = {}
    primary = folder / "marked"
    for source in sorted(primary.glob("coral-*.json")):
        rec = read(source)
        reports[source.stem] = {
            "sha256": sha(source),
            "frames": rec["frames"],
            "statuses": rec["statuses"],
            "sourceTrajectorySha256": rec["sourceTrajectorySha256"],
            "csv": rec["frameCsvSha256"],
        }
    moving = read(primary / "coral-B-moving-barrel-nominal-all-bounded.json")[
        "selectedFrames"
    ]
    fixed = read(primary / "coral-B-fixed-barrel-nominal-critical.json")[
        "selectedFrames"
    ]
    times = [26180, 26197, 26214, 26231, 26248, 26401]
    return {
        "sourceRegister": sha(folder / "controller-register.json"),
        "field": sha(folder / "roof-field.json"),
        "programAndInputs": sha(folder / "hal.json"),
        "scenarioCount": len(camera["cases"]),
        "sameEventsAsReference": camera["sameEventsAsReference"],
        "packetEventSha256": camera["selectedEventsSha256"],
        "historicalNominal": camera["cases"]["nominal"],
        "fieldCaptures": {
            case: {site: len(trial["frames"]) for site, trial in locations.items()}
            for case, locations in field["cases"].items()
        },
        "BDesktopFrames": {
            str(t): {
                "moving": moving[str(t)]["bounds"],
                "fixed": fixed[str(t)]["bounds"],
            }
            for t in times
        },
        "replay": reports,
        "actualScoredPipe": None,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("out", type=Path)
    a = p.parse_args()
    a.out.write_text(
        json.dumps(run(a.folder), indent=2) + "\n", encoding="utf8", newline="\n"
    )
