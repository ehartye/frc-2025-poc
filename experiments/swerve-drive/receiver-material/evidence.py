"""Join material-specific port, true-pipe and independently captured four-eye results."""

import argparse
import json
from pathlib import Path

import numpy as np
from observations import trial, witness


def reduce_images(out):
    return {
        mode: {
            "start": r["gate"]["accepted"],
            "start45ms": r["late"]["accepted"],
            "eligibleJavaMs": r["gate"]["eligibleCapturesJavaMs"],
            "images": [f.get("imageSha256") for f in r["frames"]],
            "outcomes": [
                [f.get("fixed"), f.get("roof"), f.get("why")] for f in r["frames"]
            ],
            "captureAccepted": [f["accepted"] for f in r["frames"]],
            "maxHeldoutError": max(
                (
                    c["maxValidationError"]
                    for f in r["frames"]
                    for c in f.get("calibration", [])
                    if "maxValidationError" in c
                ),
                default=None,
            ),
        }
        for mode, r in out.items()
    }


def process(root, geometry, mask, cal, case, all_controls=False, rims=False):
    p = case / "result.json"
    setup = witness.read(p)
    feed = setup["feed"]
    if feed["state"] != "IN_BAND" or feed["javaLastMs"] != 5460:
        observations = {}
        pipe = []
    else:
        pipe = [
            dict(javaMs=stamp, centerMm=q.tolist(), axis=v.tolist())
            for stamp, q, v in witness.frames(feed, case)
        ]
        detailed = trial(feed, case, geometry, mask, cal, full=all_controls, rims=rims)
        suffix = ("rims" if rims else "peak") + ("-all" if all_controls else "-core")
        (case / f"photometry-{suffix}.json").write_text(
            json.dumps(detailed, indent=2, allow_nan=False) + "\n",
            encoding="utf8",
            newline="\n",
        )
        observations = reduce_images(detailed)
    return {
        "case": feed["case"],
        "densityKgM3": setup["densityKgM3"],
        "shelfFriction": setup["shelfFriction"],
        "modulusPa": setup["modulusPa"],
        "stepUs": feed["plantStepUs"],
        "javaState": feed["state"],
        "javaFirstBandMs": feed["javaFirstBandMs"],
        "javaLastMs": feed["javaLastMs"],
        "dockMmMmDeg": feed["trueChassisAtFeedMmMmDeg"],
        "pipeTrajectorySha256": feed["pipeTrajectorySha256"],
        "terminalStateSha256": feed["pipeStateSha256"],
        "dockToPipeProgramSha256": feed["programSha256"],
        "physicalPipeAtCaptures": pipe,
        "observations": observations,
    }


def run(root, geometry, wheel, *, full=False, rims=False, match="rho-*"):
    calibrators = witness.read(root / "calibrators.json")
    masks = root / "calibrators.npz"
    if (
        witness.digest(masks) != calibrators["maskSha256"]
        or witness.digest(wheel) != calibrators["wheelMaskSha256"]
    ):
        raise ValueError("The two guard registrations do not match")
    with np.load(masks) as cal:
        for feed in sorted(root.glob(match + "/result.json")):
            case = feed.parent
            suffix = ("rims-" if rims else "") + ("all" if full else "core")
            dst = case / f"qualified-{suffix}.json"
            # Per-case files can be generated as each solver finishes. The final assembly
            # checks their input pins and exact coverage; changing a solver requires a new output root.
            if not dst.exists():
                result = process(root, geometry, wheel, cal, case, full, rims)
                dst.write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n",
                    encoding="utf8",
                    newline="\n",
                )
                print(
                    case.name,
                    result["javaState"],
                    [(m, r["start"]) for m, r in result["observations"].items()],
                    flush=True,
                )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("root", "geometry", "wheel"):
        p.add_argument(k, type=Path)
    p.add_argument("--full", action="store_true")
    p.add_argument("--rims", action="store_true")
    p.add_argument("--match", default="rho-*")
    a = p.parse_args()
    run(a.root, a.geometry, a.wheel, full=a.full, rims=a.rims, match=a.match)
