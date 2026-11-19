"""Collect the two signed acquisition, contact, loaded-cycle and dock comparisons."""

import argparse
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_bytes())


def collect(raw):
    result = {k: {} for k in ("feeds", "service", "cycles", "freePipe", "dock")}
    for sign in ("plus", "minus"):
        for step, gap in ((31.25, 0.1), (31.25, 0.2), (31.25, 2.6), (15.625, 0.1)):
            stem = f"cam-station-{sign}-{step:g}us-{gap:g}mm"
            feed = read(raw / (stem + ".json"))
            keys = (
                "case",
                "physicalPieceId",
                "plantStepUs",
                "capGapMm",
                "candidateStepSha256",
                "driveTraceSha256",
                "trueChassisAtFeedMmMmDeg",
                "programSha256",
                "registeredPlateTable",
                "pipeStateSha256",
                "pipeTrajectorySha256",
                "javaFirstBandMs",
                "javaLastMs",
                "state",
                "loss",
                "hardStop",
                "reactionPeakN",
            )
            result["feeds"][stem] = {key: feed[key] for key in keys}
            if gap == 0.1:
                result["service"][stem] = read(raw / (stem + "-service.json"))
        for step in (31.25, 62.5):
            name = f"{sign}-cam-cycle-{step:g}us"
            cycle = read(raw / (name + ".json"))
            keys = (
                "stationStateSha256",
                "camStepSha256",
                "issuedFixture",
                "traceSha256",
                "fullStrokeMm",
                "lastStrokeMm",
                "lastTimeS",
                "peakRailN",
                "peakButtonN",
                "peakButtonStrokeMm",
                "firstPossibleEdgeState",
                "possibleSharpEdgeInterferenceMm",
                "maxButtonForceBeyondRunnerN",
                "maxReturnButtonReactionN",
                "samePipeHeldForBothDirections",
                "stationToDockMotion",
            )
            result["cycles"][name] = {k: cycle[k] for k in keys}
            free = f"{sign}-free-{step:g}us-3s-400000-cam"
            result["freePipe"][free] = read(raw / (free + ".json"))
        result["dock"][sign] = read(raw / f"{sign}-cam-cycle-31.25us-dock.json")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    args.output.write_text(
        json.dumps(collect(args.raw), indent=2, allow_nan=False) + "\n", encoding="utf8"
    )
