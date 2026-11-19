"""Condense reproducible solver receipts; retain each signed input and time-series digest."""

import argparse
import json
import re
from pathlib import Path


def collect(directory):
    suites = {}
    for source in sorted(directory.glob("*.json")):
        if not re.fullmatch(
            r"(?:plus|minus)-(?:issued|no-buttons|free)-[0-9.]+us(?:-3s-400000(?:-kick-0\.25)?)?",
            source.stem,
        ):
            continue
        row = json.loads(source.read_bytes())
        if "-issued-" in source.name or "-no-buttons-" in source.name:
            keys = (
                "dtS",
                "activeButtons",
                "stationStateSha256",
                "stationReceiptSha256",
                "traceSha256",
                "stationCenterM",
                "stationAxis",
                "stationToDockMotion",
                "firstPossibleEdgeState",
                "firstNegativeMeanNormalS",
                "minRequiredMeanNormalMps2",
                "possibleSharpEdgeInterferenceMm",
                "finalProbeMm",
                "lastStrokeMm",
            )
            suites[source.stem] = {key: row[key] for key in keys}
        elif "-free-" in source.name and not source.stem.endswith("-cam"):
            omitted = {"capMode", "peakCapForceN", "largestCapCompressionMm"}
            suites[source.stem] = {
                **{k: v for k, v in row.items() if k not in omitted},
                "verticalKickMps": row.get("verticalKickMps", 0.0),
            }
    return suites


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(collect(args.directory), indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
