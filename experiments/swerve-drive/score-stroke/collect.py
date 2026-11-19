"""Condense reproducible solver receipts; retain each signed input and time-series digest."""
import argparse
import json
from pathlib import Path


def collect(directory):
    suites = {}
    for source in sorted(directory.glob("*.json")):
        row = json.loads(source.read_bytes())
        if "-issued-" in source.name or "-no-buttons-" in source.name:
            keys = ("dtS","activeButtons","stationStateSha256","stationReceiptSha256","traceSha256",
                    "stationCenterM","stationAxis","stationToDockMotion","firstPossibleEdgeState",
                    "firstNegativeMeanNormalS","minRequiredMeanNormalMps2","possibleSharpEdgeInterferenceMm",
                    "finalProbeMm","lastStrokeMm")
            suites[source.stem] = {key:row[key] for key in keys}
        elif "-free-" in source.name:
            suites[source.stem] = {**row,"verticalKickMps":row.get("verticalKickMps",0.)}
    return suites


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(collect(args.directory),indent=2,allow_nan=False)+"\n",encoding="utf8",newline="\n")
