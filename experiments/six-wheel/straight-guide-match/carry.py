"""Replay actual station departures and loaded shelf feedback against the selected field clock."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from plant import HERE, ROOT, Selection, read

sys.path.insert(0, str(ROOT / "observed-reef-a"))
import retention


def run(assets, side, arrival_us, step_us, mode, field, stuck, attained=False):
    source = next(
        r
        for r in read(ROOT / "mounted-guide-vision/handoff.json")["nominal"]
        if r["wheelCase"] == side
        and r["guard"] == mode.removesuffix("-all-edges")
        and f"-{arrival_us:g}us-" in r["source"]
        and r["source"].endswith("serial-carriage-recover")
    )
    if mode.endswith("-all-edges"):
        source = next(
            r
            for r in read(HERE / "stations.json")["gate"]
            if r["wheelCase"] == side
            and r["guard"] == mode
            and f"-{arrival_us:g}us-" in r["source"]
            and r["source"].endswith("serial-carriage-recover")
        )
    if attained:
        station = read(HERE / f"station-{side}-{arrival_us:g}.json")
        if mode != "guarded-all-edges":
            raise ValueError("Attained station uses all visible edges")
        source = {
            "accepted": station["guardedStationHandoff"],
            "pipeTrajectorySha256": station["pipeSha256"],
        }
    if not source["accepted"]:
        return {
            "piece": "coral-A",
            "wheelCase": side,
            "arrivalUs": arrival_us,
            "stepUs": step_us,
            "head": mode,
            "policy": "hold at station",
            "sourcePipeSha256": source["pipeTrajectorySha256"],
            "scoringPermission": False,
            "wheelDeparture": False,
        }
    selection = Selection(arrival_us, attained)
    with subprocess.Popen(
        [
            sys.executable,
            str(HERE / "camera.py"),
            str(assets),
            mode,
            *(["--attained"] if attained else []),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    ) as camera:
        try:
            row = retention.propagate(
                assets,
                side,
                step_us * 1e-6,
                field,
                True,
                camera,
                True,
                mode,
                True,
                stuck,
                True,
                selection=selection,
            )
        finally:
            camera.stdin.close()
    row.update(
        arrivalUs=arrival_us,
        stepUs=step_us,
        head=mode,
        wheelDeparture=True,
        checkpoint=selection.restore_state,
        peakStageMm=selection.stage_peak_mm,
        firstStageAboveGuardS=selection.pupil_guard_exceeded,
    )
    return row


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("assets", type=Path)
    parser.add_argument(
        "--side", choices=["leftSmall", "rightSmall"], default="leftSmall"
    )
    parser.add_argument("--arrival-us", type=float, required=True)
    parser.add_argument("--step-us", type=float, required=True)
    parser.add_argument(
        "--mode",
        default="guarded",
        choices=["guarded", "nominal", "matched-finish", "guarded-all-edges"],
    )
    parser.add_argument("--field", default="nominal")
    parser.add_argument("--attained", action="store_true")
    parser.add_argument("--stuck", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    a = parser.parse_args()
    result = run(
        a.assets, a.side, a.arrival_us, a.step_us, a.mode, a.field, a.stuck, a.attained
    )
    a.output.write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    print(
        result.get("firstFault", result.get("policy")),
        result.get("lastObservedJavaMs"),
        result.get("finalWallS"),
        flush=True,
    )
