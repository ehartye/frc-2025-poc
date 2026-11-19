"""Freeze the project replay and five-peer/field context from the same sampled wall clock."""

import argparse
import json
import math
from pathlib import Path
import numpy as np
from opening import HERE, ROOT, read, sha, n

CASES = ("plus", "minus", "blind", "late", "jam", "misaligned", "heavy", "upper-minus")


def publish(directory, out):
    receipts = {}
    traces = {}
    for case in CASES:
        item = read(directory / (case + ".json"))
        if sha(directory / (case + ".csv")) != item["csvSha256"]:
            raise ValueError("Clock fingerprint changed for " + case)
        traces[case] = item.pop("samples")
        receipts[case] = item
    f, _, team, _ = n.route.route.field.scenario(
        read(ROOT / "fixed-height-match/placements.json")["placements"], inboard=True
    )
    t = [row[0] for row in traces["plus"]]
    peers = {
        key: [np.round(program.at(max(0, wall - 18))[0], 4).tolist() for wall in t]
        for key, program in team.items()
        if key != "red2"
    }
    data = read(ROOT / "full-match/installed-stock.json")["stock"]
    stock = []
    for key, obj in data.items():
        x, _, z = obj["native_center_mm"]
        stock.append(
            [key, obj["group"], *np.round(f.field_from_native(x / 25.4, z / 25.4), 4)]
        )
    field = dict(
        size=[f.PROFILE["field"][name] * f.IN for name in ("length", "width")],
        obstacles=list(f.obstacles()),
        planes=[
            dict(point=[x * f.IN for x in item["at"]], angle=item["inwardDeg"])
            for item in f.PROFILE["rehearsal"]["planes"]
        ],
        tape=(690.875 - 144 - 65.5 - 88) * f.IN,
        stock=stock,
        station=f.DOCK,
        peers=peers,
    )
    out.mkdir(parents=True, exist_ok=True)
    receipt = dict(
        schema="reefscape-swerve-opening/1",
        scope="Prebuild station approach only. Staged pieces remain in field inventory. No pickup, transfer, delivery or scoring result.",
        samplesPerCase=4220,
        sources={
            s: sha(ROOT / s)
            for s in [
                "station-landmark/native.json",
                "station-landmark/panel.py",
                "station-landmark/field.py",
                "full-match/installed-stock.json",
                "fixed-height-match/placements.json",
                "kickoff/rule-register-v0.json",
                "mounted-guide-vision/setting-station.json",
            ]
        },
        cases=receipts,
    )
    (out / "opening.json").write_text(
        json.dumps(receipt, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
    render = {
        name: {
            k: r[k]
            for k in (
                "events",
                "actualTerminalMmMmDeg",
                "stationLockWallS",
                "peaks",
                "case",
            )
        }
        for name, r in receipts.items()
    }
    for key in render:
        render[key]["samples"] = traces[key]
    (out / "timeline.js").write_text(
        "window.SWERVE_REPLAY="
        + json.dumps(
            dict(field=field, runs=render), separators=(",", ":"), allow_nan=False
        )
        + ";\n",
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("--out", type=Path, default=HERE)
    args = p.parse_args()
    publish(args.raw, args.out)
