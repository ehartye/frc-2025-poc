"""Apply the released packet and actuator policy to the newly rendered image intervals."""

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
MATCH = ROOT / "relieved-match"
sys.path.insert(0, str(MATCH))
import control


def read(path):
    return json.loads(path.read_bytes())


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")


def rows(path):
    with path.open() as f:
        return list(csv.reader(f))[1:]


def signature(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def run(args, output=HERE):
    inp = args.images
    scene = read(inp / "scenes.json")
    hashes = scene["backgroundSha256"]
    field = read(output / "roof-field.json")
    if field["fieldSha256"] != scene["sourceSha256"]["registered field STEP"]:
        raise ValueError("Roof and fixed observers used different fields")
    with tempfile.TemporaryDirectory() as temporary:
        tmp = Path(temporary)
        nominal = tmp / "receipts"
        nominal.mkdir()
        raw = tmp / "raw"
        raw.mkdir()
        stations = {}
        inputs = {}
        for key, label in (("coral-A", "first"), ("coral-B", "return")):
            for head in ("fixed", "moving", "seat"):
                mode = "critical" if head == "fixed" else "all"
                stem = f"{key}-{head}-barrel-nominal-{mode}" + (
                    "-bounded" if head == "moving" else ""
                )
                path = inp / (stem + ".csv")
                receipt = read(inp / (stem + ".json"))
                if control.sha(path) != receipt["frameCsvSha256"]:
                    raise ValueError("Stale image observations")
                if head == "moving":
                    samples = {r[2]: [r[3], *map(float, r[4:])] for r in rows(path)}
                    for pose in field["cases"][key].values():
                        for tick, observation in pose["frames"].items():
                            observed = [observation["state"], *observation["boundsMm"]]
                            if samples.get(tick) != observed:
                                raise ValueError(
                                    "Field altered a roof observation at " + tick
                                )
                if head == "seat":
                    stations[label + "-nominal"] = {
                        "frames": [
                            [int(r[2]), r[3], *map(float, r[4:])] for r in rows(path)
                        ]
                    }
                else:
                    shutil.copyfile(path, raw / (key + "-" + head + ".csv"))
                inputs[stem] = control.sha(path)
            for res in (250, 500):
                name = f"{key}-pads-edge-{res}-forces.csv"
                shutil.copyfile(args.cache / name, raw / name)
            for name in (
                label + "-feed.json",
                key + "-moving.json",
                key + "-pads-edge-250.json",
                key + "-pads-edge-500.json",
            ):
                shutil.copyfile(MATCH / name, nominal / name)
        for name in ("landing-windows.csv", "probe-native.json", "hold-native.json"):
            shutil.copyfile(MATCH / name, nominal / name)
        write(
            nominal / "station-images.json", {"cases": stations, "sourceHashes": inputs}
        )
        write(
            nominal / "camera-registration.json",
            {
                "fixedPlaceSha256": signature(
                    [hashes[k + "-fixed"] for k in ("coral-A", "coral-B")]
                ),
                "coMovingBackingSha256": signature(
                    [
                        *[
                            hashes["barrel-moving-" + s]
                            for s in ("reference", "0", "370")
                        ],
                        read(inp / "sweep.json")["nearestPossibleSha256"],
                    ]
                ),
            },
        )
        args.out.mkdir(exist_ok=True, parents=True)
        gen = importlib.util.spec_from_file_location(
            "selected_optics_recovery", ROOT / "packet-recovery/replay.py"
        )
        recovery = importlib.util.module_from_spec(gen)
        gen.loader.exec_module(recovery)
        with patch.object(control, "HERE", nominal):
            control.run(
                args.out,
                args.jdk,
                args.gradle,
                output=output,
                source_cache=raw,
                configure=recovery.configure,
            )
        for folder in (nominal, raw):
            target = args.out / "registered-inputs" / folder.name
            target.mkdir(parents=True, exist_ok=True)
            for source in folder.iterdir():
                shutil.copyfile(source, target / source.name)
    with (output / "events.csv").open() as f:
        actual = list(csv.DictReader(f))
    with (ROOT / "packet-recovery/events.csv").open() as f:
        prior = list(csv.DictReader(f))
    cases = {}
    for mode in recovery.MODES:
        new = [r for r in actual if r["scenario"] == mode]
        old = [r for r in prior if r["scenario"] == mode]
        b = [r for r in new if r["piece"] == "coral-B"]
        cases[mode] = {
            "sameEventsAsReference": new == old,
            "finalB": b[-1] if b else None,
            "withdrawWallMs": next(
                (int(r["wall_ms"]) for r in b if r["event"] == "withdraw"), None
            ),
        }
    write(
        output / "controller.json",
        {
            "registeredImages": inputs,
            "roofFieldCheckSha256": control.sha(output / "roof-field.json"),
            "sameEventsAsReference": actual == prior,
            "referenceEventsSha256": control.sha(ROOT / "packet-recovery/events.csv"),
            "selectedEventsSha256": control.sha(output / "events.csv"),
            "cases": cases,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("images", "cache", "out", "jdk", "gradle"):
        p.add_argument(name, type=Path)
    run(p.parse_args())
