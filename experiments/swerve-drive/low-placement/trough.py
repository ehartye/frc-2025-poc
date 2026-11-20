"""Launch the finite-field solver only from an actual released tube state."""

import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SIX = ROOT / "experiments/six-wheel"
sys.path.insert(0, str(SIX / "spatial-contact"))
from physics import panel_file


def read(p):
    return json.loads(p.read_bytes())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def forward_limit(stroke):
    native = read(
        ROOT / "cad/assembly-engineering/receiver-service/issue/engineering-reopen.json"
    )
    groups = {
        "Outer slide": 1,
        "Middle slide": 0.5,
        "Left capture yoke": 1,
        "Right capture yoke": 1,
    }
    limits = []
    for row in native["bodies"]:
        if row["instance"] is None:
            continue
        ratio = groups.get(row["occurrence"].split(":")[0], 0)
        x0, y0, z0, x1, y1, z1 = row["boundsMm"]
        tilt = 0.04 if ratio else 0
        bound = (
            x1
            + (stroke + 2) * ratio
            + np.sin(tilt) * max(abs(z0 - 467), abs(z1 - 467))
            + (1 - np.cos(tilt)) * max(abs(x0), abs(x1))
        )
        limits.append(bound)
    return max(limits) / 1000


def run(runfile, jdk):
    record = read(runfile)
    if not record["event"]:
        raise ValueError("No observed free tube at the scoring lip")
    out = runfile.parent / (runfile.stem + "-field")
    out.mkdir(exist_ok=True)
    e = record["event"]
    target = SIX / "mission/target.json"
    solver = SIX / "spatial-contact/SpatialContact.java"
    prior = read(runfile.parent / "chassis.json")
    cargo = read(runfile.parent / "coral.json")
    if (
        cargo["state"] != record["sourceState"]
        or not prior["stopped"]["confirmed"]
        or abs(cargo["endWallS"] - record["startWallS"]) > 1e-7
    ):
        raise ValueError("Different stopped chassis or cargo")
    panel_file(read(target), out / "panels.csv")
    # The existing solver subtracts withdrawal from its original X535 mm nose.
    # Supply a constant offset for this held 337 mm rack's entire installed
    # forward hull. No assumed post-ejection retraction or timing advantage.
    front = forward_limit(record["commandMm"])
    with (out / "withdraw.csv").open("w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["time_s", "withdraw_m"])
        for t in (0.0, 0.001):
            writer.writerow([t, 0.535 - front])
    cases = []
    interval = prior["enclosure"]
    poses = [("attained", prior["terminalMmMmRad"])] + [
        (f"bound-{i}", v) for i, v in enumerate(itertools.product(*zip(*interval)))
    ]
    for label, (x, y, yaw) in poses:
        co, si = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[co, -si, 0], [si, co, 0], [0, 0, 1]])
        chassis = np.array([x / 1000 - 0.490, y / 1000, 0.0])
        pos = chassis + rotation @ np.asarray(e["chassisP"])
        velocity = rotation @ np.asarray(e["velocity"])
        axis = rotation @ np.asarray(e["axis"])
        omega = rotation @ np.asarray(e["omega"])
        for name, dt, stations, mu in (
            ("nominal", 0.0002, 9, 0.35),
            ("refined", 0.0001, 17, 0.35),
            ("low-friction", 0.0001, 17, 0.10),
        ):
            cases.append(
                [
                    label + "/" + name,
                    *pos,
                    *velocity,
                    *axis,
                    *omega,
                    *chassis[:2],
                    yaw,
                    -0.001,
                    3.0,
                    dt,
                    stations,
                    mu,
                ]
            )
    with (out / "cases.csv").open("w", newline="") as f:
        csv.writer(f, lineterminator="\n").writerows(cases)
    subprocess.run([str(jdk / "bin/javac"), "-d", str(out), str(solver)], check=True)
    subprocess.run(
        [
            str(jdk / "bin/java"),
            "-cp",
            str(out),
            "SpatialContact",
            *[
                str(out / n)
                for n in (
                    "panels.csv",
                    "withdraw.csv",
                    "cases.csv",
                    "results.csv",
                    "trajectory.csv",
                )
            ],
            "sweep-y",
        ],
        check=True,
    )
    with (out / "results.csv").open() as f:
        rows = [
            {k: v if k in ("case", "outcome") else float(v) for k, v in row.items()}
            for row in csv.DictReader(f)
        ]
    result = dict(
        piece=record["piece"],
        sourceState=record["sourceState"],
        releaseReceipt=sha(runfile),
        releaseTrace=record["trace"],
        releaseWallS=record["endWallS"],
        target=sha(target),
        solver=sha(solver),
        fieldInputs=sha(out / "cases.csv"),
        heldWholeRobotForemostM=front,
        withdrawal=sha(out / "withdraw.csv"),
        results=rows,
        assessedAtS=record["endWallS"] + 3.0,
        observedScorePoints=0,
        scope="One field integration for each specified state. Corner controls sample placement uncertainty. A physical release, independent occupancy, safe recovery, fresh next pipe and repeatability must all be assessed on the same registered mission.",
    )
    (out / "field.json").write_bytes(
        (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
    )
    print([(r["case"], r["outcome"]) for r in rows[:3]], flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("release", type=Path)
    p.add_argument("jdk", type=Path)
    a = p.parse_args()
    run(a.release, a.jdk)
