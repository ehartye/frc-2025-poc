"""Propagate the conditional loaded-rim release into the registered finite trough panels."""

import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from native_score import SIX, LIP, SurfaceTrace, digest, read, release_control

sys.path.insert(0, str(SIX / "spatial-contact"))
from physics import panel_file


def run(raw, jdk, output):
    dst = raw / "candidate-landing"
    dst.mkdir(exist_ok=True)
    solver = SIX / "spatial-contact/SpatialContact.java"
    target = SIX / "mission/target.json"
    panel_file(read(target), dst / "panels.csv")
    subprocess.run([str(jdk / "bin/javac"), "-d", str(dst), str(solver)], check=True)
    baseline = read(Path(__file__).with_name("native.json"))
    cases = []
    sources = {}
    for sign in ("plus", "minus"):
        name = f"{sign}-cam-cycle-31.25us"
        source = raw / (name + ".npz")
        trace = SurfaceTrace(source, read(raw / (name + ".json"))["traceSha256"])
        sources[sign] = digest(source)
        clock = trace.time[(trace.time >= 3.3)]
        withdrawal = 0.37 - np.interp(clock, trace.time, trace.stroke)
        np.savetxt(
            dst / "withdraw.csv",
            np.column_stack((clock - clock[0], withdrawal)),
            delimiter=",",
            header="time_s,withdraw_m",
            comments="",
            fmt="%.12g",
        )
        for piece in ("coral-A", "coral-B"):
            prior = read(SIX / "relieved-match" / f"{piece}-coupled-250.json")
            old = baseline["release"][piece]
            bounds = [
                LIP + old["wallXFromLipMm"][0],
                0,
                0,
                LIP + old["wallXFromLipMm"][1],
                old["wallTopMm"],
                0,
            ]
            for sec in (3.001, 3.15, 3.299):
                report = release_control(piece, bounds, trace, sec)
                pos = np.array(report["positionM"]) - [0.054, 0, 0]
                vel = np.array(report["velocityMps"])
                chassis = np.array(prior["chassisAtReefXYm"]) - [0.054, 0]
                for test, dt, stations, mu, wait, scale in (
                    ("nominal", 0.0002, 9, 0.35, 3.3 - sec, 1.0),
                    ("refined", 0.0001, 17, 0.35, 3.3 - sec, 1.0),
                    ("low-friction", 0.0001, 17, 0.10, 3.3 - sec, 1.0),
                    ("held", 0.0001, 17, 0.35, 1000.0, 1.0),
                    ("excess-speed", 0.0001, 17, 0.35, 3.3 - sec, 2.0),
                ):
                    case = f"{sign}/{piece}/{sec:g}/{test}"
                    cases.append(
                        [
                            case,
                            *pos,
                            vel[0] * scale,
                            vel[1],
                            vel[2],
                            *prior["axis"],
                            *prior["angularRadps"],
                            *chassis,
                            prior["chassisHeadingRad"],
                            wait,
                            3.0,
                            dt,
                            stations,
                            mu,
                        ]
                    )
    with (dst / "cases.csv").open("w", newline="") as f:
        csv.writer(f, lineterminator="\n").writerows(cases)
    subprocess.run(
        [
            str(jdk / "bin/java"),
            "-cp",
            str(dst),
            "SpatialContact",
            *[
                str(dst / k)
                for k in (
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
    with (dst / "results.csv").open() as f:
        rows = [
            {k: (v if k in ("case", "outcome") else float(v)) for k, v in row.items()}
            for row in csv.DictReader(f)
        ]
    result = dict(
        addedSetbackMm=54,
        simulatedAfterRimS=3.0,
        troughSourceSha256=digest(target),
        solverSha256=digest(solver),
        cycleSha256=sources,
        inputSha256=digest(dst / "cases.csv"),
        rackWithdrawalSha256=digest(dst / "withdraw.csv"),
        cases=rows,
        matchCredit=None,
    )
    output.write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf8"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("raw", "jdk", "output"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    run(a.raw, a.jdk, a.output)
