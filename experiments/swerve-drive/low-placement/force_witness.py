"""Compare the two registered finite trough solvers on actual entering and settled states."""

import argparse
import subprocess
import tempfile
from pathlib import Path
import numpy as np
from target_contact import Trough, I, IP
from trough import HERE, SIX, read, sha, panel_file
import json


def run(artifacts, jdk):
    source = SIX / "spatial-contact/SpatialContact.java"
    target_file = SIX / "mission/target.json"
    target = read(target_file)
    expected, csv_rows, sources = [], [], {}
    for f in sorted(
        (artifacts / "low-placement/advance-25").glob("*/joined-*-nominal.json")
    ):
        trace = f.with_suffix(".npy")
        if sha(trace) != read(f)["trace"]:
            raise ValueError("Different pipe trace")
        dock = read(f.parent / "chassis.json")["terminalMmMmRad"]
        samples = np.load(trace)
        sources[f.parent.name] = sha(trace)
        for n in (9, 17):
            for mu in (0.10, 0.35):
                field = Trough(target, dock, n, mu)
                for row in samples[::9]:
                    world = (row[2:5], row[8:11], row[5:8], row[11:14])
                    p, v, a, l = field.state(world)
                    omega = l / IP + (1 / I - 1 / IP) * a * (a @ l)
                    force, torque, spans, _, compression = field.solve(world)
                    expected.append(
                        [
                            *(field.rot @ force),
                            *(field.rot @ torque),
                            *spans,
                            compression,
                        ]
                    )
                    csv_rows.append(
                        ",".join(
                            map(
                                str,
                                [
                                    "sample",
                                    *p,
                                    *v,
                                    *a,
                                    *omega,
                                    0,
                                    0,
                                    0,
                                    0,
                                    3,
                                    0.0001,
                                    n,
                                    mu,
                                ],
                            )
                        )
                    )
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        panel_file(target, tmp / "panels.csv")
        (tmp / "rack.csv").write_text("time,stroke\n0,0\n")
        (tmp / "empty.csv").touch()
        (tmp / "cases.csv").write_text("\n".join(csv_rows))
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                d,
                str(source),
                str(HERE / "ForceWitness.java"),
            ],
            check=True,
        )
        output = subprocess.check_output(
            [
                str(jdk / "bin/java"),
                "-cp",
                d,
                "ForceWitness",
                *[
                    str(tmp / i)
                    for i in (
                        "panels.csv",
                        "rack.csv",
                        "empty.csv",
                        "report.csv",
                        "trace.csv",
                        "cases.csv",
                    )
                ],
            ],
            text=True,
        )
    predicted = np.asarray(expected)
    actual = np.asarray(
        [list(map(float, line.split())) for line in output.splitlines()]
    )
    delta = np.max(np.abs(predicted - actual), axis=0)
    if (
        max(delta) > 1e-7
        or not np.any(actual[:, 6] > 0)
        or not np.any(actual[:, 7] > 0)
    ):
        raise ValueError(f"Unverified floor/lip wrench {delta}")
    return dict(
        source=sha(source),
        comparison=sha(HERE / "target_contact.py"),
        field=sha(target_file),
        inputs=sources,
        samples=len(actual),
        floorSamples=int((actual[:, 6] > 0).sum()),
        lipSamples=int((actual[:, 7] > 0).sum()),
        maxForceErrorN=delta[:3].tolist(),
        maxTorqueErrorNm=delta[3:6].tolist(),
        maxSpanAndCompressionErrorM=delta[6:].tolist(),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("artifacts", "jdk", "output"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    a.output.write_bytes(
        (json.dumps(run(a.artifacts, a.jdk), indent=2) + "\n").encode()
    )
