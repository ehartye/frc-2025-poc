"""Recover the live plate, both plungers and pipe at the powered station departure."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "shelf-structure"))
import coupled


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def plant(pose):
    return coupled.CoupledPipe(
        pose,
        0.0,
        replace(coupled.GUIDE, sectors=128, radial_bins=4),
        coupled.Shelf(400000, 4, stations=48),
        True,
        2300.0,
        0.02,
        0.0,
        1500.0,
        (-0.066, 0.042),
        0.0001,
        True,
    )


def replay(job):
    register, folder, jdk, classes = job
    name = register["case"]
    pose = register["sourceRobotMmDeg"]
    pipe = plant(pose)
    feed = coupled.feed
    feed.Pipe = lambda *_: pipe
    feed.cfg = replace(feed.cfg, slices=10)
    with subprocess.Popen(
        [
            str(jdk / "bin/java"),
            "-cp",
            str(classes),
            "org.reefscape.controls.PortConsole",
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    ) as proc:
        result = feed.trial(
            proc, pose, name, 0.00003125, 0, folder, relief=True, until_ms=5460
        )
        proc.stdin.close()
        proc.wait()
    if (
        result["trajectorySHA256"] != register["trajectorySha256"]
        or result["lastPortState"] != "IN_BAND"
    ):
        raise ValueError(
            "Station handoff differs from the powered, observed source " + name
        )
    if result["portTicks"][-1][0] != 5460 or pipe.hard or pipe.loss or pipe.belt != 0.0:
        raise ValueError("Handoff lacks stationary seated custody")
    dest = folder / (name + "-state.npz")
    np.savez(
        dest,
        p=pipe.p,
        a=pipe.a,
        v=pipe.v,
        l=pipe.l,
        z=pipe.z,
        zdot=pipe.zdot,
        plunger=pipe.plunger,
        time=pipe.time,
        belt=pipe.belt,
        touch=np.array(json.dumps(pipe.touch)),
        first_belts=np.asarray(
            [t if t is not None else np.nan for t in pipe.first_belts]
        ),
        freq=pipe.freq,
        casePose=np.asarray(pose),
    )
    return check_saved(folder, register)


def check_saved(folder, r):
    name = r["case"]
    body = folder / (name + ".npy")
    state = folder / (name + "-state.npz")
    if (
        digest(body) != r["trajectorySha256"]
        or r["lastState"] != "IN_BAND"
        or r["lastJavaMs"] != 5460
    ):
        raise ValueError("Raw departure or port state changed")
    last = np.load(body, mmap_mode="r")[-1]
    with np.load(state, allow_pickle=False) as d:
        for key, left, right, scale in (
            ("p", 1, 4, 1000.0),
            ("a", 4, 7, 1.0),
            ("v", 7, 10, 1.0),
        ):
            np.testing.assert_array_equal(d[key] * scale, last[left:right])
        if (
            abs(float(d["time"]) * 1000 + 100 - last[0]) > 1e-7
            or float(d["belt"]) != 0.0
        ):
            raise ValueError("Station sample and elastic state have different clocks")
        return {
            "case": name,
            "stationTrajectorySha256": digest(body),
            "stateSha256": digest(state),
            "javaMs": 5460,
            "secondsFromLip": float(d["time"]),
            "nextBeltMps": float(d["belt"]),
            "modeCount": len(d["z"]),
            "modalPosition": d["z"].tolist(),
            "modalVelocity": d["zdot"].tolist(),
            "plungerMm": (d["plunger"] * 1000).tolist(),
            "pipeAngularMomentumKgM2PerS": d["l"].tolist(),
        }


def run(study, jdk, cache, out, collect=False):
    source = study / "elastic.json"
    source_hash = digest(source)
    inputs = json.loads(source.read_bytes())
    rows = [
        r for r in inputs["cases"] if r.get("machinedTray") and r["stepS"] == 0.00003125
    ]
    if len(rows) != 2 or {r["case"] for r in rows} != {"leftSmall", "rightSmall"}:
        raise ValueError("Both powered machined arrivals required")
    cache.mkdir(exist_ok=True, parents=True)
    with tempfile.TemporaryDirectory() as tmp:
        cls = Path(tmp)
        java = cls / "PortAcquisition.java"
        java.write_text(coupled.feed.port_program(True), encoding="utf8", newline="\n")
        expected = json.loads((ROOT / "station-landmark/feed.json").read_bytes())[
            "programSha256"
        ]
        if digest(java) != expected:
            raise ValueError("Different port controller")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(cls),
                str(java),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        if collect:
            results = [check_saved(cache, r) for r in rows]
        else:
            with ProcessPoolExecutor(max_workers=2) as pool:
                results = list(pool.map(replay, [(r, cache, jdk, cls) for r in rows]))
    table, w, _ = coupled.tables(True, 2300.0, 0.0, True)
    frame = {
        "coupledSha256": digest(ROOT / "shelf-structure/coupled.py"),
        "responseSha256": digest(ROOT / "shelf-structure/response.py"),
        "shape": list(table.shape),
        "tableSha256": hashlib.sha256(
            np.asarray(table, dtype="<f8").tobytes()
        ).hexdigest(),
        "frequencyHz": (w / (2 * np.pi)).tolist(),
    }
    out.with_name("mode-frame.json").write_text(
        json.dumps(frame, indent=2) + "\n", encoding="utf8", newline="\n"
    )
    out.write_text(
        json.dumps(
            {
                "poweredStation": source_hash,
                "controllerSha256": expected,
                "cases": results,
            },
            indent=2,
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("study", "jdk", "cache", "out"):
        p.add_argument(key, type=Path)
    p.add_argument(
        "--collect",
        action="store_true",
        help="Read completed station states and verify the full published trajectories",
    )
    a = p.parse_args()
    run(a.study, a.jdk, a.cache, a.out, a.collect)
