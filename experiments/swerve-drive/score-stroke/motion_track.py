"""Export the acquired pipe, finite rack stroke and independent trough trial in chassis millimeters."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
CASE = "plus/coral-A/3.001/nominal"


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verify(path, digest):
    if sha(path) != digest:
        raise ValueError(f"Source changed: {path.name}")


def at(times, source, columns):
    return np.column_stack(
        [np.interp(times, source[:, 0], source[:, i]) for i in columns]
    )


def rows(times, strokes, centers, axes):
    unit = axes / np.linalg.norm(axes, axis=1, keepdims=True)
    return np.round(np.column_stack([times, strokes, centers, unit]), 7).tolist()


def build(raw):
    candidate = read(HERE / "candidate.json")
    landing = read(HERE / "landing.json")
    feed_id = "cam-station-plus-31.25us-0.1mm"
    feed = candidate["feeds"][feed_id]
    (station,) = (
        path for path in (raw / feed_id).rglob("*.npy") if path.name != "guide.npy"
    )
    verify(station, feed["pipeTrajectorySha256"])
    physical = np.load(station, mmap_mode="r")
    feed_times = np.unique(np.r_[physical[0, 0], np.arange(125, 5460, 25), 1340, 5460])
    pickup = at(feed_times, physical, range(1, 7))
    cycle = raw / "plus-cam-cycle-31.25us.npz"
    free = raw / "plus-free-31.25us-3s-400000-cam.npz"
    verify(cycle, candidate["cycles"][cycle.stem]["traceSha256"])
    verify(free, candidate["freePipe"][free.stem]["traceSha256"])
    with np.load(free) as f:
        vertical = f["samples"]
    with np.load(cycle) as f:
        rack = np.column_stack([f["timeS"], f["strokeM"] * 1000])
    times = np.unique(np.r_[np.arange(0, 3.001, 0.025), 3.001])
    stroke = np.interp(times, rack[:, 0], rack[:, 1])
    # The free-pipe calculation solves shelf-relative vertical motion and roll.
    # Longitudinal/lateral custody during this dock fixture is prescribed.
    center = np.tile(pickup[-1, :3], (len(times), 1))
    center[:, 0] += stroke
    center[:, 2] += np.interp(times, vertical[:, 0], vertical[:, 4])
    axis = np.tile(pickup[-1, 3:], (len(times), 1))
    axis[:, 2] += np.interp(times, vertical[:, 0], vertical[:, 5])
    trajectory = raw / "candidate-landing/trajectory.csv"
    verify(raw / "candidate-landing/cases.csv", landing["inputSha256"])
    with trajectory.open(newline="") as f:
        trace = np.array(
            [
                [
                    float(r[k])
                    for k in ("t_s", "x_m", "y_m", "z_m", "axis_x", "axis_y", "axis_z")
                ]
                for r in csv.DictReader(f)
                if r["case"] == CASE
            ]
        )
    outcome = next(r for r in landing["cases"] if r["case"] == CASE)
    if (
        len(trace) != 61
        or outcome["outcome"] != "supported"
        or not np.allclose(
            trace[-1, 1:4], [outcome[k] for k in ("x_m", "y_m", "z_m")], atol=1e-8
        )
    ):
        raise ValueError("Landing sample register differs")
    path = PROJECT / "experiments/six-wheel/relieved-match/coral-A-coupled-250.json"
    dock = read(path)
    base = np.r_[
        np.array(dock["chassisAtReefXYm"]) - [landing["addedSetbackMm"] / 1000, 0], 0
    ]
    yaw = dock["chassisHeadingRad"]
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    relative = (trace[:, 1:4] - base) @ R * 1000
    floor_time = trace[:, 0]
    return dict(
        schema="reefscape-mechanism-film/1",
        piece="coral-A",
        case=CASE,
        geometrySha256=read(HERE / "cam.json")["candidateStepSha256"],
        sources={
            p.name: sha(p)
            for p in (
                station,
                cycle,
                free,
                trajectory,
                HERE / "candidate.json",
                HERE / "landing.json",
                path,
            )
        },
        columns=[
            "chapterSeconds",
            "rackMm",
            "pipeXmm",
            "pipeYmm",
            "pipeZmm",
            "axisX",
            "axisY",
            "axisZ",
        ],
        station=rows(feed_times / 1000, 0 * feed_times, pickup[:, :3], pickup[:, 3:]),
        extend=rows(times, stroke, center, axis),
        release=rows(
            floor_time,
            np.interp(3.3 + floor_time, rack[:, 0], rack[:, 1]),
            relative,
            trace[:, 4:] @ R,
        ),
        dockOriginM=base.tolist(),
        dockYawRad=yaw,
        firstInBandS=feed["javaFirstBandMs"] / 1000,
        landing=outcome,
        deliveryCredit=0,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(build(a.raw), separators=(",", ":"), allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
