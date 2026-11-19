"""Verify saved modal poses and replay the original held commands from the first exposure to departure."""

import argparse
import json
from pathlib import Path
import numpy as np
from dataclasses import replace
from split_checkpoint import HERE, ROOT, Station, coupled, EXPOSURES, ah, vec
from split_controller import digest


def restore(row):
    f = coupled.feed
    pose = row["sourcePoseMmDeg"]
    p = Station(
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
    if any(ah(p.model[key]) != h for key, h in row["frame"].items()):
        raise ValueError("Basis digest mismatch")
    return p


def seed(p, s):
    for key in ("p", "a", "v", "l", "z", "zdot"):
        setattr(p, key, np.asarray(s[key], dtype=float))
    p.plunger = np.asarray(s["plungerM"])
    p.belt = s["actualBeltMps"]
    p.time = s["secondsFromLip"]
    p.touch = s["touch"]
    p.first_belts = s["firstBeltsS"]
    p.hard = s["hard"]
    p.loss = s["loss"]
    p.next_exposure = None


def compare(record, root):
    f = coupled.feed
    f.cfg = replace(f.cfg, slices=10)
    runs = []
    timeline = json.loads((ROOT / "relieved-match/first-feed.json").read_bytes())
    for row in record["cases"]:
        raw = (
            root
            / "candidate"
            / row["side"]
            / f"us-{1e6 * row['stepS']:g}"
            / (row["side"] + ".npy")
        )
        if digest(raw) != row["trajectorySha256"]:
            raise ValueError("Different pipe history")
        track = np.load(raw, mmap_mode="r")
        snaps = row["exposures"]
        if len(snaps) != len(EXPOSURES):
            runs.append(
                {
                    "side": row["side"],
                    "stepS": row["stepS"],
                    "state": row["state"],
                    "departure": False,
                }
            )
            continue
        if row["state"] != "IN_BAND" or row["terminal"]["javaMs"] != 5460:
            raise ValueError("Incomplete departure")
        p = restore(row)
        s = snaps[str(EXPOSURES[0])]
        seed(p, s)
        errors = {
            k: 0.0
            for k in (
                "p",
                "a",
                "v",
                "l",
                "z",
                "zdot",
                "plungerM",
                "landsM",
                "guideStagesM",
                "damperBaseM",
            )
        }
        max_stamp = 0.0
        positions = []
        for ms in EXPOSURES:
            target = (ms - 100) / 1000
            while target - p.time > 1e-11:
                java_ms = p.time * 1000 + 100
                cmd = next(
                    x for x in reversed(row["portTicks"]) if x[0] <= java_ms + 1e-6
                )[-1]
                # Retain the original integration boundary at each Java 20 ms tick.
                next_service = (int((java_ms + 1e-6) // 20) + 1) * 20
                dt = min(row["stepS"], target - p.time, (next_service - java_ms) / 1000)
                p.moves(cmd, dt)
            expected = snaps[str(ms)]
            actual = p.capture(expected["javaCommandMps"])
            if expected["apertures"] != actual["apertures"]:
                raise ValueError("Resolved port observations changed")
            for k in errors:
                errors[k] = max(
                    errors[k],
                    float(np.max(abs(np.asarray(expected[k]) - np.asarray(actual[k])))),
                )
            sample = track[np.searchsorted(track[:, 0], ms - 1e-6)]
            max_stamp = max(max_stamp, abs(float(sample[0]) - ms))
            positions.append(
                float(np.linalg.norm(np.asarray(expected["p"]) * 1000 - sample[1:4]))
            )
        if max_stamp > 1e-5 or max(positions) > 1e-8:
            raise ValueError("Exposure source differs from trajectory")
        if max(errors.values()) > 1e-6:
            raise ValueError(f"Resumed short trajectory differs {errors}")
        tail = timeline["scheduledDriveWallS"] - row["terminal"]["wallS"]
        if not 0 < tail < 0.001:
            raise ValueError("Changed wheel service clock")
        # Continue the recorded neutral station command to the instant of first wheel service.
        seed(p, row["terminal"])
        until = p.time + tail
        while until - p.time > 1e-11:
            p.moves(
                row["terminal"]["javaCommandMps"], min(row["stepS"], until - p.time)
            )
        wheel = p.capture(row["terminal"]["javaCommandMps"])
        wheel["javaMs"] = p.time * 1000 + 100
        wheel["javaHeldFromMs"] = 5460
        wheel["javaState"] = row["terminal"]["javaState"]
        wheel["appliedWheelMotion"] = False
        runs.append(
            {
                "side": row["side"],
                "stepS": row["stepS"],
                "state": row["state"],
                "departure": True,
                "exposuresMs": list(EXPOSURES),
                "motionReplayMaxAbsByField": errors,
                "traceVsCheckpointMaxMm": max(positions),
                "secondsToNextWheelService": tail,
                "beforeFirstWheelService": wheel,
            }
        )
    return {
        "source": digest(HERE / "candidate.json"),
        "cases": runs,
        "physicalQualification": None,
    }


if __name__ == "__main__":
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("raw", type=Path)
    a.add_argument("out", type=Path)
    ns = a.parse_args()
    ns.out.write_text(
        json.dumps(
            compare(json.loads((HERE / "candidate.json").read_bytes()), ns.raw),
            indent=2,
        )
        + "\n",
        encoding="utf8",
        newline="\n",
    )
