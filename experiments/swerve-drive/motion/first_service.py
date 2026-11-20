"""Propagate the stationary, held-command pickup through the first 5 ms wheel-service boundary."""

import argparse
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "swerve_feed", Path(__file__).with_name("feed.py")
)
swerve_feed = importlib.util.module_from_spec(spec)
spec.loader.exec_module(swerve_feed)
engine, digest, HERE, ROOT = (
    swerve_feed.engine,
    swerve_feed.digest,
    swerve_feed.HERE,
    swerve_feed.ROOT,
)


def restore(raw, feed):
    step = feed["plantStepUs"] * 1e-6
    (start,) = (raw / feed["case"]).glob(
        f"*folded-{feed['plantStepUs']:g}us*/state.npz"
    )
    if digest(start) != feed["pipeStateSha256"]:
        raise ValueError("Changed terminal pipe/guide/carriage state")
    row = json.loads((start.parent / "result.json").read_bytes())
    if (
        row["loss"]
        or row["hardStop"]
        or row["stateSha256"] != feed["pipeStateSha256"]
        or row["programSha256"] != feed["programSha256"]
    ):
        raise ValueError("Incomplete acquisition cannot supply a release state")
    support = ROOT / "compliant-carriage"
    if any(digest(support / k) != v for k, v in feed["receiverBoundaries"].items()):
        raise ValueError("Different supported carriage")
    spec = importlib.util.spec_from_file_location(
        "service_carriage", support / "model.py"
    )
    carriage = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(carriage)
    beam = engine.Beam(ROOT / "guide-mounts", "folded", 1800, arm_fraction=0.236)
    if row["binding"] != beam.receipt():
        raise ValueError("Different straight guide geometry")

    class Pipe(engine.MovingGuides, carriage.FlexiblePipe):
        settings = (1.0, 5e6, 1.0, 0.002, 0.0001, 1.0, 2)

    coupled = engine.coupled
    x, y, heading = feed["trueChassisAtFeedMmMmDeg"]
    pose = [-x, -y, heading]
    pipe = Pipe(
        pose,
        0.0,
        replace(coupled.GUIDE, sectors=128, radial_bins=4, upper_mm=813),
        coupled.Shelf(400000, 4, stations=48),
        True,
        2300.0,
        0.02,
        0.0,
        1500.0,
        (-0.066, 0.042),
        feed.get("capGapMm", 0.1) / 1000,
        True,
    )
    pipe.guides = engine.GuideResponse(beam)
    with np.load(start, allow_pickle=False) as state:
        for k, value in [("freq", pipe.freq), ("guide_w", beam.w), ("casePose", pose)]:
            np.testing.assert_array_equal(state[k], value)
        for k in ("p", "a", "v", "l", "z", "zdot", "plunger"):
            setattr(pipe, k, state[k].copy())
        pipe.time, pipe.belt = float(state["time"]), float(state["belt"])
        pipe.touch = json.loads(str(state["touch"]))
        pipe.first_belts = [
            None if np.isnan(v) else float(v) for v in state["first_belts"]
        ]
        pipe.guides.q, pipe.guides.v = state["guide_q"].copy(), state["guide_v"].copy()
    if pipe.belt != 0.0 or abs(pipe.time + feed["lipWallS"] - feed["endWallS"]) > 1e-8:
        raise ValueError("Station time or held belt command differs")
    pipe.guides.last = pipe.time
    return pipe, step, beam


def propagate(raw, feed, destination):
    pipe, step, beam = restore(raw, feed)
    first = math.ceil(feed["endWallS"] * 200 - 1e-9) / 200
    target = first - feed["lipWallS"]
    initial = pipe.p.copy()
    max_shift = 0.0
    steps = 0
    old_cfg = engine.feed.cfg
    engine.feed.cfg = replace(old_cfg, slices=10)
    try:
        while target - pipe.time > 1e-12:
            pipe.moves(0.0, min(step, target - pipe.time))
            steps += 1
            max_shift = max(max_shift, np.linalg.norm(pipe.p - initial) * 1000)
    finally:
        engine.feed.cfg = old_cfg
    pipe.guides.finish(pipe.time)
    destination.mkdir(parents=True, exist_ok=True)
    name = f"{feed['case']}-{feed['plantStepUs']:g}us-service.npz"
    arrays = {
        k: getattr(pipe, k)
        for k in ("p", "a", "v", "l", "z", "zdot", "plunger", "time", "belt", "freq")
    }
    np.savez(
        destination / name,
        **arrays,
        guide_q=pipe.guides.q,
        guide_v=pipe.guides.v,
        guide_w=beam.w,
        touch=json.dumps(pipe.touch),
        first_belts=[np.nan if v is None else v for v in pipe.first_belts],
        casePose=[
            -feed["trueChassisAtFeedMmMmDeg"][0],
            -feed["trueChassisAtFeedMmMmDeg"][1],
            feed["trueChassisAtFeedMmMmDeg"][2],
        ],
    )
    return dict(
        case=feed["case"],
        stepUs=feed["plantStepUs"],
        piece=feed["physicalPieceId"],
        incoming=feed["pipeStateSha256"],
        outgoing=digest(destination / name),
        wallS=first,
        retainedContactSubsteps=steps,
        durationMs=1000 * (first - feed["endWallS"]),
        peakCenterDisplacementMm=max_shift,
        centerMm=(1000 * pipe.p).tolist(),
        axis=pipe.a.tolist(),
        terminalStageDisplacementMm=float(
            max(abs(pipe.model["relative"] @ pipe.z)) * 1000
        ),
        loss=pipe.loss,
        hardStop=pipe.hard,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("feeds", type=Path)
    parser.add_argument("checkpoints", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--feed-register", type=Path, default=HERE / "first-feed.json")
    args = parser.parse_args()
    rows = [
        propagate(args.feeds, f, args.checkpoints)
        for f in json.loads(args.feed_register.read_bytes())["attempts"]
    ]
    args.output.write_text(
        json.dumps(rows, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )
