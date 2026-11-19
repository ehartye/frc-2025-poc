"""Replay both attained poses through the release or candidate controller and store the full departure state."""

import argparse, hashlib, json, subprocess, tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from split_checkpoint import ROOT, HERE, Station, coupled, ah, PIVOT_EPOCH, vec
from split_controller import candidate, digest


def write(p, data):
    p.write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )


def run_one(job):
    side, dt, jdk, classes, output, name, deadline = job
    pose = json.loads((ROOT / "conditioned-match/opening.json").read_bytes())[side][
        "sourceRobotMmDeg"
    ]
    profile = Station
    if name in ("locked-belts", "lost-traction"):
        from split_checkpoint import ClosedLoopDisturbance

        profile = ClosedLoopDisturbance
        profile.disturbance = name
    folder = output / name / side / f"us-{1e6 * dt:g}"
    folder.mkdir(parents=True, exist_ok=True)
    f = coupled.feed
    plant = profile(
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
    old, cfg = f.Pipe, f.cfg
    f.Pipe = lambda *_: plant
    f.cfg = replace(f.cfg, slices=10)
    try:
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
            result = f.trial(
                proc, pose, side, dt, 0, folder, relief=True, until_ms=deadline
            )
            proc.stdin.close()
            proc.wait()
        tick = result["portTicks"][-1]
        state = plant.capture(tick[-1])
    finally:
        f.Pipe, f.cfg = old, cfg
    base = [
        r
        for r in json.loads((ROOT / "compliant-carriage/series.json").read_bytes())[
            "cases"
        ]
        if r["case"] == side and r["stepS"] == dt
    ][0]
    if name == "released" and (
        base["lastState"] != result["lastPortState"]
        or base["portTransitions"][-1][:4] != tick[:4]
    ):
        raise ValueError("Released outcome changed")
    if (
        name == "released"
        and base["lastState"] == "FAULT"
        and result["trajectorySHA256"] != base["trajectorySha256"]
    ):
        raise ValueError("Released fault trace changed")
    m = plant.model
    before = dict(
        id=name,
        side=side,
        stepS=dt,
        sourcePoseMmDeg=pose,
        frame={
            k: ah(m[k])
            for k in (
                "table",
                "basis",
                "linearMass",
                "linearStiffness",
                "site",
                "relative",
                "damperRel",
            )
        },
        frequenciesRadps=vec(plant.freq),
        sourceProgramSha256=digest(classes / "PortAcquisition.java"),
        previousCaseTrajectorySha256=base["trajectorySha256"],
        trajectorySha256=result["trajectorySHA256"],
        state=result["lastPortState"],
        fault=result["reason"],
        firstBandJavaMs=result["firstPortBandMs"],
        physicalCaptureQualified=False,
        wallEpoch=PIVOT_EPOCH,
        terminal=state,
        exposures=plant.exposure,
        portTicks=result["portTicks"],
    )
    # At camera exposure, bind the held most recent Java command/permissions, never a future service.
    for ms, row in list(plant.exposure.items()) + [(str(tick[0]), state)]:
        prior = next(t for t in reversed(result["portTicks"]) if t[0] <= int(ms))
        row["javaHeldFromMs"] = prior[0]
        row["javaState"] = prior[1]
        row["javaCommandMps"] = prior[-1]
    write(folder / "result.json", before)
    print(
        name,
        side,
        dt,
        result["lastPortState"],
        result["firstPortBandMs"],
        tick[0],
        flush=True,
    )
    return before


def main(args):
    groups = {
        "candidate": ("leftSmall", "rightSmall"),
        "released": ("leftSmall", "rightSmall"),
        "locked-belts": ("leftSmall",),
        "lost-traction": ("leftSmall",),
    }
    a = args.name
    steps = args.steps
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        cls = Path(tmp)
        code = cls / "PortAcquisition.java"
        code.write_text(
            candidate(coupled.feed)
            if a != "released"
            else coupled.feed.port_program(True),
            encoding="utf8",
            newline="\n",
        )
        subprocess.run(
            [
                str(args.jdk / "bin/javac"),
                "-d",
                str(cls),
                str(code),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        jobs = [
            (s, dt * 1e-6, args.jdk, cls, args.output, a, args.deadline)
            for s in groups[a]
            for dt in steps
        ]
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            rows = list(pool.map(run_one, jobs))
    write(
        args.out,
        {
            "sourceBasis": digest(ROOT / "compliant-carriage/model.py"),
            "original": digest(ROOT / "compliant-carriage/series.json"),
            "cases": rows,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "name", choices=("released", "candidate", "locked-belts", "lost-traction")
    )
    p.add_argument("jdk", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument("--deadline", type=int, default=5460)
    p.add_argument("--steps", type=float, nargs="+", default=[31.25, 15.625])
    p.add_argument("--workers", type=int, default=2)
    main(p.parse_args())
