"""Advance the attained pipe, two fitted guide backings, machined tray and port program."""

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import json
import importlib.util
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "elastic-carry"))
import checkpoint
from beam import Beam, digest, LAYOUTS
from backing import MovingGuides, GuideResponse

coupled = checkpoint.coupled
feed = coupled.feed


def write(path, data):
    path.write_text(
        json.dumps(data, indent=2, allow_nan=False) + "\n",
        encoding="utf8",
        newline="\n",
    )


def trial(job):
    (
        case,
        pose,
        layout,
        micros,
        kit,
        jdk,
        classes,
        cache,
        until,
        cutoff,
        moving,
        support,
        outward,
        collect,
    ) = job
    dt = micros * 1e-6
    beam = Beam(kit, layout, cutoff, arm_fraction=moving)
    name = f"{case}-{layout}-{micros:g}us-{cutoff:g}Hz-{moving:g}-{until}"
    if outward:
        datum = json.loads((kit / "datum.json").read_bytes())
        if (
            datum["stepSha256"] != beam.source["step"]
            or datum["outboardOffsetsMm"]["guide"] != outward
        ):
            raise ValueError("No matching native outboard registration")
        name += f"-out-{outward:g}mm"
    if support:
        name += "-serial-carriage"
    folder = cache / name
    folder.mkdir(exist_ok=True, parents=True)
    base = coupled.CoupledPipe
    support_pins = {}
    if support:
        file = support / "model.py"
        spec = importlib.util.spec_from_file_location("guide_carriage_boundary", file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        base = module.FlexiblePipe
        base.settings = (1.0, 5e6, 1.0, 0.002, 0.0001, 1.0, 2)
        support_pins = {k: digest(support / k) for k in ("model.py", "boundary.json")}

    if collect:
        r = json.loads((folder / "result.json").read_bytes())
        if (
            r["id"] != name
            or r["poseMmDeg"] != pose
            or r["binding"] != beam.receipt()
            or r.get("outwardPerSideMm", 0.0) != outward
            or r.get("carriageSupport", {}) != support_pins
        ):
            raise ValueError("Stored trial uses different inputs")
        for item, attr in (
            (name + ".npy", "trajectorySha256"),
            ("state.npz", "stateSha256"),
            ("guide.npy", "guideHistorySha256"),
        ):
            if digest(folder / item) != r[attr]:
                raise ValueError("Stored trial data changed")
        return r

    class FlexibleGuidePipe(MovingGuides, base):
        pass

    cls = FlexibleGuidePipe if layout != "rigid" else base
    pipe = cls(
        pose,
        0.0,
        replace(
            coupled.GUIDE,
            sectors=128,
            radial_bins=4,
            upper_mm=813,
            throat_mm=coupled.GUIDE.throat_mm + outward,
        ),
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
    pipe.guides = GuideResponse(beam)
    prior, prior_cfg = feed.Pipe, feed.cfg
    feed.Pipe = lambda *_: pipe
    feed.cfg = replace(feed.cfg, slices=10)
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
            r = feed.trial(proc, pose, name, dt, 0, folder, relief=True, until_ms=until)
            proc.stdin.close()
            if proc.wait() != 0:
                raise RuntimeError("Port program exited")
    finally:
        feed.Pipe, feed.cfg = prior, prior_cfg
    history = folder / "guide.npy"
    np.save(history, np.array(pipe.guides.trace))
    result = pipe.guides.finish(pipe.time)
    state = folder / "state.npz"
    np.savez(
        state,
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
        guide_q=pipe.guides.q,
        guide_v=pipe.guides.v,
        guide_w=beam.w,
    )
    transitions = []
    for row in r["portTicks"]:
        if not transitions or row[1:] != transitions[-1][1:]:
            transitions.append(row)
    row = {
        "id": name,
        "case": case,
        "poseMmDeg": pose,
        "layout": layout,
        "stepUs": micros,
        "deadlineMs": until,
        "outwardPerSideMm": outward,
        "binding": beam.receipt(),
        "lastJavaMs": r["portTicks"][-1][0],
        "firstBandMs": r["firstPortBandMs"],
        "lastState": r["lastPortState"],
        "reason": r["reason"],
        "portTransitions": transitions,
        "loss": r["loss"],
        "hardStop": r["hardStop"],
        "finalPositionMm": r["finalPositionMm"],
        "finalAxis": r["finalAxis"],
        "beltMps": pipe.belt,
        "firstShelfContact": pipe.touch,
        "peakShelfNormalN": pipe.peak_shelf_n,
        "damperWorkJ": pipe.plunger_energy.tolist(),
        "carriageSupport": support_pins,
        "shelfDisplacementSlopeTable": hashlib.sha256(
            np.asarray(pipe.table, dtype="<f8").tobytes()
        ).hexdigest(),
        "guideDisplacementTable": hashlib.sha256(
            np.asarray(beam.phi, dtype="<f8").tobytes()
        ).hexdigest(),
        "guideSlopeTable": hashlib.sha256(
            np.asarray(beam.slope_phi, dtype="<f8").tobytes()
        ).hexdigest(),
        "peakBeltCellN": pipe.peak_load,
        "peakCartridgeTravelMm": float(pipe.peak),
        "guide": result if layout != "rigid" else {"peakGuideN": pipe.peak_guiding_n},
        "trajectorySha256": r["trajectorySHA256"],
        "stateSha256": digest(state),
        "guideHistorySha256": digest(history),
    }
    write(folder / "result.json", row)
    print(name, row["lastState"], row["firstBandMs"], row["lastJavaMs"], flush=True)
    return row


def run(a):
    src = ROOT / "conditioned-match/elastic.json"
    reg = json.loads(src.read_bytes())
    rows = {
        r["case"]: r
        for r in reg["cases"]
        if r.get("machinedTray") and r["stepS"] == 0.00003125
    }
    if set(rows) != {"leftSmall", "rightSmall"}:
        raise ValueError("Expected the two signed powered arrivals")
    a.cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        cls = Path(tmp)
        code = cls / "PortAcquisition.java"
        code.write_text(feed.port_program(True), encoding="utf8", newline="\n")
        expected = json.loads((ROOT / "station-landmark/feed.json").read_bytes())[
            "programSha256"
        ]
        if digest(code) != expected:
            raise ValueError("Wrong custody program")
        subprocess.run(
            [
                str(a.jdk / "bin/javac"),
                "-d",
                str(cls),
                str(code),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        jobs = [
            (
                case,
                rows[case]["sourceRobotMmDeg"],
                layout,
                dt,
                a.kit,
                a.jdk,
                cls,
                a.cache,
                a.until,
                a.cutoff,
                a.arm_fraction,
                a.support,
                a.outward,
                a.collect,
            )
            for dt in a.steps
            for layout in a.layouts
            for case in a.cases
        ]
        with ProcessPoolExecutor(max_workers=a.workers) as pool:
            results = list(pool.map(trial, jobs))
    write(
        a.out,
        {
            "stationSource": digest(src),
            "controller": expected,
            "shelfBasis": digest(ROOT / "elastic-carry/mode-frame.json"),
            "upstreamTrajectories": {k: r["trajectorySha256"] for k, r in rows.items()},
            "cases": results,
            "physicalCapture": None,
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--collect",
        action="store_true",
        help="Verify the cached input identities and three captured data files for each requested trial",
    )
    p.add_argument("jdk", type=Path)
    p.add_argument("cache", type=Path)
    p.add_argument("out", type=Path)
    p.add_argument(
        "--outward",
        type=float,
        default=0.0,
        help="Symmetric installed shift in mm per liner",
    )
    p.add_argument(
        "--support",
        type=Path,
        help="Optional separately sprung carriage directory with the two serial guide stages",
    )
    p.add_argument("--kit", type=Path, default=ROOT / "guide-mounts")
    p.add_argument(
        "--cases",
        nargs="+",
        default=["leftSmall", "rightSmall"],
        choices=["leftSmall", "rightSmall"],
    )
    p.add_argument("--layouts", nargs="+", choices=list(LAYOUTS), default=list(LAYOUTS))
    p.add_argument("--steps", nargs="+", type=float, default=[62.5, 31.25])
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--until", type=int, default=5460)
    p.add_argument("--cutoff", type=float, default=1800)
    p.add_argument("--arm-fraction", type=float, default=0.236)
    run(p.parse_args())
