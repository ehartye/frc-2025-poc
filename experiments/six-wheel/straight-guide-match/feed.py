"""Run the released two-stage feeder from the attained opening on the selected guides."""

import argparse
import json
from pathlib import Path
import sys
import subprocess
import tempfile

from plant import HERE, ROOT, read, digest

sys.path.insert(0, str(ROOT / "guide-impact"))
import importlib.util

spec = importlib.util.spec_from_file_location(
    "signed_feed", ROOT / "guide-impact/replay.py"
)
engine = importlib.util.module_from_spec(spec)
spec.loader.exec_module(engine)
sys.path.insert(0, str(ROOT / "split-guide-acquisition"))
from split_controller import candidate


def run(assets, jdk, side, dt, offset=0):
    start = read(HERE / "revised-opening.json")[side]
    if start["status"] != "observed stationary":
        raise ValueError("No fresh station lock")
    x, y, h = start["reachedMmDeg"]
    point = [-x, -y, h]
    cache = assets / "straight-guide-match/attained-feed"
    if offset:
        trial = next(
            r
            for r in read(HERE / "station-uncertainty.json")["cases"]
            if r["side"] == side and r["offsetMm"] == offset
        )
        if start["pathSha256"] != trial["pathSha256"]:
            raise ValueError("Changed opening for contact control")
        point = [-x, -(y + offset), h + trial["yawDifferenceDeg"]]
        cache = assets / "straight-guide-match/opening-offsets" / f"{offset:+g}mm"
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temp:
        classes = Path(temp)
        src = classes / "PortAcquisition.java"
        src.write_text(candidate(engine.feed), encoding="utf8", newline="\n")
        program = read(ROOT / "split-guide-acquisition/signals.json")["programSha256"]
        if digest(src) != program:
            raise ValueError("Different Java program")
        subprocess.run(
            [
                str(jdk / "bin/javac"),
                "-d",
                str(classes),
                str(src),
                str(ROOT / "observable-acquisition/PortConsole.java"),
            ],
            check=True,
        )
        return engine.trial(
            (
                side,
                point,
                "folded",
                dt,
                ROOT / "guide-mounts",
                jdk,
                classes,
                cache,
                5460,
                1800.0,
                0.236,
                ROOT / "compliant-carriage",
                0.0,
                False,
                True,
                program,
            )
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("assets", type=Path)
    p.add_argument("jdk", type=Path)
    p.add_argument("side", choices=["leftSmall", "rightSmall"])
    p.add_argument("step", type=float)
    p.add_argument(
        "--offset",
        type=int,
        choices=(-3, 0, 3),
        default=0,
        help="Signed optical ambiguity control in millimetres",
    )
    a = p.parse_args()
    result = run(a.assets, a.jdk, a.side, a.step, a.offset)
    label = f"offset-{a.offset:+g}mm" if a.offset else "feed"
    (HERE / f"{label}-{a.side}-{a.step:g}.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf8", newline="\n"
    )
