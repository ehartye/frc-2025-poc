"""Replay the complete shop stereo study from hash-registered external inputs."""

import argparse
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def run(name, *args):
    subprocess.run([sys.executable, str(HERE / name), *map(str, args)], check=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in (
        "artifacts",
        "lamp",
        "field",
        "output",
        "jdk",
        "gradle",
        "plugin",
        "receiver_map",
    ):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    work = a.output.resolve()
    marked = work / "marked"
    raw = a.artifacts / "relieved-match"
    external = a.artifacts / "camera-disconnects"
    run("fixed_locator.py", external)
    run("interfaces.py", external)
    run("scenes.py", a.artifacts, a.lamp, a.field, work)
    run("stroke.py", a.artifacts, a.lamp, work)
    run("shelf_marks.py", work, marked)
    for piece in ("coral-A", "coral-B"):
        run("exposures_check.py", marked, raw, marked, piece, "moving", "--occlusion")
        run("exposures_check.py", marked, raw, marked, piece, "fixed", "--critical")
        run("exposures_check.py", marked, raw, marked, piece, "seat")
    for direction, depth in (("front", 0.75), ("rear", -0.75)):
        dest = work / ("fixed-" + direction)
        run("scenes.py", a.artifacts, a.lamp, a.field, dest, "--fixed-depth", depth)
        for piece in ("coral-A", "coral-B"):
            for head in ("fixed", "seat"):
                run("exposures_check.py", dest, raw, dest, piece, head, "--critical")
        run(
            "exposures_check.py",
            marked,
            raw,
            marked,
            "coral-B",
            "moving",
            "--origin",
            direction,
            "--critical",
        )
    for control in ("stale-roof", "lost-eye", "swapped-eyes"):
        run(
            "exposures_check.py",
            marked,
            raw,
            marked,
            "coral-B",
            "moving",
            "--control",
            control,
            "--critical",
        )
    run("field_roof.py", marked, raw, a.field)
    run("evidence.py", work, raw)
    run("controller.py", marked, raw, work / "hal", a.jdk, a.gradle)
    run("accounting.py", a.plugin, a.receiver_map, work / "maps")
    run("test_selected_optics.py")
