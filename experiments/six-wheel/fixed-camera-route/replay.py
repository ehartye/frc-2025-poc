"""Apply the existing stereo, field and packet policies to the guarded harness scenes."""

import argparse
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def run(a):
    images = a.out / "marked"
    raw = a.artifacts / "relieved-match"
    source = a.artifacts / "integrated-slide/issue/marked"

    def call(file, *args):
        subprocess.run([sys.executable, str(file), *map(str, args)], check=True)

    if a.stage == "images":
        call(HERE / "visibility.py", a.bounds, source, images)
        for piece in ("coral-A", "coral-B"):
            for head, options in (
                ("seat", []),
                ("fixed", ["--critical"]),
                ("moving", ["--occlusion"]),
            ):
                call(
                    ROOT / "selected-optics/exposures_check.py",
                    images,
                    raw,
                    images,
                    piece,
                    head,
                    *options,
                )
        for flag in ("lost-eye", "swapped-eyes"):
            call(
                ROOT / "selected-optics/exposures_check.py",
                images,
                raw,
                images,
                "coral-B",
                "moving",
                "--critical",
                "--control",
                flag,
            )
        call(HERE / "sightlines.py", a.bounds, raw, source, images, a.out / "sightlines.json")
    else:
        call(
            ROOT / "integrated-slide/observations.py", a.stage, a.out, raw, a.field, a.jdk, a.gradle
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage", choices=["images", "field", "packets"])
    for s in ("artifacts", "bounds", "out", "field", "jdk", "gradle"):
        p.add_argument(s, type=Path)
    a = p.parse_args()
    packages = a.artifacts / "sector-alignment/packages"
    if packages.is_dir():
        os.environ["PYTHONPATH"] = os.pathsep.join(
            filter(None, [str(packages), os.environ.get("PYTHONPATH")])
        )
    run(a)
