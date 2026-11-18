"""Materialize the exact four reviewed red STEP exports from the tracked blue assembly."""
import argparse
import hashlib
import json
from pathlib import Path
from shutil import copyfile
from zipfile import ZipFile

HERE = Path(__file__).resolve().parent
KIT = HERE / "red-kit"
BLUE = HERE.parent / "design" / "competition-package" / "poses"


def materialize(output, blue=BLUE):
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((KIT / "manifest.json").read_text())
    with ZipFile(KIT / "line-patches.zip") as archive:
        for pose, expected in manifest.items():
            original = (blue / (pose + ".step")).read_bytes()
            if hashlib.sha256(original).hexdigest() != expected["blue_sha256"]:
                raise ValueError(f"{pose}: blue source differs from reviewed bytes")
            lines = original.splitlines(keepends=True)
            for index, replacement in json.loads(archive.read(pose + ".json")):
                lines[index] = replacement.encode("ascii")
            result = b"".join(lines)
            if hashlib.sha256(result).hexdigest() != expected["red_sha256"]:
                raise ValueError(f"{pose}: failed to reproduce reviewed red export")
            (output / (pose + ".step")).write_bytes(result)
    for name in ("part-frames.json", "rotation-bounds.json", "review.json", "source-register.json"):
        copyfile(KIT / name, output / name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--blue", type=Path, default=BLUE)
    args = parser.parse_args()
    materialize(args.output, args.blue)
