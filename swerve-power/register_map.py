"""Re-root the legacy private assembly map while verifying every original source byte."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

PARENT = "cbf038a1b6e556d369d33dfe72882e21503a4f33"


def restore(repo, src, out):
    src = src.resolve()
    out.mkdir(parents=True, exist_ok=True)
    bound = json.loads(src.read_bytes())
    base = next(p for p in src.parents if (p / ".git").exists())
    for i, row in enumerate(bound["sources"]):
        target = (src.parent / row["file"]).resolve()
        if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == row["sha256"]:
            content = target.read_bytes()
        else:
            rel = target.relative_to(base).as_posix()
            advanced = repo / "experiments/six-wheel" / rel
            if (
                advanced.is_file()
                and hashlib.sha256(advanced.read_bytes()).hexdigest() == row["sha256"]
            ):
                content = advanced.read_bytes()
            else:
                content = subprocess.check_output(
                    ["git", "-C", str(repo), "show", PARENT + ":" + rel]
                )
        if hashlib.sha256(content).hexdigest() != row["sha256"]:
            raise ValueError("Source digest mismatch: " + row["id"])
        member = f"{i:02d}-{target.name}"
        (out / member).write_bytes(content)
        row["file"] = member
    (out / "selected.json").write_text(
        json.dumps(bound, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return out / "selected.json"


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("project", "map", "output"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    restore(a.project, a.map, a.output)
