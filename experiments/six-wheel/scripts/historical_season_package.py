"""Verify season-owned bytes or materialize an explicitly requested pinned upstream checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "historical/agent-frc-0.12/inventory.json"


def verify(index, root, source_paths=False):
    checked = 0
    for item in index["files"]:
        rel = item["sourcePath"] if source_paths else item.get("projectPath")
        if rel is None:
            continue
        path = (root / rel).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file():
            raise ValueError(f"Missing or invalid path: {rel}")
        data = path.read_bytes()
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError(f"Historical bytes changed: {rel}")
        checked += 1
    return checked


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args]).decode().strip()


def materialize(index, source, fetch):
    artifacts = ROOT / "artifacts"
    artifacts.mkdir(exist_ok=True)
    dest = artifacts / ("historical-agent-frc-" + index["commit"][:7])
    if dest.exists():
        raise FileExistsError(f"Existing output preserved: {dest}")
    with tempfile.TemporaryDirectory(dir=artifacts) as tmp:
        temp = Path(tmp)
        if fetch:
            source = temp / "repository"
            subprocess.run(["git", "init", "--bare", "--quiet", str(source)], check=True)
            git(source, "fetch", "--quiet", "--depth=1", index["upstream"], index["commit"])
        if git(source, "rev-parse", index["commit"]+"^{tree}") != index["tree"]:
            raise ValueError("Upstream tree identity differs from the recorded revision")
        archive = temp / "tree.zip"
        git(source, "archive", "--format=zip", "--output", str(archive), index["commit"])
        tree = temp / "source"
        tree.mkdir()
        with zipfile.ZipFile(archive) as z:
            if any(not (tree / p).resolve().is_relative_to(tree) for p in z.namelist()):
                raise ValueError("Archive path escapes the new output directory")
            z.extractall(tree)
        checked = verify(index, tree, source_paths=True)
        tree.rename(dest)
    return dest, checked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    options = parser.add_mutually_exclusive_group()
    options.add_argument("--source", type=Path, help="Existing Git repository containing the recorded commit")
    options.add_argument("--fetch", action="store_true", help="Fetch the recorded commit from the public upstream")
    args = parser.parse_args()
    index = json.loads(INDEX.read_text(encoding="utf-8"))
    print(f"{verify(index, ROOT)} project-owned operational files match the inventory.")
    if args.source or args.fetch:
        dest, checked = materialize(index, args.source, args.fetch)
        print(f"{checked} original paths verified in {dest}")


if __name__ == "__main__":
    main()
