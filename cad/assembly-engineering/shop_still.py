"""Draw the parked shop issue using its checked orbit mesh and the maintained project renderer."""

import argparse, gzip, json, runpy, sys, tempfile
from pathlib import Path
from installed_audit import HERE, ROOT, sha


def render(folder=HERE / "issue"):
    mesh = folder / "installed.stl.gz"
    source = json.loads((folder / "orbit.json").read_bytes())
    if sha(mesh) != source["baseSha256"]:
        raise ValueError("Changed shop viewer mesh")
    sys.path.insert(0, str(ROOT / "scripts"))
    renderer = runpy.run_path(str(ROOT / "scripts/export-showcase-cad.py"))["render"]
    config = json.loads((ROOT / "cad-showcase.json").read_bytes())
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / "assembly.stl"
        path.write_bytes(gzip.decompress(mesh.read_bytes()))
        groups = [dict(color=c, triangles=n) for _, c, _, n in source["baseBatches"]]
        renderer(path, folder / "installed.png", groups, config.get("backdrop"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", nargs="?", type=Path, default=HERE / "issue")
    render(ap.parse_args().folder)
