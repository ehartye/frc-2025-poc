"""Render the README still of the current preliminary design from its published review scene.

    python scripts/preliminary-still.py . [RELEASE] [--case withdraw-park] [--pose -1]

The still is drawn from the same block-level solids and colours the review page shows, with the offscreen renderer in
export-showcase-cad.py. It needs no STEP. The image path is `still` in cad-showcase.json. Field fixtures, gauges and, in
a park case, the delivered piece are hidden. Run it after staging a draft whose geometry changed, then run
`python scripts/cad-readmes.py . --update`.
"""
import argparse
import gzip
import importlib.util
import json
import struct
from pathlib import Path
from tempfile import TemporaryDirectory

HIDDEN_CATEGORIES = ("reserved-space",)


def facets(scene, case, pose):
    """Group the shown bodies' triangles by display colour, placed by the selected pose of the case."""
    grouped, shown = {}, []
    for name in sorted(case["bodies"]):
        body, placed = scene["bodies"][name], case["bodies"][name]
        if (body["fixture"] or body["category"] in HIDDEN_CATEGORIES or name in case["removed"]
                or case["stage"] == "park" and body["role"] == "payload"):
            continue
        shown.append(name)
        m = placed["matrices"][min(pose, len(placed["matrices"]) - 1) if pose >= 0 else -1]
        mesh = scene["meshes"][placed["mesh"]]
        points = [tuple(m[r * 4] * x + m[r * 4 + 1] * y + m[r * 4 + 2] * z + m[r * 4 + 3] for r in range(3))
                  for x, y, z in mesh["v"]]
        rows = grouped.setdefault(body["color"].lower(), [])
        for a, b, c in mesh["f"]:
            rows.append((points[a], points[b], points[c]))
    return grouped, shown


def write_stl(grouped, file):
    """One binary STL with the triangles of each colour contiguous, in the order the renderer colours them."""
    groups = []
    with file.open("wb") as out:
        out.write(bytes(80) + struct.pack("<I", sum(len(rows) for rows in grouped.values())))
        for color in sorted(grouped):
            for a, b, c in grouped[color]:
                u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
                n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                size = sum(x * x for x in n) ** .5 or 1
                out.write(struct.pack("<12fH", *(x / size for x in n), *a, *b, *c, 0))
            groups.append(dict(color=color, triangles=len(grouped[color])))
    return groups


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("release", nargs="?", help="defaults to the current release in the published index")
    parser.add_argument("--case", default="withdraw-park")
    parser.add_argument("--pose", type=int, default=-1, help="pose index in the case; -1 is its final pose")
    args = parser.parse_args()
    root = args.root.resolve()
    release = args.release or json.loads((root / "docs/preliminary-design/releases.json").read_bytes())["current"]
    config = json.loads((root / "cad-showcase.json").read_text(encoding="utf-8-sig"))
    image = root / config["still"]
    scene = json.loads(gzip.decompress((root / "docs/preliminary-design/releases" / release / "review-scene.json.gz").read_bytes()))
    case = next((c for c in scene["cases"] if c["id"] == args.case), None)
    if case is None or case["status"] != "pass":
        raise SystemExit("Pose the still in a passing case of the design")
    grouped, shown = facets(scene, case, args.pose)
    spec = importlib.util.spec_from_file_location("showcase_cad", Path(__file__).with_name("export-showcase-cad.py"))
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    with TemporaryDirectory() as tmp:
        mesh = Path(tmp) / "layout.stl"
        groups = write_stl(grouped, mesh)
        tool.render(mesh, image, groups, config.get("backdrop"))
    print(f"Rendered {image.relative_to(root).as_posix()}: {len(shown)} solids of {release}, case {args.case}")
