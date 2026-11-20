"""Build the orbit mesh from the five Fusion groups and the film's measured stroke."""

import gzip
import hashlib
import json
import struct
import tempfile
from pathlib import Path

from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

from installed import HERE, ROOT, load_parts
from lift import stroke_state


def phase(t):
    def ramp(a, b):
        return min(1, max(0, (t - a) / (b - a)))

    top = 133.1 + 328 / 60
    seat = top + 0.6 + 0.43
    locked = seat + 0.6
    lift = 319.4 * ramp(locked, locked + 319.4 / 40)
    return {
        "approach": -650 * (1 - ramp(130, 132.6)),
        "top": top,
        "seat": seat,
        "locked": locked,
        "spread": 36 * ramp(top, top + 0.6),
        "crosshead": 328 * ramp(133.1, top) - 8.6 * ramp(top + 0.6, seat) - lift,
        "clamp": 13.584 * ramp(seat, locked),
        "lift": lift,
        "tilt": stroke_state(lift, (0, 0, 0.3))["tiltDeg"],
    }


def cylinder(x, y, z, radius, height):
    axis = gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeCylinder(axis, radius, height).Shape()


def field_reference():
    # Same project-drawn tubes and annular plates as the explanatory film.
    for x in (-76.2, 76.2):
        for y in (-76.2, 76.2):
            yield cylinder(300 + x, y, 765.175, 16.7, 609.6)
    for z in (785.7, 1374.8):
        yield BRepAlgoAPI_Cut(
            cylinder(300, 0, z - 1.708, 94, 3.416), cylinder(300, 0, z - 2, 38.1, 4)
        ).Shape()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def build():
    native, parts = load_parts(HERE)
    groups = {
        "Shallow cage jack frame": "#77899A",
        "Cage crosshead": "#D47D3D",
        "Left cage shoe": "#647181",
        "Right cage shoe": "#647181",
        "Lower plate clamp": "#007F78",
        "Illustrative cage": "#DEC691",
    }
    triangles = bytearray()
    added = []
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "part.stl"
        for group, color in groups.items():
            shapes = (
                field_reference()
                if group == "Illustrative cage"
                else [s for g, s in parts.values() if g == group]
            )
            start = len(triangles)
            for shape in shapes:
                mesh = BRepMesh_IncrementalMesh(shape, 0.18, False, 0.18, False)
                mesh.Perform()
                writer = StlAPI_Writer()
                writer.ASCIIMode = False
                if not writer.Write(shape, str(path)):
                    raise ValueError(group)
                data = path.read_bytes()
                assert len(data) == 84 + 50 * struct.unpack_from("<I", data, 80)[0]
                triangles.extend(data[84:])
            added.append([group, color, (len(triangles) - start) // 50])
    raw = bytes(80) + struct.pack("<I", len(triangles) // 50) + triangles
    kit = gzip.compress(raw, compresslevel=9, mtime=0)
    (HERE / "climb-kit.stl.gz").write_bytes(kit)
    base = json.loads((ROOT / "cad/swerve-captive/motion-ranges.json").read_bytes())
    palette = json.loads((ROOT / "cad/component-palette.json").read_bytes())["colors"]
    compressed = (ROOT / "docs/robot/cad/assembly.stl.gz").read_bytes()
    assert sha(compressed) == base["mesh"]
    poses = []
    for i in range(209):
        p = phase(130 + i / 8)
        poses.append(
            [
                round(p[key], 6)
                for key in ("approach", "spread", "crosshead", "clamp", "lift", "tilt")
            ]
        )
    record = {
        "fusion": native["sha256"],
        "parentFusion": base["archive"],
        "sourceSteps": {r["step"]: r["sha256"] for r in native["added"].values()},
        "baseSha256": base["mesh"],
        "kitSha256": sha(kit),
        "baseBatches": [
            [
                role,
                palette.get(role, "#E6B450")
                if group == "Demonstration CORAL"
                else palette[role],
                group,
                count,
            ]
            for role, group, count in base["batches"]
        ],
        "kitBatches": added,
        "poses": poses,
    }
    (HERE / "orbit.json").write_text(
        json.dumps(record, separators=(",", ":")) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    build()
