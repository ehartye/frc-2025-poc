"""Tessellate only the 1183 installed student-owned solids in the parked issue."""

import argparse, gzip, json, re, struct, tempfile
from pathlib import Path
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from installed_audit import HERE, ROOT, reopen, sha
from attachments import base
from issue import rows, document
from weighing import native_site
from shop_sources import issue_shapes


def export(issue=HERE / "issue"):
    reg = json.loads((issue / "interfaces.json").read_bytes())
    source = issue / "station-and-jack-interfaces.step"
    if sha(source) != reg["outputSha256"]:
        raise ValueError("Wrong geometry issue")
    prior = {k: (g, role, s) for k, g, role, _, _, s in reopen()}
    changed = issue_shapes(issue, reg)
    selected = {k: row[2] for k, row in prior.items()} | changed
    for k in reg["fixedHeight"]["removed"]:
        selected.pop(k, None)
    native = {
        r["body"]
        if r["body"] != "Body1"
        else re.split(r":\d+\+", r["occurrence"])[-1].rsplit(":", 1)[0]: r
        for r in rows(HERE / "as-issued-native.csv")
    }
    colors = json.loads((ROOT / "cad/component-palette.json").read_bytes())["colors"]
    colors.update(mechanism="#007F78")
    jack_colors = {
        "Cage crosshead": "lift-drive",
        "Left cage shoe": "steel",
        "Right cage shoe": "steel",
        "Lower plate clamp": "mechanism",
        "Shallow cage jack frame": "frame",
    }
    batches = {}
    names = {}
    for k, s in selected.items():
        group = native_site(k, native, changed, reg.get("frameAssignments"))[0].split(
            ":1", 1
        )[0]
        role = prior[k][1] if k in prior else "fastener"
        role = reg.get("appearanceRoles", {}).get(k, role)
        if role == "jack":
            role = jack_colors[group] if group in jack_colors else "steel"
        if role not in colors:
            raise ValueError((k, role))
        batches.setdefault((role, colors[role], group), []).append(s)
        names.setdefault(group, []).append(k)
    packed = bytearray()
    meta = []
    with tempfile.TemporaryDirectory() as loc:
        path = Path(loc) / "item.stl"
        for (role, color, group), shapes in sorted(batches.items()):
            start = len(packed)
            for shape in shapes:
                mesh = BRepMesh_IncrementalMesh(shape, 0.32, False, 0.30, False)
                mesh.Perform()
                writer = StlAPI_Writer()
                writer.ASCIIMode = False
                if not writer.Write(shape, str(path)):
                    raise ValueError(group)
                buf = path.read_bytes()
                if len(buf) != 84 + 50 * struct.unpack_from("<I", buf, 80)[0]:
                    raise ValueError("Corrupted mesh")
                packed.extend(buf[84:])
            meta.append([role, color, group, (len(packed) - start) // 50])
    raw = bytes(80) + struct.pack("<I", len(packed) // 50) + packed
    out = issue / "installed.stl.gz"
    out.write_bytes(gzip.compress(raw, compresslevel=9, mtime=0))
    poses = [
        [0, 36 * max(0, (n - 80) / 20), 319.4 * min(1, n / 80), 0, 0, 0]
        for n in range(101)
    ]
    document(
        issue / "orbit.json",
        dict(
            issue=sha(source),
            baseSha256=sha(out),
            baseBatches=meta,
            installedBodies=len(selected),
            poses=poses,
            movingGroups={g: sorted(ks) for g, ks in names.items() if g in jack_colors},
            scope="Independent joint inspection poses with the chassis on its wheels. No cage, supported robot or scored climb is implied.",
        ),
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", nargs="?", type=Path, default=HERE / "issue")
    export(ap.parse_args().folder)
