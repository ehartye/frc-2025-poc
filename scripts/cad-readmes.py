"""Refresh or verify the shared CAD view and every project README using only Python and Git."""
import argparse
import base64
import hashlib
import html
import json
import os
import re
import subprocess
from pathlib import Path
from urllib.parse import quote

START = "<!-- cad-still -->"
END = "<!-- /cad-still -->"


def checked(path, expected):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError(f"Changed CAD evidence: {path}")
    return data


def output(file, data, update):
    previous = file.read_bytes() if file.exists() else None
    if previous == data:
        return
    if not update:
        raise ValueError(f"Stale or missing: {file}")
    file.write_bytes(data)


def run(root, update, template):
    config = json.loads((root / "cad-showcase.json").read_text(encoding="utf-8-sig"))
    if config.get("publication", {}).get("status") == "withheld":
        paths = subprocess.check_output(["git", "ls-files", "-z"], cwd=root).split(b"\0")
        for item in paths:
            if not item or Path(os.fsdecode(item)).name.lower() not in ("readme.md", "readme"):
                continue
            path = root / os.fsdecode(item)
            original = path.read_bytes()
            without_hero = re.sub(rb"<!-- cad-still -->.*?<!-- /cad-still -->\r?\n\r?\n", b"", original, flags=re.S)
            output(path, without_hero, update)
        return
    folder = root / "docs/robot/cad"
    receipt = json.loads((folder / "receipt.json").read_text(encoding="utf-8"))
    if receipt["source"] != config["source"]:
        raise ValueError("Selected CAD differs from export. Regenerate the still and mesh.")
    checked(root / config["source"], receipt["sourceSha256"])
    checked(folder / "assembly.png", receipt["imageSha256"])
    mesh = checked(folder / "assembly.stl.gz", receipt["meshSha256"])
    layout = receipt["nativeBounds"]
    if "appearances" in config or receipt.get("schema") == "agent-frc/cad-still@2":
        from cad_appearances import validated_groups
        if not config.get("appearances") or config["appearances"] != receipt.get("appearances", {}).get("file"):
            raise ValueError("Selected appearance manifest differs from the export")
        layout = dict(bounds=layout, groups=validated_groups(root, receipt, folder / "assembly.stl.gz"))
    elif receipt.get("schema") != "agent-frc/cad-still@1":
        raise ValueError("Unknown CAD receipt schema")
    if "backdrop" in config or "backdrop" in receipt:
        from cad_appearances import backdrop_colors
        backdrop = backdrop_colors(config)
        if not backdrop or backdrop != backdrop_colors(receipt):
            raise ValueError("CAD backdrop changed. Regenerate the still and mesh.")
        if isinstance(layout, list):
            layout = {"bounds": layout}
        layout["backdrop"] = backdrop
    # The orbit and all four movement chapters draw from the identical facet buffer.
    motion_dir = root / "cad/swerve-captive"
    motion = json.loads((motion_dir / "motion-ranges.json").read_bytes())
    reopen = json.loads((motion_dir / "student-reopen.json").read_bytes())
    capture = json.loads((motion_dir / "capture-track.json").read_bytes())
    film = json.loads((root / "experiments/swerve-drive/score-stroke/motion-track.json").read_bytes())
    if motion["mesh"] != receipt["meshSha256"] or motion["archive"] != capture["archive"] or motion["archive"] != reopen["sha256"]:
        raise ValueError("Moving geometry differs from the checked student archive")
    if "game-piece" not in {row[2] for row in layout["groups"]}:
        raise ValueError("The movable practice pipe is missing")
    def track(rows, note):
        return dict(rows=rows,note=note)
    neutral = [0,0,75,0,557.1145,0,1,0,0]
    layout["motion"] = dict(ranges=motion["batches"],tracks={
        "stowed":track([neutral],"Open yokes and stowed rack in the student Fusion archive."),
        "station":track([row+[0] for row in film["station"]],"Station pickup on the development approach. The pipe center and axis follow the simulated cam-tray feed."),
        "capture":track([[row[0],0,*row[2:],row[1]] for row in capture["rows"]],"The two yokes close on the same station-fed pipe. This segment starts at the first wheel service."),
        "extend":track([row+[0] for row in film["extend"]],"Separate rack-extension fixture. The receiver follows the measured cam stroke. The capture yokes are displayed open."),
        "release":track([row+[0] for row in film["release"]],"Separate discharge fixture with its own initial pipe position and speed. Field support in this fixture carries zero match delivery credit.")})
    fields = dict(motionScript=(root/"scripts/capture-motion.js").read_text(encoding="utf8"),title=html.escape(config["title"]), scope=html.escape(config["scope"]),
                  facts=f'{receipt["solids"]:,} solids · {receipt["triangles"]:,} triangles · {receipt["chordMm"]} mm chord setting',
                  bounds=json.dumps(layout), mesh=base64.b64encode(mesh).decode("ascii"))
    viewer = re.sub(r"\{\{(\w+)\}\}", lambda match: fields[match[1]], template.read_text(encoding="utf-8"))
    output(folder / "index.html", viewer.encode(), update)
    paths = subprocess.check_output(["git", "ls-files", "-co", "--exclude-standard", "-z"], cwd=root)
    names = sorted(set(n.decode() for n in paths.split(b"\0") if n and Path(n.decode()).name.lower() in ("readme.md", "readme")))
    for name in names:
        file = root / name
        if not file.exists():
            continue
        original = file.read_bytes()
        text = original.decode("utf-8-sig", errors="surrogateescape").replace("\r\n", "\n")
        text = re.sub(r"\n*" + START + ".*?" + END + r"\n*", "\n\n", text, flags=re.S)
        lines = text.splitlines(keepends=True)
        heading = next((i for i, line in enumerate(lines) if line.startswith("# ")), None)
        if heading is None:
            raise ValueError(f"README needs a title: {name}")
        def link(target):
            return quote(Path(os.path.relpath(target, file.parent)).as_posix(), safe="/.-_")
        title = config["title"]
        image = link(folder / "assembly.png")
        target = config.get("viewerUrl") or link(folder / "index.html")
        source = link(root / config["source"])
        block = (f'{START}\n[![{title}]({image})]({target})\n\n'
                 f'**Project CAD · {title}.** {config["scope"]} '
                 f'[Source STEP · {receipt["sourceSha256"][:10]}]({source}) · [Rotate the model]({target})\n{END}')
        merged = "".join(lines[:heading+1]).rstrip("\n") + "\n\n" + block + "\n\n" + "".join(lines[heading+1:]).lstrip("\n")
        if b"\r\n" in original:
            merged = merged.replace("\n", "\r\n")
        bom = b"\xef\xbb\xbf" if original.startswith(b"\xef\xbb\xbf") else b""
        output(file, bom + merged.encode(errors="surrogateescape"), update)
    print(f"Verified CAD source, mesh, still, viewer and {len(names)} README files.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--update", action="store_true")
    parser.add_argument("--template", type=Path, default=Path(__file__).with_name("cad-orbit.html"))
    args = parser.parse_args()
    run(args.root.resolve(), args.update, args.template)
