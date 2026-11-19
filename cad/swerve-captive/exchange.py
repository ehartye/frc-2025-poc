"""Assemble the public robot from the issued first-party exchange sources and group it for Fusion."""

from argparse import ArgumentParser
from collections import defaultdict
from pathlib import Path
import hashlib
import importlib.util
import json
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SW = ROOT / "experiments/swerve-drive"
spec = importlib.util.spec_from_file_location(
    "public_swerve_source", HERE.parent / "full-swerve-r3/build.py"
)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
from OCP.TopLoc import TopLoc_Location
from OCP.TDataStd import TDataStd_Name
from OCP.TCollection import TCollection_ExtendedString


def title(label, value):
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(value))


def assigned(key, groups):
    for corner in ("FL", "FR", "RL", "RR"):
        if key.startswith(corner + "-display-"):
            if key.endswith("-tread"):
                return corner + " tread"
            if "-fork-" in key:
                return corner + " steering fork"
            return corner + " module base"
    for name in ("slide", "middle"):
        if key in groups[name]:
            return "Outer slide" if name == "slide" else "Middle slide"
    return "Fixed chassis"


def build(out):
    robot = HERE.parent / "full-swerve-r4/full-robot.step"
    cam = SW / "score-stroke/cam-lands.step"
    yokes = SW / "retention/open-yokes.step"
    appearance = base.read(robot.with_name("appearances.json"))
    registers = base.read(yokes.with_suffix(".json"))
    if (
        base.sha(robot) != appearance["sourceSha256"]
        or base.sha(cam) != base.read(cam.with_name("cam.json"))["candidateStepSha256"]
        or base.sha(yokes) != registers["stepSha256"]
    ):
        raise ValueError("Changed released CAD source")
    roles = {row["id"]: row["role"] for row in appearance["solids"]}
    colors = {
        row["role"]: row["color"]
        for row in base.read(ROOT / "docs/robot/cad/receipt.json")["appearances"][
            "groups"
        ]
    }
    groups = base.read(HERE.parent / "swerve-articulation/motion-groups.json")
    parts = base.named(robot)
    changes = base.named(cam)
    if len(parts) != 975 or len(changes) != 2 or set(changes) - set(parts):
        raise ValueError("Unexpected public robot roster")
    parts.update(changes)
    classification = {k: assigned(k, groups) for k in parts}
    yoke_shapes = base.named(yokes)
    for row in registers["parts"]:
        name = row["id"]
        parts[name] = yoke_shapes[name]
        if row["frame"] == "slide":
            classification[name] = "Outer slide"
        else:
            classification[name] = (
                "Left capture yoke" if "-left-" in name else "Right capture yoke"
            )
        roles[name] = (
            "sensing"
            if "probe" in name
            else ("shaft" if "tube" in name else "receiver")
        )
    from mechanical import cut, cylinder

    demo = "Practice CORAL tube"
    parts[demo] = base.cad.brep(
        cut(
            cylinder(75, -150.8125, 557.1145, (0, 1, 0), 57.15, 301.625),
            cylinder(75, -151, 557.1145, (0, 1, 0), 50.8, 303.0),
        )
    )
    classification[demo] = "Demonstration CORAL"
    roles[demo] = "game-piece"
    colors["game-piece"] = "#E6B450"
    doc, table = base.cad.xcaf_doc()
    palette = base.XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    owner = table.NewShape()
    title(owner, "Team 1780 public captive swerve")
    roster = defaultdict(list)
    for k in parts:
        roster[classification[k]].append(k)
    for group, names in sorted(roster.items()):
        label = table.NewShape()
        title(label, group)
        ref = table.AddComponent(owner, label, TopLoc_Location())
        title(ref, group)
        for key in sorted(names):
            component = table.AddShape(parts[key], False)
            title(component, key)
            ref = table.AddComponent(label, component, TopLoc_Location())
            title(ref, key)
            color = bytes.fromhex(colors[roles[key]][1:])
            c = base.Quantity_Color(*(i / 255 for i in color), base.Quantity_TOC_sRGB)
            palette.SetColor(component, c, base.XCAFDoc_ColorGen)
    table.UpdateAssemblies()
    writer = base.STEPCAFControl_Writer()
    writer.SetColorMode(True)
    writer.SetNameMode(True)
    if (
        not writer.Transfer(doc, base.cad.STEPControl_AsIs)
        or writer.Write(str(out)) != base.cad.IFSelect_RetDone
    ):
        raise ValueError("Failed writing grouped source")
    return dict(
        source=base.sha(robot),
        cam=base.sha(cam),
        retainer=base.sha(yokes),
        exchange=base.sha(out),
        robotSolids=len(parts) - 1,
        demonstrationSolids=1,
        total=len(parts),
        groups=dict(roster),
    )


if __name__ == "__main__":
    a = ArgumentParser(description=__doc__)
    a.add_argument("output", type=Path)
    args = a.parse_args()
    args.output.parent.mkdir(exist_ok=True, parents=True)
    data = build(args.output)
    args.output.with_suffix(".json").write_text(
        json.dumps(data, indent=2) + "\n", encoding="utf8"
    )
