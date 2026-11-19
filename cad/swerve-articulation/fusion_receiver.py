"""Fusion script: rebuild or inspect the Team 1780 receiver in a private copy of the full robot."""

import hashlib
import importlib.util
import json
from pathlib import Path

import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
STUDY = HERE.parents[1] / "experiments/swerve-drive/score-stroke"
JOINTS = {"middle": "slide-L-middle", "slide": "drilled-slide-L-inner"}
ANCHOR = "bumper-cover-front-red"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def occurrences(root, groups):
    requested = {name for group in JOINTS for name in groups[group]} | {ANCHOR}
    selected = {}
    wheels = {}
    pending = list(root.occurrences)
    while pending:
        occ = pending.pop()
        name = occ.component.name.split(" (")[0]
        if name.startswith(("FL - REV", "FR - REV", "RL - REV", "RR - REV")):
            wheels[name] = occ
            continue
        if name in requested:
            if name in selected:
                raise ValueError(f"Ambiguous occurrence {name}")
            selected[name] = occ
        pending.extend(occ.childOccurrences)
    if set(selected) != requested or len(wheels) != 4:
        raise ValueError(
            f"Incomplete assembly. Missing {requested - selected.keys()}, wheels {len(wheels)}"
        )
    return selected, wheels


def machine(design, selected):
    spec = importlib.util.spec_from_file_location(
        "receiver_cams", STUDY / "fusion_cam.py"
    )
    cam = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cam)
    design.userParameters.add(
        "camLaneWidth",
        core.ValueInput.createByString("10 mm"),
        "mm",
        "Width of the paired continuous underside lands",
    )
    reference = json.loads((STUDY / "fusion.json").read_bytes())["originalR4Appearance"]
    for name, expected in reference.items():
        component = selected[name].component
        body = component.bRepBodies.item(0)
        for y in (-66, 42):
            if name == "r6-receiver-insert":
                cam.prism(
                    component,
                    body,
                    y,
                    10,
                    [(1, 488), (115, 485), (118, 485), (118, 496), (1, 496)],
                    fusion.FeatureOperations.JoinFeatureOperation,
                    f"Retain billet land at Y{y}",
                )
            cam.prism(
                component,
                body,
                y,
                10,
                [(-105, 480), (115, 480), (115, 485), (1, 488), (-105, 488)],
                fusion.FeatureOperations.CutFeatureOperation,
                f"Continuous underside relief at Y{y}",
            )
        if abs(body.volume * 1000 - expected["volumeMm3"]) > 0.05:
            raise ValueError(f"Machined volume differs for {name}")


def articulate(root, groups, selected):
    anchor = selected[ANCHOR]
    anchor.isGrounded = True
    joints = {}
    for stage, reference in JOINTS.items():
        members = core.ObjectCollection.create()
        for name in groups[stage]:
            members.add(selected[name])
        group = root.rigidGroups.add(members, False)
        group.name = f"Moving receiver - {stage}"
        geom = fusion.JointGeometry.createByPoint(root.originConstructionPoint)
        params = root.asBuiltJoints.createInput(selected[reference], anchor, geom)
        params.setAsSliderJointMotion(fusion.JointDirections.XAxisJointDirection)
        joint = root.asBuiltJoints.add(params)
        joint.name = f"Receiver / {stage}"
        limit = joint.jointMotion.slideLimits
        limit.minimumValue = 0
        limit.maximumValue = 18.5 if stage == "middle" else 37
        limit.isMinimumValueEnabled = limit.isMaximumValueEnabled = True
        joints[stage] = joint
    links = joints["middle"].parentComponent.motionLinks
    relation = links.createInput(joints["middle"], joints["slide"])
    relation.valueOne = core.ValueInput.createByString("185 mm")
    relation.valueTwo = core.ValueInput.createByString("370 mm")
    links.add(relation).name = "Telescoping rack ratio"


def check(design, groups, selected, wheels):
    joints = {
        j.name: j for component in design.allComponents for j in component.asBuiltJoints
    }
    slide = joints["Receiver / slide"].jointMotion
    middle = joints["Receiver / middle"].jointMotion
    fixed = {ANCHOR: selected[ANCHOR], **wheels}
    scales = {
        **dict.fromkeys(groups["slide"], 1),
        **dict.fromkeys(groups["middle"], 0.5),
        **dict.fromkeys(fixed, 0),
    }
    parts = {**selected, **fixed}
    baseline = {}
    poses = []
    for stroke in (0, 185, 370, 0):
        slide.slideValue = stroke / 10
        import adsk

        adsk.doEvents()
        rows = {}
        for name, occ in parts.items():
            box = occ.preciseBoundingBox
            rows[name] = [
                p * 10 for p in [*box.minPoint.asArray(), *box.maxPoint.asArray()]
            ]
        if not baseline:
            baseline = rows
        worst = max(
            abs((b - a) - (scales[name] * stroke if axis in (0, 3) else 0))
            for name, row in rows.items()
            for axis, (a, b) in enumerate(zip(baseline[name], row))
        )
        poses.append(
            dict(
                strokeMm=stroke,
                middleMm=10 * middle.slideValue,
                checked=len(rows),
                maxBoundsErrorMm=worst,
            )
        )
        if abs(10 * middle.slideValue - stroke / 2) > 0.01 or worst > 0.02:
            raise ValueError(f"Incorrect linked travel at {stroke} mm: {poses[-1]}")
    return poses


def run(context):
    app = core.Application.get()
    ui = app.userInterface
    picker = ui.createFileDialog()
    picker.title = "Open the issued full R4 or its articulated successor"
    picker.filter = "Fusion archive (*.f3d)"
    if picker.showOpen() != core.DialogResults.DialogOK:
        return
    source = Path(picker.filename)
    receipt = json.loads((HERE / "native.json").read_bytes())
    actual = digest(source)
    if actual not in (receipt["baselineSha256"], receipt["archiveSha256"]):
        raise ValueError("Archive differs from the issued source or checked successor")
    groups = json.loads((HERE / "motion-groups.json").read_bytes())
    manager = app.importManager
    doc = manager.importToNewDocument(
        manager.createFusionArchiveImportOptions(str(source))
    )
    design = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
    selected, wheels = occurrences(design.rootComponent, groups)
    if actual == receipt["baselineSha256"]:
        machine(design, selected)
        articulate(design.rootComponent, groups, selected)
    rows = check(design, groups, selected, wheels)
    ui.messageBox(
        f"Checked {rows[0]['checked']} occurrences through {len(rows)} poses. "
        "The opened copy is stowed. Use File > Export to retain your edits."
    )
