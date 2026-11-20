"""Run in Fusion with a local manifest identifying project and output directories."""

import hashlib
import json
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion


def run(context):
    cfg = json.loads(Path(r"INPUT_MANIFEST_JSON").read_text(encoding="utf-8-sig"))
    root_dir, out = Path(cfg["project"]), Path(cfg["out"])
    base = root_dir / "cad/swerve-captive/Team-1780-public-captive-swerve.f3d"
    assert (
        hashlib.sha256(base.read_bytes()).hexdigest()
        == "baffdfad1d78ab9807ae3cb6f2c3e3f0e37008bf9ef7ed3f70586f9d1d1ac9e5"
    )
    app = core.Application.get()
    doc = app.importManager.importToNewDocument(
        app.importManager.createFusionArchiveImportOptions(str(base))
    )
    (out / "owned-document.txt").write_text(doc.creationId)
    try:
        design = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = design.rootComponent
        body = next(o for o in root.occurrences if o.component.name == "Fixed chassis")

        def component(name):
            o = root.occurrences.addNewComponent(core.Matrix3D.create())
            o.component.name = name
            return o

        def pt(x, y):
            return core.Point3D.create(x / 10, y / 10, 0)

        def section(o, name, x, y, z, h, outer, inner=None, circular=False):
            # A second dock uses the forward pair of rails. The retired rear installation stays reproducible.
            if front:
                x += 518
            c = o.component
            plane = c.constructionPlanes.createInput()
            plane.setByOffset(
                c.xYConstructionPlane, core.ValueInput.createByReal(z / 10)
            )
            support = c.constructionPlanes.add(plane)
            support.isLightBulbOn = False
            sk = c.sketches.add(support)
            sk.name = name + " section"
            for spec in (outer, inner):
                if spec is None:
                    continue
                if circular:
                    sk.sketchCurves.sketchCircles.addByCenterRadius(pt(x, y), spec / 20)
                else:
                    dx, dy = spec
                    sk.sketchCurves.sketchLines.addTwoPointRectangle(
                        pt(x - dx / 2, y - dy / 2), pt(x + dx / 2, y + dy / 2)
                    )
            profile = next(
                s for s in sk.profiles if s.profileLoops.count == (2 if inner else 1)
            )
            extr = c.features.extrudeFeatures.addSimple(
                profile,
                core.ValueInput.createByReal(h / 10),
                fusion.FeatureOperations.NewBodyFeatureOperation,
            )
            extr.name = name
            extr.bodies.item(0).name = name
            sk.isLightBulbOn = False

        def joint(name, one, two, direction=None, limits=None):
            g = fusion.JointGeometry.createByPoint(root.originConstructionPoint)
            inp = root.asBuiltJoints.createInput(one, two, g)
            if direction is None:
                inp.setAsRigidJointMotion()
            else:
                inp.setAsSliderJointMotion(direction)
            j = root.asBuiltJoints.add(inp)
            j.name = name
            if limits is not None:
                lim = j.jointMotion.slideLimits
                lim.minimumValue = limits[0] / 10
                lim.isMinimumValueEnabled = True
                lim.maximumValue = limits[1] / 10
                lim.isMaximumValueEnabled = True
            return j

        front = cfg.get("mount", "front") == "front"
        fixed = component("Shallow cage jack frame")
        beam = component("Cage crosshead")
        left = component("Left cage shoe")
        right = component("Right cage shoe")
        # Two columns bolt to the side frame rails. Central cavity remains available to the stowed rack.
        for sign, side in ((1, "Left"), (-1, "Right")):
            y = sign * 321
            section(
                fixed,
                side + " rail saddle",
                -224 if front else -218,
                sign * 330,
                150,
                9,
                (64 if front else 82, 40),
            )
            section(
                fixed, side + " guide column", -238, y, 159, 552, (18, 24), (12, 18)
            )
            section(fixed, side + " screw core", -213, y, 170, 530, 12, circular=True)
            section(fixed, side + " upper thrust bridge", -226, y, 700, 10, (44, 32))
            section(fixed, side + " lower thrust bridge", -226, y, 159, 11, (44, 32))
            section(
                fixed, side + " drive and pawl envelope", -188, y, 162, 90, (34, 34)
            )
            section(
                beam,
                side + " travelling nut",
                -213,
                y,
                350,
                16 if front else 32,
                28,
                16,
                circular=True,
            )
        # Two bearing shoes slide laterally after the narrow head passes the center bore.
        section(beam, "Crosshead front web", -200, 0, 350, 42, (4, 650))
        section(
            beam, "Crosshead rear web", -224, 0, 350, 42, (4, 604 if front else 620)
        )
        section(beam, "Crosshead upper flange", -212, 0, 388, 4, (20, 604))
        section(beam, "Crosshead lower flange", -212, 0, 350, 4, (20, 604))
        section(beam, "Latch stem", -218, 0, 392, 68, 21, circular=True)
        section(beam, "Shoe dovetail carrier", -218, 0, 460, 8, (23, 37))
        section(left, "Positive bearing shoe", -218, 12, 468, 8, (22, 24))
        section(right, "Negative bearing shoe", -218, -12, 468, 8, (22, 24))
        # Opposed lower pads transfer pitch moment to the plate. Upper shoes alone
        # have just 11 mm of fore-aft bearing and cannot balance the robot.
        back = component("Lower plate clamp")
        section(back, "Clamp spider", -218, 0, 438, 6, (124, 32), (24, 24))
        for sx in (-1, 1):
            section(
                back, "Clamp flank " + str(sx), -218 + sx * 55, 0, 444, 4, (14, 112)
            )
            for sy in (-1, 1):
                section(
                    back,
                    "Bearing pad " + str(sx) + " " + str(sy),
                    -218 + sx * 55,
                    sy * 47,
                    448,
                    3,
                    (14, 18),
                )
        axes = fusion.JointDirections
        joint("Jack frame to swerve chassis", fixed, body)
        joint("Lift crosshead", beam, fixed, axes.ZAxisJointDirection, (0, 333))
        joint("Spread left cage shoe", left, beam, axes.YAxisJointDirection, (0, 36))
        joint("Spread right cage shoe", right, beam, axes.YAxisJointDirection, (-36, 0))
        joint("Preload cage plate", back, beam, axes.ZAxisJointDirection, (0, 14))
        mgr = design.exportManager
        dest = out / "Team-1780-shallow-cage-study.f3d"
        if not mgr.execute(mgr.createFusionArchiveExportOptions(str(dest))):
            raise RuntimeError("Fusion export failed")
        (out / "build.json").write_text(
            json.dumps(
                {
                    "file": dest.name,
                    "sha256": hashlib.sha256(dest.read_bytes()).hexdigest(),
                    "design": root.name,
                    "occurrences": root.occurrences.count,
                    "sourceSha256": hashlib.sha256(base.read_bytes()).hexdigest(),
                }
            )
        )
    finally:
        doc.close(False)
