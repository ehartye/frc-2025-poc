"""Create native steering, rolling, rack and mirrored capture joints on the project-owned Fusion exchange."""

import math
import adsk.core as core
import adsk.fusion as fusion


def option_mm(value):
    return core.ValueInput.createByString(f"{value} mm")


def assemble(design):
    root = design.rootComponent
    groups = {o.component.name: o for o in root.occurrences}
    for name, occ in groups.items():
        comp = occ.component
        if comp.rigidGroups.count == 0 and comp.occurrences.count > 1:
            items = core.ObjectCollection.create()
            for leaf in comp.occurrences:
                items.add(leaf)
            comp.rigidGroups.add(items, False).name = "Assembled " + name
        comp.occurrences.item(0).isGroundToParent = True
    base = groups["Fixed chassis"]
    base.isGroundToParent = True
    refs = {}
    lookup = {j.name: j for j in root.asBuiltJoints}

    def point(occ, x, y, z):
        if z not in refs:
            plane = root.xYConstructionPlane
            if z:
                prm = root.constructionPlanes.createInput()
                prm.setByOffset(plane, option_mm(z))
                plane = root.constructionPlanes.add(prm)
                plane.name = "Wheel axle elevation"
                plane.isLightBulbOn = False
            sk = root.sketches.add(plane)
            sk.name = "Swerve kingpins" if z == 0 else "Wheel roll axes"
            sk.isLightBulbOn = False
            refs[z] = sk
        return fusion.JointGeometry.createByPoint(
            refs[z].sketchPoints.add(core.Point3D.create(x / 10, y / 10, 0))
        )

    origin = fusion.JointGeometry.createByPoint(root.originConstructionPoint)

    def joint(name, a, b, mode, axis=None, geo=None):
        if name in lookup:
            return lookup[name]
        p = root.asBuiltJoints.createInput(a, b, geo or origin)
        if mode == "rigid":
            p.setAsRigidJointMotion()
        elif mode == "revolute":
            p.setAsRevoluteJointMotion(axis)
        else:
            p.setAsSliderJointMotion(axis)
        j = root.asBuiltJoints.add(p)
        j.name = name
        lookup[name] = j
        return j

    axes = fusion.JointDirections
    for corner, x, y in [
        ("FL", 282, 277),
        ("FR", 282, -277),
        ("RL", -282, 277),
        ("RR", -282, -277),
    ]:
        mount, fork, tread = [
            groups[corner + " " + n] for n in ("module base", "steering fork", "tread")
        ]
        joint(corner + " module mounting", mount, base, "rigid")
        joint(
            corner + " steer",
            fork,
            mount,
            "revolute",
            axes.ZAxisJointDirection,
            point(fork, x, y, 0),
        )
        joint(
            corner + " roll",
            tread,
            fork,
            "revolute",
            axes.YAxisJointDirection,
            point(tread, x, y, 38.1),
        )

    def slide(name, one, two, axis):
        return joint(name, groups[one], groups[two], "slider", axis)

    middle = slide(
        "Receiver middle", "Middle slide", "Fixed chassis", axes.XAxisJointDirection
    )
    outer = slide(
        "Receiver outer", "Outer slide", "Fixed chassis", axes.XAxisJointDirection
    )
    left = slide(
        "Left yoke close", "Left capture yoke", "Outer slide", axes.YAxisJointDirection
    )
    right = slide(
        "Right yoke close",
        "Right capture yoke",
        "Outer slide",
        axes.YAxisJointDirection,
    )
    demo = slide(
        "CORAL feed and exit",
        "Demonstration CORAL",
        "Outer slide",
        axes.XAxisJointDirection,
    )

    # Each joint's positive sense is measured on its own moving occurrence. Set
    # the limits and links using that actual motion, including mirrored sliders.
    def sense(j, occ, coordinate):
        before = occ.transform2.translation.asArray()
        j.jointMotion.slideValue = 0.1
        after = occ.transform2.translation.asArray()
        j.jointMotion.slideValue = 0
        delta = (after[coordinate] - before[coordinate]) * 10
        if abs(abs(delta) - 1) > 0.002:
            raise ValueError(f"Slider motion disagrees {j.name}: {delta}")
        return 1 if delta > 0 else -1

    def limits(j, travel):
        lim = j.jointMotion.slideLimits
        lim.minimumValue = min(0.0, travel / 10)
        lim.maximumValue = max(0.0, travel / 10)
        lim.isMinimumValueEnabled = lim.isMaximumValueEnabled = True

    def link(a, b, one, two, name):
        if any(v.name == name for v in root.motionLinks):
            return
        opt = root.motionLinks.createInput(a, b)
        opt.valueOne = option_mm(one)
        opt.valueTwo = option_mm(two)
        root.motionLinks.add(opt).name = name

    ds = sense(middle, groups["Middle slide"], 0)
    os = sense(outer, groups["Outer slide"], 0)
    ls = sense(left, groups["Left capture yoke"], 1)
    rs = sense(right, groups["Right capture yoke"], 1)
    ps = sense(demo, groups["Demonstration CORAL"], 0)
    for j, end in [
        (middle, 185 * ds),
        (outer, 370 * os),
        (left, -35 * ls),
        (right, 35 * rs),
        (demo, 180 * ps),
    ]:
        limits(j, end)
    link(middle, outer, 185 * ds, 370 * os, "Half travel middle stage")
    link(left, right, -35 * ls, 35 * rs, "Opposing bore fingers")
    return dict(middle=ds, outer=os, left=ls, right=rs, coral=ps)
