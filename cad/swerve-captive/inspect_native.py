"""Independently read and actuate the saved public robot in Autodesk Fusion."""

import math
import adsk.core
import adsk.fusion


def relative(moving, reference):
    """Native row-major world matrices, first occurrence expressed in second, cm."""
    a = moving.transform2.asArray()
    b = reference.transform2.asArray()
    out = []
    for i in range(3):
        for j in range(3):
            out.append(sum(b[4 * k + i] * a[4 * k + j] for k in range(3)))
        out.append(sum(b[4 * k + i] * (a[4 * k + 3] - b[4 * k + 3]) for k in range(3)))
    return out + [0.0, 0.0, 0.0, 1.0]


def inspect(design):
    root = design.rootComponent
    occs = {}

    def capture(occ, parent=None):
        key = occ.fullPathName
        comp = occ.component
        row = dict(
            bodies=comp.bRepBodies.count,
            bodyNames=[b.name for b in comp.bRepBodies],
            baseFeatures=comp.features.baseFeatures.count,
            editableFeatures=comp.features.extrudeFeatures.count,
        )
        if parent is not None:
            row["parent"] = parent
        if key in occs:
            raise ValueError("Repeated occurrence path " + key)
        occs[key] = row
        for child in occ.childOccurrences:
            capture(child, key)

    for occ in root.occurrences:
        capture(occ)
    byname = {j.name: j for j in root.asBuiltJoints}
    rows = {}
    for name, joint in byname.items():
        motion = joint.jointMotion
        a, b = joint.occurrenceOne, joint.occurrenceTwo
        if a.fullPathName not in occs or b.fullPathName not in occs:
            raise ValueError("Missing native joint endpoint")
        tp = adsk.fusion.JointTypes
        mode = {
            tp.RigidJointType: "rigid",
            tp.RevoluteJointType: "revolute",
            tp.SliderJointType: "slider",
        }[motion.jointType]
        row = dict(
            type=mode, occurrenceOne=a.fullPathName, occurrenceTwo=b.fullPathName
        )
        if mode != "rigid":
            # Measure the physical endpoints after each real native drive command.
            if mode == "slider":
                s = motion.slideValue
                end = -25 if name == "Left yoke close" else 25
                command = lambda v: setattr(motion, "slideValue", v / 10.0)
                read = lambda: 10.0 * motion.slideValue
                initial = s * 10.0
            else:
                initial = math.degrees(motion.rotationValue)
                end = 32.0
                command = lambda v: setattr(motion, "rotationValue", math.radians(v))
                read = lambda: math.degrees(motion.rotationValue)
            samples = []
            try:
                for task in (0, end):
                    command(task)
                    adsk.doEvents()
                    samples.append(
                        dict(position=read(), relativeTransform=relative(a, b))
                    )
            finally:
                command(initial)
            row["samples"] = samples
        rows[name] = row
    # Confirm that independently driving the source actuators moves both linked members.
    motion = byname["Receiver outer"].jointMotion
    left = byname["Left yoke close"].jointMotion
    sweep = []
    for extent, closed in [(0, 0), (185, -35), (370, -35), (370, 0), (0, 0)]:
        motion.slideValue = extent / 10.0
        left.slideValue = closed / 10.0
        adsk.doEvents()
        state = {}
        for name in (
            "Receiver middle",
            "Receiver outer",
            "Left yoke close",
            "Right yoke close",
        ):
            j = byname[name]
            state[name] = relative(j.occurrenceOne, j.occurrenceTwo)
        sweep.append(dict(strokeMm=extent, leftYokeMm=closed, endpoints=state))
    return dict(
        components=occs,
        joints=rows,
        linkedSweep=sweep,
        relativeTransformFrame="occurrenceTwo",
        transformTranslationUnit="cm",
        definitionCount=design.allComponents.count,
        linkNames=[v.name for v in root.motionLinks],
    )
