"""Install the two moving yokes into a copy of the restricted articulated R4."""

import adsk.core as core
import adsk.fusion as fusion


def occurrences(root, names):
    result = {}
    for occ in root.allOccurrences:
        key = occ.component.name.split(" (")[0]
        if key in names:
            if key in result:
                raise ValueError(f"Duplicate part {key}")
            result[key] = occ
    if result.keys() != names:
        raise ValueError(f"Missing {names - result.keys()}")
    return result


def install(design, kit):
    root = design.rootComponent
    anchor = "drilled-slide-L-inner"
    parts = occurrences(root, {anchor} | {v["id"] for v in kit["parts"]})
    travel = {}
    for stage, sign in (("slide", 0), ("left", -1), ("right", 1)):
        members = [
            r["id"]
            for r in kit["parts"]
            if (
                r["frame"] == "slide"
                if stage == "slide"
                else r["frame"] == "yoke" and "-" + stage + "-" in r["id"]
            )
        ]
        coll = core.ObjectCollection.create()
        for key in members:
            coll.add(parts[key])
        root.rigidGroups.add(coll, False).name = "Captive rack " + stage
        geo = fusion.JointGeometry.createByPoint(root.originConstructionPoint)
        settings = root.asBuiltJoints.createInput(parts[members[0]], parts[anchor], geo)
        if not sign:
            settings.setAsRigidJointMotion()
            root.asBuiltJoints.add(settings).name = "Capture housings to slide"
            continue
        settings.setAsSliderJointMotion(fusion.JointDirections.YAxisJointDirection)
        joint = root.asBuiltJoints.add(settings)
        joint.name = stage.capitalize() + " yoke close"
        limits = joint.jointMotion.slideLimits
        limits.minimumValue = min(0, sign * 3.5)
        limits.maximumValue = max(0, sign * 3.5)
        limits.isMinimumValueEnabled = limits.isMaximumValueEnabled = True
        travel[stage] = joint
    p = root.motionLinks.createInput(travel["left"], travel["right"])
    p.valueOne = core.ValueInput.createByString("-35 mm")
    p.valueTwo = core.ValueInput.createByString("35 mm")
    root.motionLinks.add(p).name = "Opposing bore fingers"
    probes(design, kit)


def probes(design, kit):
    root = design.rootComponent
    names = {r["id"] for r in kit["parts"]}
    parts = occurrences(root, names)
    for side, limit in (("left", 8), ("right", -8)):
        ref = parts["retainer-" + side + "-probe-guide"]
        tip = parts["retainer-" + side + "-rim-probe"]
        geom = fusion.JointGeometry.createByPoint(root.originConstructionPoint)
        settings = root.asBuiltJoints.createInput(tip, ref, geom)
        settings.setAsSliderJointMotion(fusion.JointDirections.YAxisJointDirection)
        joint = root.asBuiltJoints.add(settings)
        joint.name = side.capitalize() + " free rim probe"
        limits = joint.jointMotion.slideLimits
        limits.minimumValue = min(0, limit / 10)
        limits.maximumValue = max(0, limit / 10)
        limits.isMinimumValueEnabled = limits.isMaximumValueEnabled = True


def check(design, kit):
    import adsk

    root = design.rootComponent
    parts = occurrences(
        root, {"drilled-slide-L-inner"} | {r["id"] for r in kit["parts"]}
    )
    joints = {j.name: j for c in design.allComponents for j in c.asBuiltJoints}
    wheels = [
        o
        for o in root.allOccurrences
        if o.component.name.startswith(("FL - REV", "FR - REV", "RL - REV", "RR - REV"))
    ]
    if len(wheels) != 4:
        raise ValueError("Expected all four genuine supplier modules")
    parts.update({o.fullPathName: o for o in wheels})
    capture = lambda: {
        n: o.preciseBoundingBox.minPoint.asArray() for n, o in parts.items()
    }
    original = capture()
    rows = []
    byid = {r["id"]: r for r in kit["parts"]}
    for x, close in ((0, 0), (0, 35), (185, 35), (370, 35), (370, 0), (0, 0)):
        joints["Receiver / slide"].jointMotion.slideValue = x / 10
        joints["Left yoke close"].jointMotion.slideValue = -close / 10
        adsk.doEvents()
        observed = capture()
        errors = {}
        for n in parts:
            pos = [(b - a) * 10 for a, b in zip(original[n], observed[n])]
            exp = [0 if n in {v.fullPathName for v in wheels} else x, 0, 0]
            if n in byid and byid[n]["frame"] in ("yoke", "probe"):
                exp[1] = -close if "-left-" in n else close
            errors[n] = max(abs(a - b) for a, b in zip(pos, exp))
        worst = max(errors, key=errors.get)
        rows.append(
            dict(
                strokeMm=x,
                closureMm=close,
                checked=len(parts),
                maximumErrorMm=errors[worst],
                worstPart=worst,
            )
        )
        if errors[worst] > 0.02:
            raise ValueError("Unexpected motion " + str(rows[-1]))
    return dict(poses=rows, wheelModules=[v.component.name for v in wheels])
