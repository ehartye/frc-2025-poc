"""Author, export or independently reopen the material-assigned shallow-cage alternate in Fusion."""

from pathlib import Path
import csv
import hashlib
import json
import adsk
import adsk.core as core
import adsk.fusion as fusion

CONFIGURATION = "fixed-low-captive-shallow-service-1"
FILENAME = "Team-1780-engineering-shallow.f3d"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open(encoding="utf8", newline="") as file:
        return list(csv.DictReader(file))


def key_for(occ, body):
    return body.name if body.name != "Body1" else occ.component.name


def physical_observations(d):
    physical = []
    for occ in d.rootComponent.allOccurrences:
        for body in occ.bRepBodies:
            n = body.nativeObject or body
            attrs = {
                i.name: i.value
                for i in n.attributes
                if i.groupName == "Team 1780 engineering"
            }
            props = n.getPhysicalProperties(
                fusion.CalculationAccuracy.VeryHighCalculationAccuracy
            )
            proxy = body.physicalProperties
            if not attrs:
                raise ValueError(
                    "Absent saved identity " + occ.fullPathName + " " + body.name
                )
            row = dict(
                occurrence=occ.fullPathName,
                body=body.name,
                identity=attrs["identity"],
                instance=attrs["instance"] or None,
                materialId=body.material.id,
                materialName=body.material.name,
                volumeCm3=props.volume,
                massKg=props.mass,
                brepVolumeCm3=n.volume,
                defaultProxyVolumeCm3=proxy.volume,
            )
            if attrs.get("stock"):
                row["stock"] = attrs["stock"]
            bb = body.preciseBoundingBox
            row["boundsMm"] = [
                10 * v for pt in (bb.minPoint, bb.maxPoint) for v in pt.asArray()
            ]
            physical.append(row)
    return sorted(physical, key=lambda r: (r["occurrence"], r["body"]))


def acquire_shapes(app, path):
    doc = app.importManager.importToNewDocument(
        app.importManager.createSTEPImportOptions(str(path))
    )
    try:
        design = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        out = {}
        cache = fusion.TemporaryBRepManager.get()
        for occ in design.rootComponent.allOccurrences:
            for body in occ.bRepBodies:
                name = occ.component.name
                if name in out:
                    raise ValueError("Duplicate imported body " + name)
                out[name] = cache.copy(body)
        for body in design.rootComponent.bRepBodies:
            if body.name in out:
                raise ValueError(body.name)
            out[body.name] = cache.copy(body)
        return out
    finally:
        doc.close(False)


def insert(c, shape, name):
    if c.parentDesign.designType == fusion.DesignTypes.ParametricDesignType:
        f = c.features.baseFeatures.add()
        f.name = "Shop exchange " + name
        f.startEdit()
        n = c.bRepBodies.add(shape, f)
        n.name = name
        f.finishEdit()
        return f.bodies.item(0)
    n = c.bRepBodies.add(shape)
    n.name = name
    return n


def run(context):
    cfg = json.loads(Path(r"INPUT_MANIFEST_JSON").read_text(encoding="utf-8-sig"))
    root = Path(cfg["project"])
    folder = Path(cfg.get("frozenInputs", root / "cad/assembly-engineering"))
    out = Path(cfg["out"])
    app = core.Application.get()
    stage = cfg["stage"]
    rows = {r["id"]: r for r in read_csv(folder / "material-register.csv")}
    path = out / cfg.get("filename", FILENAME)
    source = root / "cad/shallow-cage/Team-1780-shallow-cage-study.f3d"
    change = json.loads((folder / "interfaces.json").read_bytes())
    if digest(source) != change["sourceArchiveSha256"]:
        raise ValueError("Initial archive changed")
    if (
        stage == "patch"
        and "parentArchiveSha256" in change
        and digest(Path(cfg["fromArchive"])) != change["parentArchiveSha256"]
    ):
        raise ValueError("Changed patch starting archive")
    if stage in ("create", "patch"):
        kit_path = folder / "station-and-jack-interfaces.step"
        if digest(kit_path) != change["outputSha256"]:
            raise ValueError("Change package mismatch")
        kit = acquire_shapes(app, kit_path)
        if set(kit) != set(change["replacement"] + change["additional"]):
            raise ValueError(("Imported changes have different IDs", list(kit)[:20]))
    doc = app.importManager.importToNewDocument(
        app.importManager.createFusionArchiveImportOptions(
            str(
                source
                if stage == "create"
                else Path(cfg["fromArchive"])
                if stage == "patch"
                else path
            )
        )
    )
    try:
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        top = d.rootComponent
        if stage in ("create", "patch"):
            old = {}
            known = set(rows) | set(change["fixedHeight"]["removed"])
            by_id = {
                hashlib.sha256(
                    ("team-1780-engineering-v1/" + k).encode()
                ).hexdigest(): k
                for k in known
            }
            for occ in top.allOccurrences:
                for b in occ.component.bRepBodies:
                    tag = b.attributes.itemByName("Team 1780 engineering", "identity")
                    if stage == "patch" and not tag:
                        raise ValueError(
                            ("Missing inherited stable ID", occ.fullPathName, b.name)
                        )
                    key = by_id[tag.value] if tag else key_for(occ, b)
                    if key in old:
                        raise ValueError(("Duplicate issued body", key))
                    old[key] = (occ, b)
            lib = app.materialLibraries.itemByName("Fusion Material Library")
            colors = {ap.name: ap for ap in d.appearances}
            fixed = next(
                o.component
                for o in top.occurrences
                if o.component.name == "Fixed chassis"
            )
            matcache = {}
            for key in change["fixedHeight"]["removed"]:
                if key in old:
                    oo, bb = old.pop(key)
                    if oo.component.bRepBodies.count > 1:
                        oo.component.features.removeFeatures.add(bb)
                    else:
                        oo.deleteMe()
            groups = {o.component.name: o for o in top.occurrences}
            moving = {}
            rotor_stems = {
                "screw core",
                "ratchet wheel",
                "upper thrust disk",
                "shoulder thrust disk",
                "M10 thrust locknut",
                "M10 tab washer",
                "shaft drive socket",
                "under thrust shaft washer",
                "over thrust shaft washer",
            }
            for side, ym in (("Left", 321), ("Right", -321)):
                for suffix, x, y in (
                    ("screw rotation", 305, ym),
                    ("pawl rotation", 335, ym - 4),
                ):
                    name = side + " " + suffix
                    if name not in groups:
                        occ = top.occurrences.addNewComponent(core.Matrix3D.create())
                        occ.component.name = name
                        groups[name] = occ
                    moving[name] = [x, y]

            def group_for(k):
                if k in change.get("frameAssignments", {}):
                    return groups[change["frameAssignments"][k]]
                for side in ("Left", "Right"):
                    if k.startswith(side + " "):
                        item = k[len(side) + 1 :]
                        if item in rotor_stems:
                            return groups[side + " screw rotation"]
                        if item == "pawl":
                            return groups[side + " pawl rotation"]
                        if item == "travelling nut":
                            return groups["Cage crosshead"]
                        if (
                            k not in old
                            and not k.startswith("Left rail")
                            and not k.startswith("Right rail")
                        ):
                            return groups["Shallow cage jack frame"]
                if (
                    "bearing shoe stop pin" in k
                    or "bearing shoe M2.5 stop locknut" in k
                    or k == "Stem antirotation spring pin"
                ):
                    return groups["Cage crosshead"]
                return None

            for i, (k, r) in enumerate(rows.items()):
                if i % 100 == 0:
                    (out / "progress.txt").write_text(str(i) + " " + k)
                saved = old.get(k)
                group = group_for(k)
                expected_stock = (
                    r["physicalDisplayMaterial"]
                    if r["kind"] in ("made-stock", "surface treatment")
                    else ""
                )
                if (
                    stage == "patch"
                    and saved
                    and k not in kit
                    and k not in cfg.get("changedMaterials", [])
                ):
                    previous = saved[1]
                    attributes = {
                        a.name: a.value
                        for a in previous.attributes
                        if a.groupName == "Team 1780 engineering"
                    }
                    if (
                        previous.material.name == r["physicalDisplayMaterial"]
                        and attributes.get("instance", "") == r["instance"]
                        and attributes.get("stock", "") == expected_stock
                    ):
                        continue
                if saved:
                    o, b = saved
                    ap = b.appearance
                    c = o.component
                    if group and group.component != c:
                        c.features.removeFeatures.add(b)
                        o = group
                        c = o.component
                        b = None
                else:
                    o = group or fixed.occurrences.addNewComponent(
                        core.Matrix3D.create()
                    )
                    c = o.component
                    if not group:
                        c.name = k
                    ap = next(
                        (
                            v
                            for key, v in colors.items()
                            if key in ("FRC 2025 - fastener", "FRC 2025 - plate")
                        ),
                        None,
                    )
                    b = None
                if k in kit:
                    if b is not None:
                        c.features.removeFeatures.add(b)
                    local = fusion.TemporaryBRepManager.get().copy(kit[k])
                    inverse = o.transform2.copy()
                    inverse.invert()
                    fusion.TemporaryBRepManager.get().transform(local, inverse)
                    b = insert(c, local, k)
                matname = r["physicalDisplayMaterial"]
                if stage == "patch" and matname not in matcache:
                    current = d.materials.itemByName(matname)
                    if current:
                        matcache[matname] = current
                if matname not in matcache:
                    mat = lib.materials.itemByName(matname)
                    if not mat:
                        ref = lib.materials.itemByName("Polyethylene, Low Density")
                        mat = d.materials.addByCopy(ref, matname)
                        for prop in mat.materialProperties:
                            if prop.id in ("structural_Density", "thermal_Density"):
                                target = core.FloatProperty.cast(prop)
                                if target.units != "KilogramPerCubicMeter":
                                    raise ValueError((target.units, matname))
                                target.value = float(r["nativeNominalDensityKgM3"])
                    matcache[matname] = mat
                if k in kit:
                    b.name = k
                b.material = matcache[matname]
                role = change.get("appearanceRoles", {}).get(k)
                if role:
                    ap = colors.get("FRC 2025 - " + role, ap)
                if ap:
                    b.appearance = ap
                # The independent reopener reads persistent attributes, volume and occurrence;
                # it does not infer the installed article from a public label or body order.
                identity = hashlib.sha256(
                    ("team-1780-engineering-v1/" + k).encode()
                ).hexdigest()
                for key, value in {
                    "identity": identity,
                    "instance": r["instance"],
                    "stock": expected_stock,
                }.items():
                    b.attributes.add("Team 1780 engineering", key, value)
            for name, (x, y) in moving.items():
                if top.asBuiltJoints.itemByName(name):
                    continue
                if not groups[name].component.bRepBodies.count:
                    groups[name].deleteMe()
                    continue
                sketch = top.sketches.add(top.xYConstructionPlane)
                sketch.name = name + " axis datum"
                pt = sketch.sketchPoints.add(core.Point3D.create(x / 10, y / 10, 0))
                geometry = fusion.JointGeometry.createByPoint(pt)
                jinput = top.asBuiltJoints.createInput(
                    groups[name], groups["Shallow cage jack frame"], geometry
                )
                jinput.setAsRevoluteJointMotion(
                    fusion.JointDirections.ZAxisJointDirection
                )
                top.asBuiltJoints.add(jinput).name = name
                sketch.isVisible = False
            if not d.exportManager.execute(
                d.exportManager.createFusionArchiveExportOptions(str(path))
            ):
                raise RuntimeError("Native archive export failed")
        else:
            body_rows = physical_observations(d)
            if len(body_rows) != len(rows):
                raise ValueError(("Incomplete observation", len(body_rows), len(rows)))
            report = {
                "schema": "agent-frc/native-physical-properties@1",
                "configuration": change.get("configuration", CONFIGURATION),
                "opened": True,
                "archiveSha256": digest(path),
                "nativeBodyCount": len(body_rows),
                "rawBodyTotalKg": sum(r["massKg"] for r in body_rows),
                "rootMassKg": top.physicalProperties.mass,
                "bodies": body_rows,
            }
            (out / "engineering-reopen.json").write_bytes(
                (json.dumps(report, indent=2) + "\n").encode()
            )
            joint = {}
            for j in top.asBuiltJoints:
                motion = j.jointMotion
                jm = fusion.SliderJointMotion.cast(motion)
                rotation = fusion.RevoluteJointMotion.cast(motion)
                ends = {
                    "Lift crosshead": 319.4,
                    "Spread left cage shoe": 36,
                    "Spread right cage shoe": -36,
                    "Preload cage plate": 13.584,
                }
                record = dict(
                    one=j.occurrenceOne.fullPathName,
                    two=j.occurrenceTwo.fullPathName,
                    type=motion.objectType,
                )
                joint[j.name] = record
                if not jm and not rotation:
                    continue
                end = ends.get(
                    j.name, 1 if jm else (-10 if "pawl rotation" in j.name else 10)
                )
                limits = jm.slideLimits if jm else rotation.rotationLimits
                scale = 10 if jm else 180 / 3.141592653589793
                if limits.isMinimumValueEnabled and end < limits.minimumValue * scale:
                    end = abs(end)
                if limits.isMaximumValueEnabled and end > limits.maximumValue * scale:
                    end = -abs(end)
                readings = []
                for pos in (0, end, 0):
                    if jm:
                        jm.slideValue = pos / 10
                    else:
                        rotation.rotationValue = pos * 3.141592653589793 / 180
                    adsk.doEvents()
                    readings.append(
                        dict(
                            command=pos,
                            readback=jm.slideValue * 10
                            if jm
                            else rotation.rotationValue * 180 / 3.141592653589793,
                            a=j.occurrenceOne.transform2.asArray(),
                            b=j.occurrenceTwo.transform2.asArray(),
                        )
                    )
                record["unit"] = "mm" if jm else "degree"
                record["samples"] = readings
            (out / "engineering-joints.json").write_bytes(
                (json.dumps(joint, indent=2) + "\n").encode()
            )
            view = app.activeViewport
            cam = view.camera
            cam.viewOrientation = core.ViewOrientations.IsoTopRightViewOrientation
            cam.isSmoothTransition = False
            cam.isFitView = True
            view.camera = cam
            view.refresh()
            view.saveAsImageFile(str(out / "engineering-native.png"), 1280, 850)
    finally:
        doc.close(False)
        (out / ("closed-" + stage + ".txt")).write_text(
            "Closed assigned engineering document."
        )
