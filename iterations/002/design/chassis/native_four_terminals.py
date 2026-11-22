"""Machine the shared transverse rails and commission three duplicates of the fitted port cartridge."""
import itertools
import json
import math
from pathlib import Path
import shutil
import traceback

from adsk import core, fusion


def load_module(path):
    import types
    module = types.ModuleType(path.stem)
    module.__file__ = str(path)
    exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    return module


def drill_port_centers(stock, component, axis, start, centers, radius, travel, name):
    plane, sign = stock.plane(component, axis, start)
    sketch = component.sketches.add(plane)
    sketch.name = name
    for xyz in centers:
        sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.modelToSketchSpace(stock.point(xyz)), radius / 10)
    profiles = core.ObjectCollection.create()
    for profile in sketch.profiles:
        profiles.add(profile)
    if profiles.count != len(centers):
        raise ValueError("Unexpected hole profile count")
    operation = component.features.extrudeFeatures
    cut = operation.createInput(profiles, fusion.FeatureOperations.CutFeatureOperation)
    cut.participantBodies = [component.bRepBodies.item(0)]
    cut.setDistanceExtent(False, core.ValueInput.createByReal(travel / sign / 10))
    operation.add(cut).name = name
    sketch.isLightBulbOn = False


def run(context):
    app = core.Application.get()
    before = [doc.creationId for doc in app.documents]
    original_active = app.activeDocument
    work = Path(CONFIG["jobInputs"])
    out = Path(CONFIG["resultDirectory"])
    chassis = work / "iterations/002/design/chassis"
    stock = load_module(chassis / "native_corner.py")
    support = Path(CONFIG["plugin"])
    guard = load_module(support / "fusion_checked_placement.py")
    authority = load_module(support / "fusion_authority.py")
    donor_name = "Port terminal cartridge:1"
    report = dict(before=before, stages=[], status="running")

    def save(stage):
        report["stage"] = stage
        (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")

    def reopen(path):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))

    def records(design):
        rows = stock.world_body_rows(design)
        keyed = {(r["path"], r["name"]): r for r in rows}
        if len(keyed) != len(rows):
            raise ValueError("Duplicate installed article identity")
        return keyed

    try:
        if before or original_active:
            raise ValueError("Require Fusion Home at acquisition")
        NATIVE_JOB["verifyDependencies"]()
        register = authority.register_path(work)
        lineage = register.parent / "issue-004/lineage.json"
        start = authority.verify(work, register, lineage)
        plan = json.loads((chassis / "other-terminal-fit.json").read_bytes())
        if plan["authority"] != start or plan["totalProposedSolids"] != 552 or any(t["blocked"] for t in plan["targets"].values()):
            raise ValueError("Station inputs differ from the source and clearance screen")
        report["start"] = start
        doc = reopen(authority.resolve(work, authority.load(lineage)["archive"]))
        des = stock.design_of(doc)
        old = records(des)
        root = des.rootComponent
        front = root.occurrences.itemByName("Front transverse socket rail:1")
        if len(old) != 504 or front.component.bRepBodies.count != 1:
            raise ValueError("Unexpected active mounting stock")
        if max(abs(v - w) for v, w in zip(front.transform2.asArray(), core.Matrix3D.create().asArray())) > 1e-8:
            raise ValueError("Unexpected native material coordinate system for front rail")
        rail = front.component
        z = 94.63336287672979
        drill_port_centers(stock, rail, 2, z + .2, [(333.9375, v, z + .2) for v in (-65, -35, 35, 65)],
                           6.4, -12.2, "Ream four upper wall cartridge shoulders")
        drill_port_centers(stock, rail, 0, 323.1, [(323.1, v, z - 3.175 - 7.1) for v in (-50, 50)],
                           2.6, 3.5, "Drill two inboard keeper holes")
        des.computeAll()
        parent = out / "machined-504.f3d"
        if parent.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(parent))):
            raise ValueError("Could not preserve the machined parent")
        doc.close(False)
        doc = reopen(parent)
        current = records(stock.design_of(doc))
        if set(current) != set(old):
            raise ValueError("Machining changed the installed article roster")
        changes = {}
        for key, row in current.items():
            other = old[key]
            delta = other["volumeMm3"] - row["volumeMm3"]
            if max(abs(a-b) for a,b in zip(row["boundsMm"], other["boundsMm"])) > .003 or row["material"] != other["material"]:
                raise ValueError(f"Original geometry shifted at {key}")
            if abs(delta) > .01:
                changes[row["path"]] = delta
        expected = plan["targets"]["front"]["additionalRailRemovalMm3"]
        if set(changes) != {"Front transverse socket rail:1", "Front transverse socket rail:2"} or any(abs(delta-expected) > .02 for delta in changes.values()):
            raise ValueError(f"Wrong machined rail volume {changes}")
        doc.close(False)
        report["coldRailRemovalMm3"] = changes
        save("Machined two transverse rails")
        hosts = ["Port longitudinal socket rail:2", "Front transverse socket rail:1", "Front transverse socket rail:2"]
        placements = {}
        for number, (direction, host) in enumerate(zip(("starboard", "front", "rear"), hosts), 2):
            row = plan["targets"][direction]
            a = math.radians(row["rotationDeg"])
            c, s = round(math.cos(a)), round(math.sin(a))
            pose = stock.matrix([[c, -s, 0, row["shiftXmm"]], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            target = "Port terminal cartridge:" + str(number)
            def place(design):
                root = design.rootComponent
                src = root.occurrences.itemByName(donor_name)
                occurrence = root.occurrences.addExistingComponent(src.component, pose)
                if occurrence.fullPathName != target:
                    raise ValueError("Unexpected purchased/stock assembly identity")
                if design.snapshots.hasPendingSnapshot:
                    design.snapshots.add()
                joint = root.asBuiltJoints.createInput(occurrence, root.occurrences.itemByName(host), None)
                joint.setAsRigidJointMotion()
                root.asBuiltJoints.add(joint).name = direction + " cartridge on four upper wall pilots"
            nxt = out / (direction + "-installed.f3d")
            record = guard.checked_placement(app, parent, stock.sha(parent), nxt,
                dict(occurrence=target, sourceOccurrence=donor_name, worldTransformCm=list(pose.asArray())), place)
            report["stages"].append(dict(station=direction, parentSha256=stock.sha(parent), guard=record))
            save("Guard " + direction)
            if record["status"] != "pass":
                raise ValueError("Native placement held " + direction)
            parent = nxt
            placements[target] = pose
        doc = reopen(parent)
        des = stock.design_of(doc)
        rows = records(des)
        if len(rows) != 552:
            raise ValueError("Incomplete final mounted assembly")
        report["nativePlacements"] = {}
        for name, pose in placements.items():
            witnesses = []
            for (path, article), row in old.items():
                if not path.startswith(donor_name + "+"):
                    continue
                at = rows[(name + path[len(donor_name):], article)]
                box = row["boundsMm"]
                corners = [stock.point(p) for p in itertools.product(*[(box[i], box[i+3]) for i in range(3)])]
                for vertex in corners:
                    vertex.transformBy(pose)
                expected = [extreme(p.asArray()[i]*10 for p in corners) for extreme in (min,max) for i in range(3)]
                error = max(abs(a-b) for a,b in zip(expected,at["boundsMm"]))
                if error > .003 or abs(row["volumeMm3"]-at["volumeMm3"]) > .015 or row["material"] != at["material"]:
                    raise ValueError("Rigid installation changed " + at["path"])
                witnesses.append(dict(path=at["path"], article=article, maxBoundErrorMm=error, material=at["material"]))
            if len(witnesses) != 16:
                raise ValueError("Incomplete cartridge occurrence registration")
            report["nativePlacements"][name] = witnesses
        report["coldBodyCount"] = len(rows)
        doc.close(False)
        mirror = out / "mirror"
        for name in ("iterations/active.json", "iterations/001/iteration.json", "iterations/001/retrospective.json",
                     "iterations/seals/001.json", "iterations/002/iteration.json"):
            target = mirror / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(work / name, target)
        dest = "iterations/002/design/cad/private/terminals-552.f3d"
        (mirror / dest).parent.mkdir(parents=True)
        shutil.copyfile(parent, mirror / dest)
        fresh = authority.register_path(mirror)
        spec = authority.load(register)
        spec.update(configuration="Four parked SDS L2 modules and four locally bolted 16-article terminal cartridges",
                    revision="002-four-terminals-552-r5", source=dict(kind="offline",archive=dict(file=dest,sha256=stock.sha(parent))),
                    basis="Guarded original port and three exact rigid duplicate cartridges, matched shared rail machining, and independent cold material registration. Upper, perimeter, preloads and original swerve internals remain open.")
        fresh.write_text(json.dumps(spec,indent=2)+"\n")
        issue = fresh.parent / "issue-005"
        report["lineage"] = authority.export_issue(app, mirror, fresh, issue)
        report["verifiedAuthority"] = authority.verify(mirror, fresh, issue / "lineage.json")
        report["status"] = "cold four-terminal lower"
    except Exception:
        report.update(status="held", error=traceback.format_exc())
    finally:
        for doc in reversed(list(app.documents)):
            if doc.creationId not in before:
                doc.close(False)
        if original_active and original_active.isValid:
            original_active.activate()
        report.update(after=[doc.creationId for doc in app.documents], restored=app.documents.count == 0 and app.activeDocument is None)
        save("terminal")
    print(json.dumps({k:report[k] for k in ("status","coldBodyCount","restored","error") if k in report}))
