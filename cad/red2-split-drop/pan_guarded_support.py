"""Extract the three editable machining articles and replay the first checked insertion."""
import json
import traceback
from pathlib import Path

import adsk
from adsk import core, fusion


def run(context):
    from hashlib import sha256
    import importlib.util

    def sha(path):
        return sha256(path.read_bytes()).hexdigest()

    def module(name):
        path = Path(CONFIG["plugin"]) / (name + ".py")
        spec = importlib.util.spec_from_file_location(name, path)
        result = importlib.util.module_from_spec(spec)
        exec(compile(path.read_bytes(), str(path), "exec"), result.__dict__)
        return result

    app = core.Application.get()
    before = [doc.creationId for doc in app.documents]
    previous = app.activeDocument
    root = Path(CONFIG["jobInputs"])
    out = Path(CONFIG["resultDirectory"])
    out.mkdir(exist_ok=True)
    report = dict(before=before)
    result_file = out / (CONFIG["requestId"] + ".json")

    def save():
        result_file.write_text(json.dumps(report, indent=2))

    def open_archive(file):
        options = app.importManager.createFusionArchiveImportOptions(str(file))
        doc = app.importManager.importToNewDocument(options)
        return doc, fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))

    def export(design, file, component=None):
        if file.exists():
            raise ValueError("Existing issue " + file.name)
        args = (str(file), component) if component else (str(file),)
        opts = design.exportManager.createFusionArchiveExportOptions(*args)
        if not opts or not design.exportManager.execute(opts):
            raise ValueError("Could not export " + file.name)
        return sha(file)

    def location(design):
        return {o.fullPathName: o for o in design.rootComponent.allOccurrences}

    def census(design):
        if design.parentDocument.documentReferences.count:
            raise ValueError("Unexpected externally linked article")
        records = {}
        for occ in design.rootComponent.allOccurrences:
            if occ.isReferencedComponent or occ.component.meshBodies.count:
                raise ValueError("Unresolved material in " + occ.fullPathName)
            for b in occ.component.bRepBodies:
                if not b.isSolid:
                    raise ValueError("Non-solid article " + b.name)
                key = occ.fullPathName + "/" + b.name
                if key in records:
                    raise ValueError("Duplicate stock ID " + key)
                box = b.createForAssemblyContext(occ).preciseBoundingBox
                records[key] = dict(world=list(occ.transform2.asArray()),
                                    boundsMm=[10*x for point in (box.minPoint,box.maxPoint) for x in point.asArray()],
                                    material=b.material.name)
        if design.rootComponent.bRepBodies.count:
            raise ValueError("Unexpected root-level body")
        return records

    def compare(expected, actual):
        if set(expected) != set(actual):
            raise ValueError("Installed body roster changed")
        for key, a in expected.items():
            b = actual[key]
            if a["material"] != b["material"] or any(abs(x-y) > 1e-6 for field in ("world","boundsMm")
                                                      for x,y in zip(a[field],b[field])):
                raise ValueError("Retained geometry, material or placement changed: " + key)

    def feature_counts(component):
        return dict(sketches=component.sketches.count, extrudes=component.features.extrudeFeatures.count,
                    combines=component.features.combineFeatures.count)

    d = None
    try:
        if result_file.exists():
            raise ValueError("Fresh native result required")
        for path, expected in CONFIG["inputs"].items():
            if sha(root/path) != expected:
                raise ValueError("Changed staged input " + path)
        report["authority"] = module("fusion_authority").verify(root,root/"cad/fusion-authority.json",
                                                               root/"artifacts/native-master-review/master-issue-03/lineage.json")
        guard = module("fusion_checked_placement")
        partial = root/"artifacts/red2-integrated/pan-thrust/native"
        cold = json.loads((partial/"pan-r3-stock-machining-01.json").read_bytes())
        parent = partial/"integrated-340mm-pan-r3-machined.f3d"
        if sha(parent) != cold["nativeSha256"] or cold["authority"]["archiveSha256"] != report["authority"]["archiveSha256"]:
            raise ValueError("Machining checkpoint source mismatch")
        keys = ["Stationary diaphragm with four bosses", "Match-ground axial shim", "Hardened removable end cap"]
        fixed = "340 mm pan fixed installation:1"
        src = cold["cold"]["parts"]
        report.update(stage="extract editable sources",sourceSha256=sha(parent),sources={})
        save()
        d,design = open_archive(parent)
        all_stock = census(design)
        if len(all_stock) != 297:
            raise ValueError("Expected 297 initial stocks")
        removable = {}
        for n,key in enumerate(keys,1):
            name = src[key]["body"]
            path = src[key]["path"]
            o = location(design)[path]
            fc = feature_counts(o.component)
            if o.component.bRepBodies.count != 1 or not fc["extrudes"] or not fc["sketches"]:
                raise ValueError("No editable machined stock for " + name)
            record = all_stock[path+"/"+name]
            source = out/f"article-{n}.f3d"
            report["sources"][name] = dict(archive=source.name,sha256=export(design,source,o.component),
                                           originalPath=path,features=fc,world=record["world"],material=record["material"])
            removable[path+"/"+name] = record
        save()
        for key in reversed(keys):
            path = src[key]["path"]
            if not location(design)[path].deleteMe():
                raise ValueError("Could not detach private source " + path)
        if not design.computeAll():
            raise ValueError("Could not recompute the working predecessor")
        retained = {k:v for k,v in all_stock.items() if k not in removable}
        compare(retained,census(design))
        predecessor = out/"machined-preinsertion-294.f3d"
        report["predecessorSha256"] = export(design,predecessor)
        d.close(False)
        d = None
        NATIVE_JOB["verifyDependencies"]()
        d,design = open_archive(predecessor)
        compare(retained,census(design))
        report.update(stage="cold 294-body predecessor",retainedBodies=len(retained))
        save()
        d.close(False)
        d = None

        # The drawing makes the support axes and both original fixed wrapper origins coincident.
        path = src[keys[0]]["path"]
        name = src[keys[0]]["body"]
        tower = fixed + "+Starboard bearing tower:1"
        identity = list(core.Matrix3D.create().asArray())
        data = report["sources"][name]
        if max(abs(x-y) for x,y in zip(data["world"],identity)) > 1e-8:
            raise ValueError("Unexpected original tower/diaphragm chassis datum")
        request = dict(occurrence=path,worldTransformCm=identity,interfaces=[dict(
            kind="bearing", maxGapMm=.001,bodies=[[path,name],[tower,"Starboard bearing tower"]],
            basis="Four integral support bosses seat on the starboard tower outer Y=-198.15 mm face.")])
        def edit(design):
            comp = location(design)[fixed].component
            file = out/data["archive"]
            if sha(file) != data["sha256"]:
                raise ValueError("Changed extracted source")
            previous_paths = set(location(design))
            opts = app.importManager.createFusionArchiveImportOptions(str(file))
            inserted = app.importManager.importToTarget2(opts,comp)
            children = [o for p,o in location(design).items() if p not in previous_paths]
            if not inserted or len(children) != 1:
                raise ValueError("The archive must create one local component")
            target = children[0]
            target.component.name = name
            target.isGroundToParent = True
            if feature_counts(target.component) != data["features"] or target.component.bRepBodies.item(0).material.name != data["material"]:
                raise ValueError("Editable features or material changed on installation")
        report["stage"] = "guarding fixed support"
        save()
        successor = out/"checked-fixed-support-295.f3d"
        report["guard"] = guard.checked_placement(app,predecessor,report["predecessorSha256"],successor,request,edit)
        if report["guard"]["status"] != "pass":
            raise ValueError("The first placement was held by the native guard")
        d,design = open_archive(successor)
        compare(retained|{path+"/"+name:all_stock[path+"/"+name]},census(design))
        report.update(status="checked-fixed-support",stage="cold checked support",successorSha256=sha(successor))
    except Exception:
        report.update(status="failed",error=traceback.format_exc())
    finally:
        if d and d.creationId not in before:
            d.close(False)
        if previous and previous.isValid:
            previous.activate()
        report["restored"] = before == [doc.creationId for doc in app.documents]
        save()
    print(json.dumps({k:report[k] for k in ("status","stage","restored","successorSha256","error") if k in report}))
