"""Run inside Autodesk Fusion with a local JSON manifest bound to source STEP hashes.

The local manifest path is supplied to the script runner as its context dictionary
or substituted in a temporary copy. Commercial archives remain on the owner's system.
"""
import hashlib
import json
from pathlib import Path
import adsk.core
import adsk.fusion


def run(context):
    config = json.loads(Path(context["manifest"] if isinstance(context, dict) and "manifest" in context
                             else "INPUT_MANIFEST_JSON").read_bytes())
    out = Path(config["output"])
    source = Path(config["module"])
    if hashlib.sha256(source.read_bytes()).hexdigest() != config["moduleSha256"]:
        raise ValueError("The registered manufacturer exchange has changed")
    app = adsk.core.Application.get()
    title = "FRC 2025 - Fixed-height robot - MAXSwerve chassis"
    if any(doc.name == title for doc in app.documents):
        raise ValueError("A chassis study is already open. Reuse the identified document.")
    report = {"title": title, "manufacturerStepSha256": config["moduleSha256"], "stages": []}
    def record(stage, **details):
        report["stages"].append({"stage": stage, **details})
        (out / "fusion-build-progress.json").write_text(json.dumps(report, indent=2))
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.name = title
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    def mat(values, offset):
        m = adsk.core.Matrix3D.create()
        for row in range(3):
            for col in range(3):
                m.setCell(row, col, values[row][col])
            m.setCell(row, 3, offset[row] / 10)
        return m
    def dims(name, mm, description):
        return design.userParameters.add(name, adsk.core.ValueInput.createByString(str(mm) + " mm"),
                                         "mm", description)
    dims("Wheelbase", 564, "Distance between front and rear steer axes")
    dims("Track", 554, "Distance between left and right steer axes")
    dims("NominalWheel", 76.2, "Free molded tire diameter before installed rollout measurement")
    sketch = root.sketches.add(root.xYConstructionPlane)
    sketch.name = "Steer axis centers - wheelbase and track"
    pnt = adsk.core.Point3D.create
    dims2 = sketch.sketchDimensions
    orient = adsk.fusion.DimensionOrientations
    for tag, site in config["sites"].items():
        x, y = site["xM"] * 100, site["yM"] * 100
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(pnt(x,y,0),3.81)
        diameter = dims2.addDiameterDimension(circle, pnt(x+4,y+4,0))
        diameter.parameter.expression = "NominalWheel"
        dims2.addDistanceDimension(sketch.originPoint,circle.centerSketchPoint,orient.HorizontalDimensionOrientation,pnt(x/2,y+2,0),False)
        dims2.addDistanceDimension(sketch.originPoint,circle.centerSketchPoint,orient.VerticalDimensionOrientation,pnt(x+2,y/2,0),False)
    record("created-parametric-layout", documentCreationId=doc.creationId, sketchDimensions=dims2.count)
    for tag,site in config["sites"].items():
        mx = mat(site["sourceXyzToRobot"], [site["xM"]*1000,site["yM"]*1000,38.1])
        occ = root.occurrences.addNewComponent(mx)
        comp = occ.component
        comp.name = tag + " - REV MAXSwerve"
        comp.attributes.add("FRC","sourceSHA256",config["moduleSha256"])
        occ.attributes.add("FRC","corner",tag)
        ok = app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(source)),comp)
        if not ok: raise RuntimeError("Manufacturer import failed "+tag)
        record("manufacturer-import",corner=tag,nested=comp.allOccurrences.count,matrix=occ.transform2.asArray())
    sketch.isVisible = False
    archive = out/"fixed-height-maxswerve-independent.f3d"
    if archive.exists(): raise ValueError("Archive already exists; name the revision before exporting")
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(archive))):
        raise RuntimeError("Archive export failed")
    record("exported", archiveName=archive.name, byteLength=archive.stat().st_size,
           sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    reopened=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
    design=adsk.fusion.Design.cast(app.activeProduct)
    root=design.rootComponent
    rows={}
    for o in root.occurrences:
        tag=o.attributes.itemByName("FRC","corner")
        if not tag: continue
        rows[tag.value]={"transformCm":o.transform2.asArray(),"nestedOccurrences":o.component.allOccurrences.count,
                         "partBodies":sum(x.bRepBodies.count for x in o.component.allOccurrences)+o.component.bRepBodies.count,
                         "sourceSha256":o.component.attributes.itemByName("FRC","sourceSHA256").value}
    if set(rows)!=set(config["sites"]) or any(v["partBodies"]<60 for v in rows.values()):
        raise ValueError("Reopened native module occurrence, body count or corner assignment is incomplete")
    for tag,v in rows.items():
        s=config["sites"][tag]
        expected=mat(s["sourceXyzToRobot"],[s["xM"]*1000,s["yM"]*1000,38.1]).asArray()
        if max(abs(x-y) for x,y in zip(expected,v["transformCm"]))>1e-5:raise ValueError("Reopen transform changed: "+tag)
    record("reopened", corners=rows, sketchDimensions=root.sketches.item(0).sketchDimensions.count,
           parameters={p.name:p.expression for p in design.userParameters}, document=reopened.name)
    app.activeViewport.fit()
    app.activeViewport.saveAsImageFile(str(out/"fusion-chassis.png"),1400,900)
