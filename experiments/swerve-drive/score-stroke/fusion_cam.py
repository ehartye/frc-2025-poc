"""Build an isolated student Fusion document from the project's existing stock and native features."""

from pathlib import Path
import json
import traceback
import adsk.core
import adsk.fusion


def prism(component, body, y, width, points, operation, label):
    core = adsk.core
    normal = component.xZConstructionPlane.geometry.normal.y
    planes = component.constructionPlanes
    plane_input = planes.createInput()
    plane_input.setByOffset(
        component.xZConstructionPlane, core.ValueInput.createByReal(y / 10 / normal)
    )
    plane = planes.add(plane_input)
    plane.name = f"{label} axis"
    sketch = component.sketches.add(plane)
    sketch.name = f"{label} · longitudinal profile"
    points = [
        sketch.modelToSketchSpace(core.Point3D.create(x / 10, y / 10, z / 10))
        for x, z in points
    ]
    for a, b in zip(points, points[1:] + points[:1]):
        sketch.sketchCurves.sketchLines.addByTwoPoints(a, b)
    extrudes = component.features.extrudeFeatures
    feature = extrudes.createInput(sketch.profiles.item(0), operation)
    feature.setSymmetricExtent(core.ValueInput.createByString("camLaneWidth"), True)
    feature.participantBodies = [body]
    finished = extrudes.add(feature)
    finished.name = label


def run(context):
    cfg = json.loads(Path("INPUT_MANIFEST_JSON").read_bytes())
    app = adsk.core.Application.get()
    active = app.activeDocument
    ours = []
    receipt = {"errors": []}
    try:
        source = app.importManager.importToNewDocument(
            app.importManager.createSTEPImportOptions(cfg["baseline"])
        )
        ours.append(source)
        baseline = adsk.fusion.Design.cast(
            source.products.itemByProductType("DesignProductType")
        )
        stock = {}
        for comp in baseline.allComponents:
            for body in comp.bRepBodies:
                for key in cfg["baselineVolumesMm3"]:
                    if (
                        key in (body.name, comp.name)
                        or abs(body.volume * 1000 - cfg["baselineVolumesMm3"][key])
                        < 0.02
                    ):
                        stock[key] = body
        if set(stock) != set(cfg["baselineVolumesMm3"]):
            raise ValueError(f"Source stock count {list(stock)}")
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        ours.append(doc)
        design = adsk.fusion.Design.cast(
            doc.products.itemByProductType("DesignProductType")
        )
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        doc.name = "R4 receiver · continuous scoring cam lands · study"
        design.userParameters.add(
            "camLaneWidth",
            adsk.core.ValueInput.createByString("10 mm"),
            "mm",
            "Finished land lane width",
        )
        manager = adsk.fusion.TemporaryBRepManager.get()
        rows = {}
        for key, old in stock.items():
            comp = design.rootComponent.occurrences.addNewComponent(
                adsk.core.Matrix3D.create()
            ).component
            comp.name = key
            base = comp.features.baseFeatures.add()
            base.name = "Issued R4 machined stock"
            base.startEdit()
            body = comp.bRepBodies.add(manager.copy(old), base)
            body.name = key
            base.finishEdit()
            for y in (-66.0, 42.0):
                if key == "r6-receiver-insert":
                    prism(
                        comp,
                        comp.bRepBodies.item(0),
                        y,
                        10,
                        [(1, 488), (115, 485), (118, 485), (118, 496), (1, 496)],
                        adsk.fusion.FeatureOperations.JoinFeatureOperation,
                        f"Retain billet land at Y{y:g}",
                    )
                prism(
                    comp,
                    comp.bRepBodies.item(0),
                    y,
                    10,
                    [(-105, 480), (115, 480), (115, 485), (1, 488), (-105, 488)],
                    adsk.fusion.FeatureOperations.CutFeatureOperation,
                    f"Machine relief at Y{y:g}",
                )
            volume = comp.bRepBodies.item(0).volume * 1000
            rows[key] = {
                "volumeMm3": volume,
                "extrudes": comp.features.extrudeFeatures.count,
            }
            if (
                abs(volume - cfg["candidateVolumesMm3"][key]) > 0.8
                or comp.bRepBodies.count != 1
            ):
                raise ValueError(
                    f"Unexpected native machined volume {key} {rows[key]} expected {cfg['candidateVolumesMm3'][key]}"
                )
        exporter = design.exportManager
        if not exporter.execute(
            exporter.createFusionArchiveExportOptions(cfg["archive"])
        ):
            raise ValueError("Fusion archive export failed")
        reopened = app.importManager.importToNewDocument(
            app.importManager.createFusionArchiveImportOptions(cfg["archive"])
        )
        ours.append(reopened)
        result = adsk.fusion.Design.cast(
            reopened.products.itemByProductType("DesignProductType")
        )
        receipt["reopened"] = {
            c.name: {
                "volumeMm3": c.bRepBodies.item(0).volume * 1000,
                "extrudes": c.features.extrudeFeatures.count,
            }
            for c in result.allComponents
            if c.bRepBodies.count == 1
        }
        if set(receipt["reopened"]) != set(rows) or any(
            abs(receipt["reopened"][k]["volumeMm3"] - v["volumeMm3"]) > 0.05
            or receipt["reopened"][k]["extrudes"] != v["extrudes"]
            for k, v in rows.items()
        ):
            raise ValueError("Reopened archive differs")
        receipt["nativeFeatures"] = rows
        receipt["reopenedLaneWidthMm"] = (
            result.userParameters.itemByName("camLaneWidth").value * 10
        )
    except Exception:
        receipt["errors"].append(traceback.format_exc())
    finally:
        for doc in reversed(ours):
            doc.close(False)
        if active and active.isValid:
            active.activate()
        Path(cfg["receipt"]).write_text(json.dumps(receipt, indent=2), encoding="utf8")
