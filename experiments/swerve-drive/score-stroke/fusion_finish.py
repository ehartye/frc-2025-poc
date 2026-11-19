"""Apply the selected R4's actual role appearance to this isolated native candidate."""

from pathlib import Path
import json
import traceback
import adsk.core
import adsk.fusion


def run(context):
    cfg = json.loads(Path("INPUT_MANIFEST_JSON").read_bytes())
    app = adsk.core.Application.get()
    active = app.activeDocument
    docs = []
    out = {"errors": []}
    try:
        manager = app.importManager
        source = manager.importToNewDocument(
            manager.createFusionArchiveImportOptions(cfg["source"])
        )
        docs.append(source)
        design = adsk.fusion.Design.cast(
            source.products.itemByProductType("DesignProductType")
        )
        target = set(cfg["parts"])
        source_role = next(
            x
            for x in json.loads(Path(cfg["paletteSource"]).read_bytes())[
                "originalGeometry"
            ]["roles"]
            if x["role"] == "shelf"
        )
        color = bytes.fromhex(source_role["rgb"][1:])
        base = app.materialLibraries.itemByName(
            "Fusion Appearance Library"
        ).appearances.itemByName("Plastic - Matte (Gray)")
        appearance = design.appearances.addByCopy(base, "FRC 2025 - shelf")
        for prop_id in ("opaque_albedo", "surface_albedo"):
            prop = adsk.core.ColorProperty.cast(
                appearance.appearanceProperties.itemById(prop_id)
            )
            if prop:
                prop.value = adsk.core.Color.create(*color, 255)
        found = set()
        for comp in design.allComponents:
            if comp.name in target and comp.bRepBodies.count == 1:
                comp.bRepBodies.item(0).appearance = appearance
                found.add(comp.name)
        if found != target:
            raise ValueError("Different Fusion components")
        exporter = design.exportManager
        if not exporter.execute(
            exporter.createFusionArchiveExportOptions(cfg["archive"])
        ):
            raise ValueError("Color export failed")
        doc = manager.importToNewDocument(
            manager.createFusionArchiveImportOptions(cfg["archive"])
        )
        docs.append(doc)
        reopening = adsk.fusion.Design.cast(
            doc.products.itemByProductType("DesignProductType")
        )
        rows = {}
        for comp in reopening.allComponents:
            if comp.name in target and comp.bRepBodies.count == 1:
                body = comp.bRepBodies.item(0)
                ap = body.appearance
                prop = ap.appearanceProperties.itemById(
                    "opaque_albedo"
                ) or ap.appearanceProperties.itemById("surface_albedo")
                actual = adsk.core.ColorProperty.cast(prop).value
                rgb = "#%02X%02X%02X" % (actual.red, actual.green, actual.blue)
                if rgb != source_role["rgb"]:
                    raise ValueError(f"Unexpected reopened appearance {rgb}")
                rows[comp.name] = {
                    "role": "shelf",
                    "rgb": rgb,
                    "volumeMm3": body.volume * 1000,
                    "extrudes": comp.features.extrudeFeatures.count,
                }
        if set(rows) != target:
            raise ValueError("Wrong reopened inventory")
        out["reopened"] = rows
        out["reopenedLaneWidthMm"] = (
            reopening.userParameters.itemByName("camLaneWidth").value * 10
        )
    except Exception:
        out["errors"].append(traceback.format_exc())
    finally:
        for doc in reversed(docs):
            doc.close(False)
        if active and active.isValid:
            active.activate()
        Path(cfg["receipt"]).write_text(json.dumps(out, indent=2), encoding="utf8")
