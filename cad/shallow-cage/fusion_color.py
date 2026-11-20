"""Apply the project palette to the Fusion-authored alternate before independent reopening."""

from pathlib import Path
import json
import adsk.core as core
import adsk.fusion as fusion


def run(context):
    cfg = json.loads(Path(r"INPUT_MANIFEST_JSON").read_text(encoding="utf-8-sig"))
    out = Path(cfg["out"])
    name = "Team-1780-shallow-cage-study.f3d"
    pending = out / "uncolored.f3d"
    if not pending.exists():
        (out / name).rename(pending)
    app = core.Application.get()
    doc = app.importManager.importToNewDocument(
        app.importManager.createFusionArchiveImportOptions(str(pending))
    )
    try:
        design = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        roles = {
            "Shallow cage jack frame": "frame",
            "Cage crosshead": "lift-drive",
            "Lower plate clamp": "mechanism",
            "Left cage shoe": "steel",
            "Right cage shoe": "steel",
        }
        colors = json.loads(
            (Path(cfg["project"]) / "cad/component-palette.json").read_bytes()
        )["colors"]
        colors.setdefault("mechanism", "#007F78")
        matte = app.materialLibraries.itemByName(
            "Fusion Appearance Library"
        ).appearances.itemByName("Plastic - Matte (Gray)")
        for occ in design.rootComponent.occurrences:
            if occ.component.name not in roles:
                continue
            role = roles[occ.component.name]
            key = "Shallow jack - " + role
            ap = design.appearances.itemByName(key) or design.appearances.addByCopy(
                matte, key
            )
            rgb = bytes.fromhex(colors[role][1:])
            prop = core.ColorProperty.cast(
                ap.appearanceProperties.itemById("opaque_albedo")
            )
            prop.value = core.Color.create(*rgb, 255)
            for b in occ.component.bRepBodies:
                b.appearance = ap
        mgr = design.exportManager
        if not mgr.execute(mgr.createFusionArchiveExportOptions(str(out / name))):
            raise RuntimeError("Color archive export failed")
    finally:
        doc.close(False)
