"""Inspect the archived robot's physical properties and available material names in Fusion."""

import csv
import hashlib
import json
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion


def run(context):
    cfg = json.loads(Path(r"INPUT_MANIFEST_JSON").read_text("utf-8-sig"))
    project = Path(cfg["project"])
    out = Path(cfg["out"])
    source = project / "cad/shallow-cage/Team-1780-shallow-cage-study.f3d"
    src = hashlib.sha256(source.read_bytes()).hexdigest()
    assert src == "88cc5f0927094bd2bbc60da862a137e5eada4484cc84510a514d760f27117c49"
    app = core.Application.get()
    doc = app.importManager.importToNewDocument(
        app.importManager.createFusionArchiveImportOptions(str(source))
    )
    try:
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        rows = []
        for o in d.rootComponent.allOccurrences:
            for body in o.component.bRepBodies:
                props = body.physicalProperties
                material = body.material
                rows.append(
                    (
                        o.fullPathName,
                        body.name,
                        material.name,
                        material.id,
                        props.mass,
                        props.volume * 1000,
                        props.density * 1e6,
                    )
                )
        with (out / "native-materials.csv").open("w", encoding="utf8", newline="") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(
                (
                    "occurrence",
                    "body",
                    "material",
                    "materialId",
                    "massKg",
                    "volumeMm3",
                    "densityKgM3",
                )
            )
            w.writerows(rows)
        (out / "native-material-summary.json").write_text(
            json.dumps(
                dict(
                    archiveSha256=src,
                    bodies=len(rows),
                    summedMassKg=sum(r[4] for r in rows),
                    materials=sorted(set(r[2] for r in rows)),
                    rootMassKg=d.rootComponent.physicalProperties.mass,
                    options={
                        lib.name: [m.name for m in lib.materials]
                        for lib in app.materialLibraries
                        if lib.materials.count > 0
                    },
                ),
                indent=2,
            ),
            encoding="utf8",
        )
    finally:
        doc.close(False)
        (out / "audit-closed.txt").write_text(
            "Closed owned audit document", encoding="utf8"
        )
