"""Reopen the exact student download in Fusion, exercise real joints and export kit bodies."""

from pathlib import Path
import adsk
import hashlib
import json
import adsk.core as core
import adsk.fusion as fusion


def run(context):
    cfg = json.loads(Path(r"INPUT_MANIFEST_JSON").read_text(encoding="utf-8-sig"))
    out = Path(cfg["out"])
    archive = out / "Team-1780-shallow-cage-study.f3d"
    app = core.Application.get()
    doc = app.importManager.importToNewDocument(
        app.importManager.createFusionArchiveImportOptions(str(archive))
    )
    try:
        d = fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        root = d.rootComponent
        groups = {o.component.name: o for o in root.occurrences}
        joints = {j.name: j for j in root.asBuiltJoints}
        wanted = {
            "Lift crosshead": ("Cage crosshead", 319.4),
            "Spread left cage shoe": ("Left cage shoe", 36.0),
            "Spread right cage shoe": ("Right cage shoe", -36.0),
            "Preload cage plate": ("Lower plate clamp", 13.584),
        }
        samples = {}
        for key, (group, req) in wanted.items():
            j = joints[key]
            dat = []
            for mm in [0, req, 0]:
                jm = fusion.SliderJointMotion.cast(j.jointMotion)
                jm.slideValue = mm / 10
                adsk.doEvents()
                tr = groups[group].transform2
                dat.append(
                    dict(
                        commandMm=mm,
                        nativeMm=jm.slideValue * 10,
                        xyzMm=[x * 10 for x in tr.translation.asArray()],
                    )
                )
            samples[key] = dat
        kit = {}
        mgr = d.exportManager
        for name in [
            "Shallow cage jack frame",
            "Cage crosshead",
            "Left cage shoe",
            "Right cage shoe",
            "Lower plate clamp",
        ]:
            comp = groups[name].component
            filename = name.replace(" ", "-").lower() + ".step"
            path = out / filename
            if not mgr.execute(mgr.createSTEPExportOptions(str(path), comp)):
                raise RuntimeError(filename)
            kit[name] = {
                "bodies": [
                    {"name": b.name, "volumeMm3": b.volume * 1000}
                    for b in comp.bRepBodies
                ],
                "features": comp.features.extrudeFeatures.count,
                "step": filename,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        (out / "motion-debug.json").write_text(json.dumps(samples))
        samples_check = {
            k: max(abs(s[1]["xyzMm"][i] - s[0]["xyzMm"][i]) for i in range(3))
            for k, s in samples.items()
        }
        if any(abs(samples_check[k] - abs(wanted[k][1])) > 0.002 for k in wanted):
            raise ValueError(str(samples_check))
        data = dict(
            source="cad/shallow-cage/" + archive.name,
            sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
            opened=True,
            rootOccurrences=len(groups),
            nativeJoints=len(joints),
            added=kit,
            motion=samples,
        )
        (out / "reopen.json").write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf8"
        )
        vp = app.activeViewport
        cam = vp.camera
        cam.viewOrientation = core.ViewOrientations.IsoTopRightViewOrientation
        cam.isFitView = True
        cam.isSmoothTransition = False
        vp.camera = cam
        vp.refresh()
        vp.saveAsImageFile(str(out / "climber-native.png"), 1280, 850)
    finally:
        doc.close(False)
