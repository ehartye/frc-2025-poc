"""Apply the rear correction to a local copy of the hash-bound student archive in Fusion.

Dispatch each operation separately: open, replace, export, reopen, inspect.
The supplied manifest contains r1Archive, r2Step, r2Archive, output and
the declared hashes r1Sha256/r2StepSha256. Preserve the R1 file.
"""
from pathlib import Path
import hashlib
import json
import adsk.core
import adsk.fusion


def run(context):
    cfg=json.loads(Path(context.get("manifest","FUSION_REAR_MANIFEST") if isinstance(context,dict)
                        else "FUSION_REAR_MANIFEST").read_bytes())
    app=adsk.core.Application.get()
    op=cfg["operation"]
    old=Path(cfg["r1Archive"]);step=Path(cfg["r2Step"]);out=Path(cfg["r2Archive"])
    if op=="open":
        if hashlib.sha256(old.read_bytes()).hexdigest()!=cfg["r1Sha256"]:raise ValueError("Unknown R1 archive")
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(old)))
        doc.name="FRC 2025 - Swerve rear electronics - edit"
        return
    if op=="reopen":
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(out)))
        doc.name="FRC 2025 - MAXSwerve chassis R2 - student review"
        return
    title="FRC 2025 - MAXSwerve chassis R2 - student review" if op=="inspect" else "FRC 2025 - Swerve rear electronics - edit"
    choices=[doc for doc in app.documents if doc.name==title]
    if len(choices)!=1:raise ValueError("Expected one open "+title)
    doc=choices[0];doc.activate()
    design=adsk.fusion.Design.cast(app.activeProduct);root=design.rootComponent
    if op=="replace":
        if hashlib.sha256(step.read_bytes()).hexdigest()!=cfg["r2StepSha256"]:raise ValueError("Unknown revised exchange")
        originals=[c for c in root.allOccurrences if c.component.name.split(" (")[0]=="swerve-relieved-ribbed-deck"]
        if len(originals)!=1:raise ValueError("Original deck name is ambiguous")
        (originals[0].nativeObject or originals[0]).deleteMe()
        c=root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        c.component.name="R2 - Rear motor clearance and elevated stationary host"
        c.component.attributes.add("FRC","revisionStepSHA256",cfg["r2StepSha256"])
        if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(step)),c.component):
            raise ValueError("Could not place rear exchange.")
        return
    if op=="export":
        if out.exists():raise ValueError("Archive already exists")
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(out))):
            raise ValueError("Export failed")
        return
    if op!="inspect":raise ValueError("Unknown operation")
    names=[c.component.name.split(" (")[0] for c in root.allOccurrences]
    if names.count("host-drilled-swerve-deck")!=1 or names.count("swerve-relieved-ribbed-deck")!=0:
        raise ValueError("Reopened deck replacement did not survive")
    wheels=[]
    for c in root.allOccurrences:
        if "REV-21-3004-P01" in c.component.name and c.bRepBodies.count:
            xyz=[x*10 for x in c.bRepBodies.item(0).physicalProperties.centerOfMass.asArray()]
            corner=c.fullPathName.split(" ")[0];ref=cfg["sites"][corner]
            if max(abs(a-b) for a,b in zip(xyz,[1000*ref["xM"],1000*ref["yM"],38.1]))>.01:
                raise ValueError("Unexpected actual imported tread datum "+corner)
            wheels.append({"corner":corner,"centerMm":xyz})
    if len(wheels)!=4:raise ValueError("Expected four source wheel treads")
    result={"archive":out.name,"sha256":hashlib.sha256(out.read_bytes()).hexdigest(),"bytes":out.stat().st_size,
            "frameSourceSha256":cfg["r1Sha256"],"rearStepSha256":cfg["r2StepSha256"],"rootOccurrences":root.occurrences.count,
            "newDeck":names.count("host-drilled-swerve-deck"),"oldDeck":names.count("swerve-relieved-ribbed-deck"),
            "elevatedPlate":names.count("elevated-host-plate"),"wheelCenters":wheels,"activeDocument":doc.name}
    Path(cfg["output"]).write_text(json.dumps(result,indent=2))
