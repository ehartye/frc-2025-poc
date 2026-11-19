"""Run one stage per Fusion request against the local project files."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import adsk.core
import adsk.fusion

TITLE="FRC 2025 - Full swerve R3 - authoring"
STUDENT="FRC 2025 - Full swerve R3 - student review"
SUPER="R3 - Fixed-low receiver, guides, controls and red perimeter"
SUPPLIER="R3 - Supplier camera housings and signal light"

def run(context):
    config=Path(context.get("manifest","R3_MANIFEST") if isinstance(context,dict) else "R3_MANIFEST")
    c=json.loads(config.read_bytes())
    work=Path(c["directory"])
    app=adsk.core.Application.get()
    command=c["stage"]
    archive=work/"FRC-2025-full-swerve-R3.f3d"
    manager=app.importManager
    if command in ("create","reopen"):
        source=Path(c["r2"]) if command=="create" else archive
        if command=="create" and hashlib.sha256(source.read_bytes()).hexdigest()!=c["r2Sha256"]:raise ValueError("Changed R2 source")
        doc=manager.importToNewDocument(manager.createFusionArchiveImportOptions(str(source)))
        doc.name=TITLE if command=="create" else STUDENT
        return
    wanted=STUDENT if command=="inspect" else TITLE
    documents=[d for d in app.documents if d.name==wanted]
    if len(documents)!=1:raise ValueError("Expected one named "+wanted)
    doc=documents[0]
    doc.activate()
    design=adsk.fusion.Design.cast(app.activeProduct)
    root=design.rootComponent
    if command in ("super","supplier"):
        name,file=(SUPER,"superstructure.step") if command=="super" else (SUPPLIER,"vendor-optics.step")
        if any(o.component.name==name for o in root.occurrences):raise ValueError("Group already present")
        component=root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        component.name=name
        if not manager.importToTarget(manager.createSTEPImportOptions(str(work/file)),component):raise ValueError(file)
        return
    if command=="export":
        if archive.exists():raise ValueError("Requested output already exists")
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(archive))):
            raise ValueError("Fusion export failed")
        return
    rows=json.loads(Path(c["appearances"]).read_bytes())["solids"]
    colors=json.loads(Path(c["palette"]).read_bytes())["colors"]
    roster={r["id"]:r for r in rows if not r["representation"].startswith("public-")}
    appearances={}
    if command=="color":
        base=app.materialLibraries.itemByName("Fusion Appearance Library").appearances.itemByName("Plastic - Matte (Gray)")
        if not base:raise ValueError("Need Fusion Matte Gray appearance")
        for role in sorted({r["role"] for r in roster.values()}):
            name="FRC 2025 - "+role
            ap=design.appearances.itemByName(name) or design.appearances.addByCopy(base,name)
            rgb=bytes.fromhex(colors[role][1:])
            for prop_id in ("opaque_albedo","surface_albedo"):
                prop=adsk.core.ColorProperty.cast(ap.appearanceProperties.itemById(prop_id))
                prop.value=adsk.core.Color.create(*rgb,255)
            appearances[role]=ap
    counts=Counter()
    touched=set()
    unknown=[]
    errors=[]
    wheels={}
    vendors={}
    optics=[]
    visible={}
    for top in root.occurrences:
        family=top.component.name
        purchased="REV MAXSwerve" in family
        queue=[top]
        while queue:
            occ=queue.pop()
            queue.extend(list(occ.childOccurrences))
            component=occ.component
            if not component.bRepBodies.count:continue
            key=component.name.split(" (")[0]
            if purchased:
                if "REV-21-3004-P01" in key:
                    wheels[family[:2]]=[x*10 for x in occ.bRepBodies.item(0).physicalProperties.centerOfMass.asArray()]
                vendors[family]=vendors.get(family,0)+component.bRepBodies.count
                continue
            if family==SUPPLIER:
                optics.append(key)
                continue
            if key not in roster:
                unknown.append(key);continue
            if key in touched:errors.append("Duplicated project solid "+key)
            touched.add(key)
            role=roster[key]["role"]
            for body in component.bRepBodies:
                if command=="color":body.appearance=appearances[role]
                ap=body.appearance
                if ap is None or ap.name!="FRC 2025 - "+role:
                    errors.append("Different role appearance on "+key)
                    continue
                tone=adsk.core.ColorProperty.cast(ap.appearanceProperties.itemById("opaque_albedo")).value
                if (tone.red,tone.green,tone.blue)!=tuple(bytes.fromhex(colors[role][1:])):errors.append("Different RGB on "+key)
                counts[role]+=1
            if key.startswith("bumper-cover") or key in ("guide-L-machined-mast","guide-R-machined-mast","r6-polymer-facing"):
                real=occ.preciseBoundingBox
                bb=[v*10 for v in real.minPoint.asArray()+real.maxPoint.asArray()]
                err=max(abs(a-b) for a,b in zip(bb,roster[key]["boundsMm"]))
                visible[key]={"boundsMm":bb,"maxBoundErrorMm":err,"role":role,"sRGB":colors[role],"visible":occ.isVisible}
                if err>.01 or not occ.isVisible:errors.append("Registration or visibility mismatch on "+key)
    missing=sorted(roster.keys()-touched)
    for name,pt in wheels.items():
        x=282 if name.startswith("F") else -282
        y=277 if name.endswith("L") else -277
        if max(abs(a-b) for a,b in zip(pt,[x,y,38.1]))>.01:errors.append("Displaced tread "+name)
    result={"configuration":c["configuration"],"document":doc.name,"nativeRootGroups":[o.component.name for o in root.occurrences],
            "projectColoredSolids":sum(counts.values()),"roleCounts":dict(counts),"supplierModuleBodyCounts":vendors,"wheelCentersMm":wheels,
            "supplierOpticalAndLampSolids":len(optics),"registeredVisualChecks":visible,"missing":missing,"unexpected":unknown,"errors":errors}
    (work/(command+"-result.json")).write_text(json.dumps(result,indent=2))
    if len(wheels)!=4 or len(vendors)!=4 or len(optics)<27 or missing or unknown or errors:
        raise ValueError("See "+command+"-result.json")
    vp=app.activeViewport
    camera=vp.camera
    camera.isSmoothTransition=False
    camera.cameraType=adsk.core.CameraTypes.OrthographicCameraType
    camera.target=adsk.core.Point3D.create(0,0,45)
    camera.eye=adsk.core.Point3D.create(155,-203,143)
    camera.upVector=adsk.core.Vector3D.create(0,0,1)
    camera.isFitView=True
    vp.camera=camera
    vp.fit()
    vp.refresh()
    if command=="inspect":
        vp.saveAsImageFile(str(work/"full-swerve-R3.png"),1400,1100)

