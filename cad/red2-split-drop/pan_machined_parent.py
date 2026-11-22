"""Machine only the three existing R2 blanks and qualify its native stowed pose through recompute."""
import hashlib
import importlib.util
import itertools
import json
import math
import traceback
from pathlib import Path

import adsk
from adsk import core, fusion


SHAFT = "Idler headed stub hex shank"
SOCKET = "Starboard driven hex socket"
TOWER = "Starboard bearing tower"
FLOOR = "430 mm pan floor"
NEW_PARTS = {SHAFT:"One-piece stepped shaft", SOCKET:"Cross-drilled moving socket", TOWER:"Drilled starboard bearing tower"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(path, name):
    spec = importlib.util.spec_from_file_location(name,path)
    obj = importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(),str(path),"exec"),obj.__dict__)
    return obj


def component(design,name):
    hits=[o for o in design.rootComponent.allOccurrences if o.component.name==name]
    if len(hits)!=1:
        raise ValueError("Expected one " + name)
    return hits[0]


def pose(design):
    joint=design.rootComponent.asBuiltJoints.itemByName("Pan shaft pitch")
    occ=component(design,FLOOR)
    b=occ.component.bRepBodies.item(0)
    local=[54.5,16.1,58.]
    v=next(v for v in b.vertices if math.dist(v.geometry.asArray(),local)<1e-7)
    true_world=[10*x for x in v.createForAssemblyContext(occ).geometry.asArray()]
    r=joint.jointMotion.rotationValue
    expected=[205+340*math.cos(r),161,580-340*math.sin(r)]
    limits=joint.jointMotion.rotationLimits
    return dict(jointRad=r,rootWorldMatrixCm=list(occ.transform2.asArray()),landmarkWorldMm=true_world,
                landmarkLocalMm=[10*x for x in v.geometry.asArray()],landmarkExpectedFromSignedAxisMm=expected,
                landmarkErrorMm=math.dist(true_world,expected),localVolumeMm3=1000*b.getPhysicalProperties(
                    fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume,
                floorLocalBoundsMm=[10*x for p in (b.preciseBoundingBox.minPoint,b.preciseBoundingBox.maxPoint) for x in p.asArray()],
                floorMaterial=b.material.name,axisXZmm=[205,580],currentRestRad=limits.restValue,
                restEnabled=limits.isRestValueEnabled,snapshots=design.snapshots.count)


def check_stow(data):
    if abs(data["jointRad"]+math.radians(70))>1e-7 or data["landmarkErrorMm"]>.003:
        raise ValueError("Incorrect installed stowed coordinate: "+str(data))


def set_joint(design, rad):
    m=design.rootComponent.asBuiltJoints.itemByName("Pan shaft pitch").jointMotion
    m.rotationValue=rad
    adsk.doEvents()


def set_stow(design):
    motion=design.rootComponent.asBuiltJoints.itemByName("Pan shaft pitch").jointMotion
    rest=-math.radians(70)
    # This is the return/park coordinate of a freely movable joint.
    motion.rotationLimits.restValue=rest
    motion.rotationLimits.isRestValueEnabled=True
    set_joint(design,rest)
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    check_stow(pose(design))


def census(design):
    if design.parentDocument.documentReferences.count or design.rootComponent.bRepBodies.count:
        raise ValueError("Unexpected linked document or root body")
    out={}
    for occ in design.rootComponent.allOccurrences:
        if occ.isReferencedComponent or occ.component.meshBodies.count:
            raise ValueError("Unresolved geometry: "+occ.fullPathName)
        for body in occ.component.bRepBodies:
            id=occ.fullPathName+"/"+body.name
            if id in out or not body.isSolid:
                raise ValueError("Unresolved physical identity "+id)
            box=body.createForAssemblyContext(occ).preciseBoundingBox
            out[id]=dict(world=list(occ.transform2.asArray()),material=body.material.name,
                         bounds=[10*x for p in (box.minPoint,box.maxPoint) for x in p.asArray()])
    return out


def compare_retained(before,after):
    if len(before)!=294 or set(before)!=set(after):
        raise ValueError("Need the same 294 retained R2 bodies")
    changed=0
    for k,a in before.items():
        b=after[k]
        if a["material"]!=b["material"] or max(abs(x-y) for x,y in zip(a["world"],b["world"]))>1e-7:
            raise ValueError("Changed existing material/placement "+str((k,a,b)))
        if k.rsplit("/",1)[1] in NEW_PARTS:
            changed+=1
        elif max(abs(x-y) for x,y in zip(a["bounds"],b["bounds"]))>1e-5:
            raise ValueError("Changed nearby geometry "+str((k,a,b)))
    if changed!=3:
        raise ValueError("Need exactly three machining targets")


def machine(design,plan,geom,scoped):
    def circle(name,axis,start,length,radius,label,join=False):
        comp=component(design,name).component
        body=comp.bRepBodies.item(0)
        if comp.bRepBodies.count!=1:
            raise ValueError("Unexpected stock "+name)
        plane,sign=geom.plane(comp,axis,start[axis])
        sketch=comp.sketches.add(plane)
        sketch.name=label+" profile"
        sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.modelToSketchSpace(geom.point(start)),radius/10)
        creating=fusion.FeatureOperations.NewBodyFeatureOperation
        cutting=fusion.FeatureOperations.CutFeatureOperation
        feature=scoped.extrusion(comp,sketch.profiles.item(0),length/sign,creating if join else cutting,
                                 geom,None if join else body)
        feature.name=label
        if join:
            if feature.bodies.count!=1:
                raise ValueError("Ambiguous added stock section "+label)
            collection=core.ObjectCollection.create()
            collection.add(feature.bodies.item(0))
            args=comp.features.combineFeatures.createInput(body,collection)
            args.operation=fusion.FeatureOperations.JoinFeatureOperation
            args.isKeepToolBodies=False
            comp.features.combineFeatures.add(args).name="Join "+label+" to stock"
        sketch.isLightBulbOn=False
        if comp.bRepBodies.count!=1:
            raise ValueError("Machining split "+name)
    start=plan["stackPlanesUmm"][0]
    end=plan["rigidCapSeatUmm"]
    circle(SHAFT,1,[205,-202,580],-(start-202),11.35,"Turn abutment",True)
    circle(SHAFT,1,[205,-start,580],-(end-start),6.33,"Turn journal",True)
    circle(SHAFT,1,[205,-(end-10),580],-11,1.5,"Axial cap screw envelope")
    circle(SHAFT,2,[205,-177,564],24,1.65,"Shaft cross pin clearance")
    circle(SOCKET,2,[205,-177,564],24,1.65,"Socket cross pin clearance")
    circle(SOCKET,2,[205,-177,588],10,1.5,"Socket cross screw thread envelope")
    for n,(x,z) in enumerate(itertools.product([184.,226.],[570.,590.]),1):
        circle(TOWER,1,[x,-190.,z],-10,1.5,"Support attachment thread envelope "+str(n))


def run(context):
    app=core.Application.get()
    active=app.activeDocument
    before=[doc.creationId for doc in app.documents]
    root=Path(CONFIG["jobInputs"])
    out=Path(CONFIG["resultDirectory"])
    out.mkdir(exist_ok=True)
    report=dict(before=before,poses={})
    rec=out/(CONFIG["requestId"]+".json")
    def store():rec.write_text(json.dumps(report,indent=2))
    def open_doc(path):
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
        return doc,fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
    doc=None
    try:
        for file,sig in CONFIG["inputs"].items():
            if sha(root/file)!=sig:raise ValueError("Changed frozen input "+file)
        ptr=root/"artifacts/red2-integrated"
        plan=json.loads((ptr/"pan-thrust/r3-native-plan.json").read_bytes())
        current=module(Path(CONFIG["plugin"])/"fusion_authority.py","pan_authority")
        report["authority"]=current.verify(root,root/"cad/fusion-authority.json",root/"artifacts/native-master-review/master-issue-03/lineage.json")
        if report["authority"]["archiveSha256"]!=plan["authority"]["sha256"]:raise ValueError("Changed authority")
        # Observe a bare recomputation before any deletion or machining. Leave the registered R3 bytes untouched.
        doc,design=open_doc(ptr/"pan-thrust/native/integrated-340mm-pan-r3-machined.f3d")
        log=report["poses"]
        log["r3Cold"]=pose(design)
        check_stow(log["r3Cold"])
        if not design.computeAll():raise ValueError("R3 bare recomputation failed")
        adsk.doEvents()
        log["r3AfterBareRecompute"]=pose(design)
        store()
        doc.close(False)
        doc=None
        parent=ptr/"pan-native/integrated-340mm-pan-r2.f3d"
        if sha(parent)!=plan["parentArchive"]:raise ValueError("Changed R2 predecessor")
        doc,design=open_doc(parent)
        report["parentSha256"]=sha(parent)
        log["r2Cold"]=pose(design)
        check_stow(log["r2Cold"])
        reference=census(design)
        set_joint(design,0)
        if abs(pose(design)["jointRad"])>1e-7:raise ValueError("Could not position the machining datum")
        scripts=root/"cad/red2-split-drop"
        report["stage"]="machine only three existing stocks"
        store()
        machine(design,plan,module(scripts/"chassis_native.py","pan_geom"),module(scripts/"pan_scoped_stock.py","pan_cut"))
        set_stow(design)
        log["beforeRecompute"]=pose(design)
        if not design.computeAll():raise ValueError("R2 machining recompute failed")
        adsk.doEvents()
        log["afterRecompute"]=pose(design)
        store()
        check_stow(log["afterRecompute"])
        compare_retained(reference,census(design))
        def inspect():
            measurements={}
            for name,k in NEW_PARTS.items():
                occ=component(design,name)
                b=occ.component.bRepBodies.item(0)
                v=1000*b.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume
                wanted=plan["stock"][k]["volumeMm3"]
                measurements[name]=dict(volumeMm3=v,expectedMm3=wanted,material=b.material.name)
                if occ.component.bRepBodies.count!=1 or abs(v-wanted)>.025 or b.material.name!="2025 6061-T6":
                    raise ValueError("Bad machined material or solid "+name)
            return measurements
        report["machining"]=inspect()
        file=out/"machined-preinsertion-294.f3d"
        if file.exists() or not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(file))):
            raise ValueError("Could not save fresh machined-only archive")
        report["archiveSha256"]=sha(file)
        store()
        doc.close(False)
        doc=None
        NATIVE_JOB["verifyDependencies"]()
        doc,design=open_doc(file)
        log["cold294"]=pose(design)
        check_stow(log["cold294"])
        report["coldMachining"]=inspect()
        compare_retained(reference,census(design))
        report.update(stage="cold 294-body parent before placements",status="machined-only",retainedBodies=294)
        step=file.with_suffix(".step")
        if step.exists() or not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
            raise ValueError("Cold STEP export failed")
        report["stepSha256"]=sha(step)
    except Exception:
        report.update(status="failed",error=traceback.format_exc())
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if active and active.isValid:active.activate()
        report["restored"]=before==[d.creationId for d in app.documents]
        store()
    print(json.dumps({k:report[k] for k in ("status","stage","restored","archiveSha256","stepSha256","error") if k in report}))
