"""Insert the editable diaphragm and both housing races under individual native placement guards."""
import json
import traceback
from pathlib import Path

import adsk
from adsk import core, fusion


def run(context):
    root=Path(CONFIG["jobInputs"])
    out=Path(CONFIG["resultDirectory"])
    out.mkdir(exist_ok=True)
    result=out/(CONFIG["requestId"]+".json")
    app=core.Application.get()
    active=app.activeDocument
    before=[d.creationId for d in app.documents]
    report=dict(before=before,placements=[])
    def save():result.write_text(json.dumps(report,indent=2))
    import importlib.util
    def module(file,name):
        spec=importlib.util.spec_from_file_location(name,file)
        lib=importlib.util.module_from_spec(spec)
        exec(compile(file.read_bytes(),str(file),"exec"),lib.__dict__)
        return lib
    base=module(root/"cad/red2-split-drop/pan_machined_parent.py","pan_parent")
    guard=module(Path(CONFIG["plugin"])/"fusion_checked_placement.py","pan_guard")
    sha=base.sha
    t=root/"artifacts/red2-integrated/pan-thrust"
    def read(f):return json.loads(f.read_bytes())
    def cold(path):
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
        return doc,fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
    def counts(c):
        return dict(sketches=c.sketches.count,extrudes=c.features.extrudeFeatures.count,combines=c.features.combineFeatures.count)
    def freeze_material(design,b):
        name="6655K17 supplier steel (grade unspecified)"
        candidate=design.materials.itemByName(name)
        if not candidate:
            source=design.materials.itemByName("Steel")
            if not source:
                for lib in app.materialLibraries:
                    source=lib.materials.itemByName("Steel")
                    if source:break
            if not source:raise ValueError("No native physical steel family")
            candidate=design.materials.addByCopy(source,name)
        b.material=candidate
        if b.material.name!=name or not .0077<b.physicalProperties.mass/b.volume<.0081:
            raise ValueError("Could not bind documented steel family")
    fixed="340 mm pan fixed installation:1"
    support_name="Piloted bilateral bearing support"
    def import_source(design,stock,path):
        parent=design.rootComponent.occurrences.itemByName(fixed).component
        prior={o.fullPathName for o in design.rootComponent.allOccurrences}
        options=app.importManager.createFusionArchiveImportOptions(str(path))
        inserted=app.importManager.importToTarget2(options,parent)
        children=[o for o in design.rootComponent.allOccurrences if o.fullPathName not in prior]
        if not inserted or len(children)!=1:raise ValueError("Expected one editable source component")
        target=children[0]
        target.component.name=support_name
        target.isGroundToParent=True
        if counts(target.component)!=stock["features"] or target.component.bRepBodies.count!=1:
            raise ValueError("Machined feature history changed")
        if target.component.bRepBodies.item(0).material.name!=stock["material"]:
            raise ValueError("The diaphragm material changed")
    def import_race(design,name,file,matrix,reference):
        other=None
        try:
            other=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(file)))
            source=fusion.Design.cast(other.products.itemByProductType("DesignProductType"))
            solids=[(b,None) for b in source.rootComponent.bRepBodies]
            solids.extend((b,o) for o in source.rootComponent.allOccurrences for b in o.component.bRepBodies)
            if len(solids)!=1 or not solids[0][0].isSolid:raise ValueError("Genuine race must transfer as one solid")
            raw,o=solids[0]
            actual=raw.createForAssemblyContext(o) if o else raw
            measured=actual.preciseBoundingBox
            bounds=[10*x for p in (measured.minPoint,measured.maxPoint) for x in p.asArray()]
            if max(abs(x-y) for x,y in zip(bounds,reference["boundsMm"]))>.006:
                raise ValueError("Original supplier STEP frame changed "+str(bounds))
            mgr=fusion.TemporaryBRepManager.get()
            native=mgr.copy(raw)
            if o and not mgr.transform(native,o.transform2):raise ValueError("Could not retain source root frame")
        finally:
            if other:other.close(False)
            design.parentDocument.activate()
        world=core.Matrix3D.create()
        world.setWithArray(matrix)
        comp=design.rootComponent.occurrences.itemByName(fixed).component
        # The guard independently checks the attained root transform and all neighbors.
        occ=comp.occurrences.addNewComponent(world)
        occ.component.name=name
        occ.isGroundToParent=True
        child=occ.component
        feat=child.features.baseFeatures.add()
        feat.name="Exact supplier BRep from 6655K17 housing washer"
        feat.startEdit()
        try:
            body=child.bRepBodies.add(native,feat)
            if not body:raise ValueError("Could not transfer genuine supplier BRep")
            body.name=name
        finally:feat.finishEdit()
        b=child.bRepBodies.item(0)
        b.name=name
        freeze_material(design,b)
        v=1000*b.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume
        if abs(v-reference["volumeMm3"])>.01:raise ValueError("Genuine race material volume differs "+str(v))
        child.attributes.add("Source","exact STEP",reference["sha256"])
    doc=None
    try:
        for f,sig in CONFIG["inputs"].items():
            if sha(root/f)!=sig:raise ValueError("Changed frozen input "+f)
        authority=module(Path(CONFIG["plugin"])/"fusion_authority.py","issue_authority")
        report["authority"]=authority.verify(root,root/"cad/fusion-authority.json",root/"artifacts/native-master-review/master-issue-03/lineage.json")
        prior=t/"machined-only/machined-preinsertion-294.f3d"
        issued=read(t/"machined-only/pan-r3-machined-only-01.json")
        sources=read(t/"guarded-support/pan-r3-checked-support-01.json")["sources"]
        stock=sources[support_name];article=t/"guarded-support"/stock["archive"]
        if sha(prior)!=issued["archiveSha256"] or sha(article)!=stock["sha256"]:raise ValueError("Parent or article source changed")
        study=read(t/"r3-housing-races-seated.json")
        if len(study["installed"])!=2 or study["machinedNativeSha256"]!=read(t/"native/pan-r3-stock-machining-01.json")["nativeSha256"]:
            raise ValueError("Missing paired seated-race evidence")
        initial=stock["world"]
        order=[(support_name,initial,lambda design:import_source(design,stock,article),"Starboard bearing tower",
                "Four integral support bosses seat on starboard tower Y=-198.15 mm.")]
        for side in ("inboard","outboard"):
            proof=study["installed"][side];source=proof["source"];ref=source["originalTransfer"]
            if proof["finalSeatGapMm"]>.001 or proof["finalSeatCommonMm3"]>.001:raise ValueError("Source seat fails "+side)
            file=root/ref["file"]
            if sha(file)!=ref["sha256"]:raise ValueError("Missing genuine housing race")
            rows=[row[:] for row in source["matrixMm"]]
            rows[1][3]+=proof["fixedShiftYmm"]
            matrix=[c if j!=3 else c/10 for row in rows for j,c in enumerate(row)]+[0.,0.,0.,1.]
            name="6655K17 "+side+" stationary housing race"
            def install(design,n=name,f=file,m=matrix,r=ref):return import_race(design,n,f,m,r)
            order.append((name,matrix,install,support_name,"Real housing washer seated on its correct diaphragm face at axial limit with original OD pilot."))
        for i,(name,matrix,install,against,basis) in enumerate(order,1):
            report["stage"]="check "+name
            save()
            path=fixed+"+"+name+":1"
            request=dict(occurrence=path,worldTransformCm=matrix,interfaces=[dict(kind="bearing",maxGapMm=.001,
                bodies=[[path,name],[fixed+"+"+against+":1",against]],basis=basis)])
            file=out/(f"checked-{294+i}.f3d")
            NATIVE_JOB["verifyDependencies"]()
            item=guard.checked_placement(app,prior,sha(prior),file,request,install)
            report["placements"].append(item);save()
            if item["status"]!="pass":raise ValueError("Placement held: "+name)
            doc,design=cold(file)
            point=base.pose(design)
            base.check_stow(point)
            installed=base.census(design)
            item["cold"]=dict(solids=len(installed),floorTipMm=point["landmarkWorldMm"],stowRad=point["jointRad"])
            if len(installed)!=294+i or path+"/"+name not in installed:raise ValueError("Cold issue roster changed")
            if i==1 and counts(base.component(design,name).component)!=stock["features"]:raise ValueError("Lost editable diaphragm features")
            save()
            if i==len(order):
                step=file.with_suffix(".step")
                if step.exists() or not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
                    raise ValueError("Could not issue cold derivative")
                report.update(stepSha256=sha(step),archiveSha256=sha(file))
            doc.close(False);doc=None
            prior=file
        report["status"]="fixed support and two housing races seated"
    except Exception:
        report.update(status="failed",error=traceback.format_exc())
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if active and active.isValid:active.activate()
        report["restored"]=before==[d.creationId for d in app.documents]
        save()
    print(json.dumps({k:report[k] for k in ("stage","status","restored","archiveSha256","stepSha256","error") if k in report}))
