"""Replace only the seven revised fabrications on a separately saved and checked native parent."""
from hashlib import sha256
from pathlib import Path
import json
import traceback
import types
from adsk import core,fusion


def module(path):
    m=types.ModuleType(path.stem)
    m.__file__=str(path)
    exec(compile(path.read_bytes(),str(path),"exec"),m.__dict__)
    return m


def run(context):
    app=core.Application.get()
    prior=app.activeDocument
    before=[d.creationId for d in app.documents]
    doc=None
    root=Path(CONFIG["jobInputs"])
    folder=Path(CONFIG["resultDirectory"])
    chassis=root/"iterations/002/design/chassis"
    out={"before":before,"status":"starting"}
    def archive(des,path):
        if path.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(path))):
            raise ValueError("Native save failed "+str(path))
        progress.checkpoint(path)
    def opened(path):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
    def census(des):
        result={}
        for occ in des.rootComponent.allOccurrences:
            for b in occ.component.bRepBodies:
                key=(occ.fullPathName,b.name)
                if key in result or not b.isSolid:raise ValueError("Duplicate or non-solid "+str(key))
                bb=b.createForAssemblyContext(occ).preciseBoundingBox
                pp=b.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                result[key]={"material":b.material.name,"mm3":pp.volume*1000,
                             "mm":[10*n for p in (bb.minPoint,bb.maxPoint) for n in p.asArray()]}
        return result
    def unchanged(prior_rows,now):
        if len(prior_rows)!=647:raise ValueError("Incorrect retained roster")
        for k,a in prior_rows.items():
            b=now[k]
            if a["material"]!=b["material"] or abs(a["mm3"]-b["mm3"])>.025 or max(abs(i-j) for i,j in zip(a["mm"],b["mm"]))>.006:
                raise ValueError("Retained source body changed "+str(k))
    try:
        if before or prior:raise ValueError("Expected vacant Fusion station")
        NATIVE_JOB["verifyDependencies"]()
        plan=json.loads((chassis/"connected-seven-input.json").read_bytes())
        auth=module(Path(CONFIG["plugin"])/"fusion_authority.py")
        register=auth.register_path(root)
        if auth.verify(root,register,register.parent/plan["lineage"])!=plan["authority"]:raise ValueError("Canonical parent changed")
        src=root/plan["archive"]["file"]
        if plan["authority"]["archiveSha256"] != plan["archive"]["sha256"] or sha256(src.read_bytes()).hexdigest()!=plan["archive"]["sha256"]:raise ValueError("Machining checkpoint changed")
        changed=json.loads((chassis/"inset-corrected-brace-foot-clearance.json").read_bytes())["stockChanges"]
        expected=plan["expectedUnthreaded"]
        if set(expected)!=set(changed) or len(expected)!=7:raise ValueError("Expected seven changed forms")
        progress=module(chassis/"native_job_telemetry.py").NativeProgress(folder,src)
        tools=module(chassis/"native_corner.py")
        forming=module(chassis/"native_connected_chassis.py")
        def stage(num,label,article):return progress.operation(label,article,num,7)
        with progress.operation("cold open predecessor and index",src.name,1,1):
            doc=opened(src);des=tools.design_of(doc);assembly=des.rootComponent
            original=census(des)
            if len(original)!=654:raise ValueError("Expected 654 predecessor bodies")
            old=assembly.occurrences.itemByName("Red 2 inset structural machining:1")
            rows={occ.component.name:occ for occ in old.childOccurrences}
            removed={(rows[n].fullPathName,n) for n in changed}
            if not removed<=original.keys():raise ValueError("Missing original forms")
            kept={key:r for key,r in original.items() if key not in removed}
        for i,n in enumerate(changed,1):
            with stage(i,"remove prior occurrence",n):
                if not (rows[n].nativeObject or rows[n]).deleteMe():raise ValueError("Could not remove "+n)
        with progress.operation("save 647-body machining base","frame",1,1):
            if not des.computeAll():raise ValueError("Regeneration failed")
            unchanged(kept,census(des))
            archive(des,folder/"diagnostic-647.f3d")
        root_holder=assembly.occurrences.addNewComponent(core.Matrix3D.create())
        root_holder.component.name="Serviceable brace revision"
        mat=des.materials.itemByName("002 machined 6061-T6")
        if not mat:raise ValueError("Exact parent material missing")
        for i,(name,row) in enumerate(changed.items(),1):
            with stage(i,"form revision",name):
                forming.machine(root_holder.component,row,mat,tools,expected[name]["alreadyRemovedCutIndices"])
        root_holder.isGrounded=True
        with progress.operation("save formed seven before assembly diagnosis","frame",1,1):
            if des.snapshots.hasPendingSnapshot:des.snapshots.add()
            if not des.computeAll():raise ValueError("Regeneration failed")
            archive(des,folder/"diagnostic-seven.f3d")
        with progress.operation("cold reopen and compare all retained bodies","frame",1,1):
            doc.close(False);doc=None;doc=opened(folder/"diagnostic-seven.f3d")
            des=tools.design_of(doc);actual=census(des);unchanged(kept,actual)
            if len(actual)!=654:raise ValueError("Unexpected successor count")
            holder=des.rootComponent.occurrences.itemByName("Serviceable brace revision:1")
            if not holder or not holder.isGrounded:raise ValueError("Revised assembly root not fixed")
            result={}
            for o in holder.childOccurrences:
                name=o.component.bRepBodies.item(0).name
                r=actual[(o.fullPathName,name)]
                target=expected[name]
                result[name]={"worldGapMm":max(abs(x-y) for x,y in zip(r["mm"],target["bounds"])),
                              "volumeDeltaMm3":r["mm3"]-target["volumeMm3"],"material":r["material"],
                              "groundToParent":(o.nativeObject or o).isGroundToParent}
            if set(result)!=set(expected):raise ValueError("Changed successor roles")
            out.update(changed=result,unchangedCount=len(kept),bodyCount=len(actual))
            for n,r in result.items():
                if r["worldGapMm"]>.025 or abs(r["volumeDeltaMm3"])>max(.08,expected[n]["volumeMm3"]*.00002) or not r["groundToParent"] or r["material"]!="002 machined 6061-T6":raise ValueError("Held revised form "+n)
        with progress.operation("derive neutral diagnosis","frame",1,1):
            step=folder/"seven-unthreaded.step"
            if step.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):
                raise ValueError("Diagnostic STEP save failed")
            out["diagnosticStepSHA256"]=sha256(step.read_bytes()).hexdigest()
        out.update(status="seven-stock machining checkpoint",limits="Unthreaded cutting stage, preceding the modeled taps, purchased screws, placement and service reviews.")
    except Exception:
        out.update(status="held",exception=traceback.format_exc())
    finally:
        if doc:doc.close(False)
        out["after"]=[d.creationId for d in app.documents]
        out["restored"]=out["after"]==before and app.activeDocument is None
        (folder/"native-receipt.json").write_text(json.dumps(out,indent=1)+"\n")
    print(json.dumps({k:out.get(k) for k in ("status","restored")}))
