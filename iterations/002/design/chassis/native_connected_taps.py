"""Machine two existing nutbars in place, retaining the unfinished parent and full rooted model."""
from pathlib import Path
import json,traceback,types
from adsk import core, fusion

def module(p):
    m=types.ModuleType(p.stem);m.__file__=str(p)
    exec(compile(p.read_bytes(),str(p),"exec"),m.__dict__)
    return m

def run(context):
    app=core.Application.get();start=[d.creationId for d in app.documents];doc=None
    root=Path(CONFIG["jobInputs"]);out=Path(CONFIG["resultDirectory"]);h=root/"iterations/002/design/chassis"
    r={"before":start,"status":"working","nutbars":[]}
    try:
        if start or app.activeDocument:raise ValueError("Home/0 required")
        NATIVE_JOB["verifyDependencies"]()
        authority=module(Path(CONFIG["plugin"])/"fusion_authority.py");reg=authority.register_path(root)
        specification=json.loads((h/"connected-thread-input.json").read_bytes())
        base=authority.verify(root,reg,reg.parent/specification["lineage"])
        if base!=specification["authority"]:raise ValueError("Canonical starting assembly changed")
        issue=specification["source"];parent=root/issue["file"]
        prior=json.loads((h/"connected-seven-stock-receipt.json").read_bytes())
        if prior.get("status")!="seven-stock machining checkpoint" or prior["unchangedCount"]!=647 or prior["bodyCount"]!=654 or not prior["restored"]:raise ValueError("Unclosed seven-form checkpoint")
        if authority.digest(parent)!=issue["sha256"]:raise ValueError("Seven-form guarded checkpoint changed")
        read=module(h/"native_job_telemetry.py").NativeProgress(out,parent)
        def opened(p):
            return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
        def des_of(d):
            return fusion.Design.cast(d.products.itemByProductType("DesignProductType"))
        def bars(des):
            target=des.rootComponent.occurrences.itemByName("Serviceable brace revision:1")
            if not target or not target.isGrounded:raise ValueError("Missing installed revised assembly root")
            roster={o.component.bRepBodies.item(0).name:o for o in target.childOccurrences}
            return {n:roster[n] for n in specification["tapHoles"]}
        with read.operation("cold open accepted seven-stock parent",parent.name,1,1):
            doc=opened(parent);des=des_of(doc);items=bars(des)
            if sum(o.component.bRepBodies.count for o in des.rootComponent.allOccurrences)!=654:raise ValueError("Predecessor roster differs")
        for i,(name,loc) in enumerate(items.items(),1):
            with read.operation("cut two actual 1/4-20 UNC class 2B female threads",name,i,2):
                if not (loc.nativeObject or loc).isGroundToParent:raise ValueError("Loose receiver")
                comp=loc.component;solid=comp.bRepBodies.item(0);faces=core.ObjectCollection.create();observed=[]
                for f in solid.faces:
                    c=core.Cylinder.cast(f.geometry)
                    if not c or abs(abs(c.axis.z)-1)>.000001 or abs(c.radius*10-2.5527)>.00001:continue
                    observed.append([c.origin.x*10,c.origin.y*10]);faces.add(f)
                goals=specification["tapHoles"][name]
                if len(observed)!=2 or max(min(sum((x-y)**2 for x,y in zip(q,g)) for q in observed)**.5 for g in goals)>.002:raise ValueError("Wrong receiving holes")
                cutter=comp.features.threadFeatures
                setting=cutter.createThreadInfo(True,"ANSI Unified Screw Threads","1/4-20 UNC","2B")
                req=cutter.createInput(faces,setting);req.isModeled=True;req.isFullLength=True
                feature=cutter.add(req);feature.name="1/4-20 UNC-2B full-length brace screw threads"
                if not feature.isModeled or comp.bRepBodies.count!=1:raise ValueError("Threads not cut as modeled")
                r["nutbars"].append(dict(name=name,axesXYmm=observed,modeled=feature.isModeled,fullLength=feature.isFullLength))
        with read.operation("save tapped receivers before full diagnosis","frame",1,1):
            if not des.computeAll():raise ValueError("Regeneration failed")
            f3d=out/"four-tapped-holes.f3d"
            if f3d.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(f3d))):raise ValueError("Archive save failed")
            read.checkpoint(f3d)
        with read.operation("cold verify modeled receivers and derive for phase calibration","frame",1,1):
            doc.close(False);doc=None;doc=opened(f3d);des=des_of(doc);items=bars(des)
            for name,o in items.items():
                comp=o.component;feats=comp.features.threadFeatures
                if feats.count!=1 or not feats.item(0).isModeled or not feats.item(0).isFullLength:raise ValueError("Cold modeled tap lost "+name)
            step=out/"four-tapped-holes.step"
            if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError("Cold STEP export failed")
            r.update(status="four modeled holes; seated purchased screw phase pending",nativeSha256=authority.digest(f3d),stepSha256=authority.digest(step))
    except Exception:r.update(status="held",exception=traceback.format_exc())
    finally:
        if doc:doc.close(False)
        r["after"]=[d.creationId for d in app.documents];r["restored"]=r["after"]==start and app.activeDocument is None
        (out/"native-receipt.json").write_text(json.dumps(r,indent=1)+"\n")
    print(json.dumps({k:r.get(k) for k in ("status","restored")}))
