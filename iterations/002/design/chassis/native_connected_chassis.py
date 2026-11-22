"""Cold-register the matched commercial leaves and form the connected Red 2 chassis machining checkpoint."""
import json,math,traceback,types
from pathlib import Path
import adsk
from adsk import core,fusion


def module(path):
    m=types.ModuleType(path.stem);m.__file__=str(path)
    exec(compile(path.read_bytes(),str(path),"exec"),m.__dict__)
    return m


def sketch_profile(sk,axis,z,p,stock):
    def point(uv):
        a=list(uv);a.insert(axis,z)
        return sk.modelToSketchSpace(stock.point(a))
    def edge(a,b):sk.sketchCurves.sketchLines.addByTwoPoints(point(a),point(b))
    if p["kind"]=="circle":
        sk.sketchCurves.sketchCircles.addByCenterRadius(point(p["center"]),p["radius"]/10);return
    if p["kind"]=="polygon":
        pts=p["points"]
        for a,b in zip(pts,pts[1:]+pts[:1]):edge(a,b)
        return
    x,y,X,Y=p["bounds"];r=p.get("radius",0)
    if not r:
        sk.sketchCurves.sketchLines.addTwoPointRectangle(point([x,y]),point([X,Y]));return
    t=r/math.sqrt(2)
    for a,b in [((x+r,y),(X-r,y)),((X,y+r),(X,Y-r)),((X-r,Y),(x+r,Y)),((x,Y-r),(x,y+r))]:edge(a,b)
    for a,b,c in [((X-r,y),(X-r+t,y+r-t),(X,y+r)),((X,Y-r),(X-r+t,Y-r+t),(X-r,Y)),
                  ((x+r,Y),(x+r-t,Y-r+t),(x,Y-r)),((x,y+r),(x+r-t,y+r-t),(x+r,y))]:
        sk.sketchCurves.sketchArcs.addByThreePoints(point(a),point(b),point(c))


def machine(group,row,mat,stock,already_removed):
    identity=core.Matrix3D.create()
    occ=group.occurrences.addNewComponent(stock.matrix(row["matrixMm"]) if "matrixMm" in row else identity)
    comp=occ.component;comp.name=row["name"];body=None
    for index,step in enumerate(row["steps"]):
        if index in already_removed:continue
        axis=step["axis"];z=step["start"];plane,sign=stock.plane(comp,axis,z)
        sk=comp.sketches.add(plane);sk.name=step["name"]+" profile"
        for p in step["profiles"]:sketch_profile(sk,axis,z,p,stock)
        profiles=core.ObjectCollection.create()
        for profile in sk.profiles:profiles.add(profile)
        if profiles.count!=len(step["profiles"]):raise ValueError("Ambiguous sketch loops "+row["name"]+" / "+step["name"])
        mode={"new":fusion.FeatureOperations.NewBodyFeatureOperation,"join":fusion.FeatureOperations.JoinFeatureOperation,"cut":fusion.FeatureOperations.CutFeatureOperation}[step["kind"]]
        if "offsetXY" in step:
            if axis!=2 or len(step["profiles"])!=1:raise ValueError("Unsupported oblique feature")
            upper,_=stock.plane(comp,axis,z+step["travel"]);other=comp.sketches.add(upper)
            poly=step["profiles"][0];dx,dy=step["offsetXY"]
            sketch_profile(other,axis,z+step["travel"],dict(kind="polygon",points=[[x+dx,y+dy] for x,y in poly["points"]]),stock)
            loft=comp.features.loftFeatures;req=loft.createInput(mode)
            req.loftSections.add(sk.profiles.item(0));req.loftSections.add(other.profiles.item(0))
            if step["kind"]=="cut":req.participantBodies=[body]
            feat=loft.add(req);other.isLightBulbOn=False
        else:
            ext=comp.features.extrudeFeatures;req=ext.createInput(profiles,mode)
            if step["kind"]=="cut":req.participantBodies=[body]
            req.setDistanceExtent(False,core.ValueInput.createByReal(step["travel"]/sign/10))
            feat=ext.add(req)
        feat.name=step["name"];sk.isLightBulbOn=False
        if comp.bRepBodies.count!=1:raise ValueError("Split or missing stock "+row["name"]+" / "+step["name"])
        body=comp.bRepBodies.item(0)
    body.name=row["name"];body.material=mat;occ.isGroundToParent=True
    return occ


def run(context):
    app=core.Application.get();baseline=[d.creationId for d in app.documents];doc=None
    out=Path(CONFIG["resultDirectory"]);proj=Path(CONFIG["jobInputs"]);cad=proj/"iterations/002/design";scripts=Path(CONFIG["plugin"])
    stock=module(cad/"chassis/native_corner.py");guard=module(scripts/"fusion_checked_placement.py");authority=module(scripts/"fusion_authority.py")
    project=json.loads((cad/"chassis/inset-matched-native-plan.json").read_bytes());shop=json.loads((cad/"chassis/inset-chassis-shop.json").read_bytes())
    compare=json.loads((cad/"chassis/inset-stock-check.json").read_bytes())
    donor=cad/"private-inputs/matched-sds/corner.f3d";cold=json.loads((donor.parent/"cold.json").read_bytes())
    report=dict(status="running",before=baseline,checks=[],heldSourceDefects=["steering key fit","simplified belt","thread representation","endplay and lot rating"])
    def save(stage):
        report["stage"]=stage;(out/"native-receipt.json").write_text(json.dumps(report,indent=1)+"\n")
    def reopen(path):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(path)))
    def measured(des):
        records=[]
        for occ in des.rootComponent.allOccurrences:
            for body in occ.component.bRepBodies:
                if not body.isSolid:raise ValueError("Uncommissioned surface "+occ.fullPathName)
                proxy=body.createForAssemblyContext(occ);box=proxy.preciseBoundingBox
                high=body.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                records.append(dict(path=occ.fullPathName,name=body.name,material=body.material.name,volumeMm3=high.volume*1000,
                            defaultVolumeMm3=body.volume*1000,boundsMm=[10*k for v in (box.minPoint,box.maxPoint) for k in v.asArray()]))
        return records
    def archive(des,path):
        if path.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(path))):raise ValueError("Full-root save failed "+str(path))
    def settle(des):
        if des.snapshots.hasPendingSnapshot:des.snapshots.add()
        if not des.computeAll():raise ValueError("Native recompute failed")
    def checked(source,target,req,edit):
        result=guard.checked_placement(app,source,stock.sha(source),target,req,edit)
        report["checks"].append(result);save("guard "+target.stem)
        if result["status"]!="pass":raise ValueError("Placement veto "+target.stem)
    name="Matched SDS L2 module";original=name+":1";correction_keys=tuple({a["path"].partition("+")[0] for a in cold["roster"]})
    def part_id(row):
        path=row["path"];tokens=path.split("+");at=[i for i,s in enumerate(tokens) if s in correction_keys]
        if len(at)!=1:raise ValueError("Unbound purchased path "+path)
        return ("+".join(tokens[at[0]:]),row["name"])
    def witness(observed,expected,material_key):
        read={part_id(r):r for r in observed}
        if len(read)!=len(observed) or len(read)!=127 or len(expected)!=127:raise ValueError("Wrong count inspecting matched module")
        errors=[];issues=[]
        for e in expected:
            r=read[(e["path"],e["body"])]
            gap=max(abs(a-b) for a,b in zip(r["boundsMm"],e[material_key]))
            if gap>.006 or abs(r["volumeMm3"]-e["volumeMm3"])>.025 or r["material"]!=e["material"]:issues.append(dict(path=e["path"],body=e["body"],gapMm=gap,veryHighVolumeChangeMm3=r["volumeMm3"]-e["volumeMm3"],material=r["material"]))
            errors.append(gap)
        if issues:
            report["componentIssues"]=issues;raise ValueError("Changed physical components: "+str(len(issues)))
        return dict(count=len(read),maxWorldBoundsErrorMm=max(errors))
    try:
        if baseline or app.activeDocument or (out/"native-receipt.json").exists():raise ValueError("Fresh, empty native station required")
        NATIVE_JOB["verifyDependencies"]();reg=authority.register_path(proj);lin=reg.parent/"issue-005/lineage.json";own=authority.verify(proj,reg,lin)
        if not own==project["authority"]==shop["authority"]==compare["authority"]:raise ValueError("Stale native or commercial issue")
        if stock.sha(donor)!=project["donorSHA256"] or stock.sha(cad/"chassis/inset-chassis-shop.json")!=compare["shopPlanSHA256"]:raise ValueError("Changed source or stock")
        report["authority"]=own;save("reopen current 552 and read four supplier occurrences")
        doc=reopen(authority.resolve(proj,authority.load(lin)["archive"]));des=stock.design_of(doc);root=des.rootComponent
        names=[f"Front left SDS MK4i L2:{i}" for i in range(1,5)]
        report["priorActualRootPosesCm"]={n:list(root.occurrences.itemByName(n).transform2.asArray()) for n in names}
        for n,corner in zip(names,("FL","FR","RR","RL")):
            expected=stock.matrix(project["poses"][corner]).asArray()
            expected=list(expected)
            for index in (3,7):expected[index]+=math.copysign(1.27,expected[index])
            if max(abs(a-b) for a,b in zip(report["priorActualRootPosesCm"][n],expected))>1e-7:
                raise ValueError("Canonical module pose changed: "+corner)
        doc.close(False);doc=None
        held=json.loads((cad/"chassis/connected-native-02-held.json").read_bytes())
        parent=cad/"private-inputs/matched-sds/four-corners-508.f3d"
        if held["authority"]!=own or held["checks"][-1]["successorSha256"]!=stock.sha(parent):
            raise ValueError("508 cold checkpoint no longer matches its guarded placement")
        if len(held["checks"])!=4 or any(v["status"]!="pass" for v in held["checks"]):
            raise ValueError("Previous corner guards are incomplete")
        report["starting508SHA256"]=stock.sha(parent)
        doc=reopen(parent);rows=measured(stock.design_of(doc))
        if len(rows)!=508:raise ValueError("Lost matched commercial leaf")
        report["508PlacedMaterials"]={label:witness([r for r in rows if r["path"].split("+")[0]==f"{name}:{i}"],project["expected"][label],"expectedBoundsMm") for i,label in enumerate(("FL","FR","RR","RL"),1)}
        doc.close(False);doc=None;save("cold-verified four complete commercial corners")
        groupname="Red 2 inset structural machining"
        def build(d):
            holder=d.rootComponent.occurrences.addNewComponent(core.Matrix3D.create());comp=holder.component;comp.name=groupname
            source=next((m for lib in app.materialLibraries for m in lib.materials if "6061" in m.name and "t6" in m.name.lower().replace("-","")),None)
            if source is None:raise ValueError("Required native 6061-T6 absent")
            mat=d.materials.itemByName("002 machined 6061-T6") or d.materials.addByCopy(source,"002 machined 6061-T6")
            for i,row in enumerate(shop["articles"]):
                try:machine(comp,row,mat,stock,compare["parts"][row["name"]]["alreadyRemovedCutIndices"])
                except Exception:
                    report["failedArticle"]=row["name"]
                    save("failed stock "+row["name"])
                    try: archive(d,out/"unreviewed-partial-machining.f3d")
                    except Exception: report["partialArchiveError"]=traceback.format_exc()
                    raise
                if i%12==0:save("formed stock "+str(i+1)+" / "+str(len(shop["articles"])))
            holder.isGrounded=True;settle(d)
        new=out/"connected-machining.f3d"
        checked(parent,new,dict(occurrence=groupname+":1",worldTransformCm=list(core.Matrix3D.create().asArray())),build)
        doc=reopen(new);des=stock.design_of(doc);rows=measured(des)
        machining=[r for r in rows if r["path"].split("+")[0]==groupname+":1"]
        if len(rows)!=508+len(compare["parts"]) or len(machining)!=len(compare["parts"]):raise ValueError("Wrong installed commercial or stock roster")
        holder=des.rootComponent.occurrences.itemByName(groupname+":1")
        locks=[dict(name=occ.component.name,groundToParent=bool((occ.nativeObject or occ).isGroundToParent))
               for occ in holder.childOccurrences]
        if not holder.isGrounded or len(locks)!=len(shop["articles"]) or not all(r["groundToParent"] for r in locks):
            raise ValueError("Cold fixed fabrication attachment changed")
        report["coldFixedAttachments"]=locks
        report["coldMachining"]=[]
        for row in machining:
            v=compare["parts"][row["name"]];dv=abs(v["volumeMm3"]-row["volumeMm3"]);db=max(abs(i-j) for i,j in zip(v["bounds"],row["boundsMm"]))
            if db>.025 or dv>max(.08,v["volumeMm3"]*.00002) or row["material"]!="002 machined 6061-T6":raise ValueError("Machining or material changed "+str((row["name"],db,dv,row["material"])))
            report["coldMachining"].append(dict(name=row["name"],volumeMm3=row["volumeMm3"],boundsErrorMm=db,volumeErrorMm3=dv))
        step=out/"connected-machining.step"
        if step.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError("STEP derivation failed")
        report.update(status="machining checkpoint",nativeSha256=stock.sha(new),stepSha256=stock.sha(step),coldBodies=len(rows))
    except Exception:
        report.update(status="held",exception=traceback.format_exc())
    finally:
        if doc:doc.close(False)
        report["after"]=[d.creationId for d in app.documents];report["restored"]=report["after"]==baseline and app.activeDocument is None
        save("terminal")
    print(json.dumps(dict(status=report["status"],restored=report["restored"],stage=report["stage"])))

