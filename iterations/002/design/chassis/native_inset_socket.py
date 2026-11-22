"""Cold-issue the first fitted internal shoe and inset wheel on the 552 source corner."""
from pathlib import Path
import json
import traceback
from adsk import core, fusion


def read_module(f):
    import types
    m=types.ModuleType(f.stem);m.__file__=str(f)
    exec(compile(f.read_bytes(),str(f),"exec"),m.__dict__)
    return m


def run(context):
    app=core.Application.get()
    project=Path(CONFIG["jobInputs"]);out=Path(CONFIG["resultDirectory"])
    study=project/"iterations/002/design";scripts=Path(CONFIG["plugin"])
    stock=read_module(study/"chassis/native_corner.py")
    auth=read_module(scripts/"fusion_authority.py")
    guard=read_module(scripts/"fusion_checked_placement.py")
    baseline=[d.creationId for d in app.documents]
    report=dict(before=baseline,stages=[])
    plan=json.loads((study/"chassis/inset-socket-shoe.json").read_bytes())
    group_name="Port socket and fitted load spreader"
    source_name="Front left SDS MK4i L2:1"
    def save(stage):
        report["stage"]=stage
        (out/"receipt.json").write_text(json.dumps(report,indent=2)+"\n")
    def open_native(p):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
    def export(des,p,comp=None):
        if p.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(p),comp or des.rootComponent)):
            raise ValueError("Failed new archive "+str(p))
        return p
    def write_stock(component,name,steps,material):
        occ=component.occurrences.addNewComponent(core.Matrix3D.create());c=occ.component;c.name=name;body=None
        for step_name,axis,start,travel,regions,kind in steps:
            plane,sign=stock.plane(c,axis,start);sk=c.sketches.add(plane);sk.name=step_name
            def xyz(uv):
                a=list(uv);a.insert(axis,start)
                return sk.modelToSketchSpace(stock.point(a))
            for a in regions:
                if len(a)==3:sk.sketchCurves.sketchCircles.addByCenterRadius(xyz(a[:2]),a[2]/10)
                else:sk.sketchCurves.sketchLines.addTwoPointRectangle(xyz(a[:2]),xyz(a[2:]))
            profiles=core.ObjectCollection.create()
            for p in sk.profiles:profiles.add(p)
            if profiles.count!=len(regions):raise ValueError("Unexpected closed sketch "+step_name)
            extr=c.features.extrudeFeatures
            mode={"new":fusion.FeatureOperations.NewBodyFeatureOperation,"cut":fusion.FeatureOperations.CutFeatureOperation,
                  "join":fusion.FeatureOperations.JoinFeatureOperation}[kind]
            req=extr.createInput(profiles,mode)
            if kind=="cut":req.participantBodies=[body]
            req.setDistanceExtent(False,core.ValueInput.createByReal(travel/sign/10))
            extr.add(req).name=step_name
            if c.bRepBodies.count!=1:raise ValueError("Expected one editable stock: "+name+"/"+step_name)
            body=c.bRepBodies.item(0);sk.isLightBulbOn=False
        body.name=name;body.material=material;occ.isGrounded=True
        return occ
    def introduce_stock(des):
        root=des.rootComponent;occ=root.occurrences.addNewComponent(core.Matrix3D.create())
        group=occ.component;group.name=group_name
        material=next((m for lib in app.materialLibraries for m in lib.materials if '6061' in m.name and 't6' in m.name.lower().replace('-','')),None)
        if not material:raise ValueError("Native 6061-T6 stock unavailable")
        inside=des.materials.itemByName("002 unrated 6061-T6 structural stock") or des.materials.addByCopy(material,"002 unrated 6061-T6 structural stock")
        lo=43.83336287672979;hi=94.63336287672979;cy=plan["pinAxisMm"][1];pinx=plan["pinAxisMm"][0]
        xs=plan["axesMm"];left,right=xs[0]-7.,xs[-1]+5.35;yy0,yy1=cy-9.475,cy+9.475
        # All six module axes are retained; the second end's module is outside this first mounted issue.
        rail=[("Saw port rail with unchanged end datum",0,-227.3,454.6,[(cy-12.7,lo,cy+12.7,hi)],"new"),
              ("Hollow 1 x 2 x .125 inch tube",0,-228.,456.,[(cy-9.525,lo+3.175,cy+9.525,hi-3.175)],"cut"),
              ("Drill six SDS through axes",2,lo-1,hi-lo+2,[(x,cy,2.65) for x in xs+[-v for v in xs]],"cut"),
              ("Line ream positive axial key",2,lo-1,hi-lo+2,[(pinx,cy,5.025)],"cut")]
        write_stock(group,"Inset port socket rail",rail,inside)
        for side,k in (("lower",1),("upper",-1)):
            z=lo+3.2 if k==1 else hi-3.2
            steps=[("Mill continuous shoe land",2,z,k*4.25,[(left,yy0,right,yy1)],"new"),
                   ("Keep thick inner pin bearing",2,z+k*4.25,k*4.75,[(left,yy0,xs[0]+7,yy1)],"join"),
                   ("Keep outer pin bearing",2,z+k*4.25,k*1.75,[(xs[-1]-6.5,yy0,right,yy1)],"join"),
                   ("Drill three reamed supplier screw holes",2,z-k,k*11.,[(x,cy,2.49) for x in xs],"cut"),
                   ("Ream positive shear pin",2,z-k,k*11.,[(pinx,cy,5.025)],"cut")]
            write_stock(group,side+" fitted socket shoe",steps,inside)
        pin=[("Turn load pin to size",2,lo+.1,hi-lo-.2,[(pinx,cy,5.)],"new"),
             ("Bore lightweight core",2,lo-.5,hi-lo+1.,[(pinx,cy,2.4)],"cut")]
        write_stock(group,"Captured hollow load pin",pin,inside)
        occ.isGrounded=True
    try:
        if baseline or app.activeDocument:raise ValueError("Expected reserved empty Fusion station")
        NATIVE_JOB["verifyDependencies"]()
        reg=auth.register_path(project);line=reg.parent/"issue-005/lineage.json"
        report["authority"]=auth.verify(project,reg,line)
        if report["authority"]!=plan["authority"]:raise ValueError("Wrong canonical source")
        save("read actual source FL module")
        doc=open_native(auth.resolve(project,auth.load(line)["archive"]));des=stock.design_of(doc)
        original=des.rootComponent.occurrences.itemByName(source_name)
        if not original:raise ValueError("Missing true installed source module")
        world=original.transform2.copy();expected=stock.world_body_rows(des)
        expected=[r for r in expected if r["path"].startswith(source_name+"+")]
        if len(expected)!=121:raise ValueError("Incomplete original fitted module")
        delta=world.translation;delta.x-=1.27;delta.y-=1.27;world.translation=delta
        leaf=export(des,out/"module.f3d",original.component);doc.close(False)
        doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType);des=stock.design_of(doc)
        des.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType
        root=des.rootComponent
        if not app.importManager.importToTarget(app.importManager.createFusionArchiveImportOptions(str(leaf)),root):
            raise ValueError("Could not reinstantiate the registered module")
        if root.occurrences.count!=1:raise ValueError("Module staged incorrectly")
        item=root.occurrences.item(0);item.component.name="Inset front left SDS L2"
        target=item.fullPathName
        seed=export(des,out/"source-local.f3d");doc.close(False)
        def place(des):
            o=des.rootComponent.occurrences.itemByName(target)
            o.isGrounded=False;o.transform2=world;o.isGrounded=True
        parent=out/"inset-121.f3d"
        witness=guard.checked_placement(app,seed,stock.sha(seed),parent,dict(occurrence=target,worldTransformCm=list(world.asArray())),place)
        report["stages"].append(witness);save("placed configured source module")
        if witness["status"]!="pass":raise ValueError("Source module placement held")
        nxt=out/"inset-125.f3d";identity=core.Matrix3D.create()
        witness=guard.checked_placement(app,parent,stock.sha(parent),nxt,dict(occurrence=group_name+":1",worldTransformCm=list(identity.asArray())),introduce_stock)
        report["stages"].append(witness);save("added editable port load path")
        if witness["status"]!="pass":raise ValueError("Installed socket shoe held")
        doc=open_native(nxt);des=stock.design_of(doc);rows=stock.world_body_rows(des)
        if len(rows)!=125:raise ValueError("Unexpected final installed stock")
        old={(r["path"].split("+",1)[1],r["name"]):r for r in expected}
        report["supplierReadback"]=[]
        for r in rows:
            if not r["path"].startswith(target+"+"):continue
            prior=old.pop((r["path"].split("+",1)[1],r["name"]))
            measured=r["boundsMm"];aim=[v-(12.7 if i%3<2 else 0) for i,v in enumerate(prior["boundsMm"])]
            error=max(abs(a-b) for a,b in zip(measured,aim))
            if error>.01 or abs(prior["volumeMm3"]-r["volumeMm3"])>.06 or prior["material"]!=r["material"]:
                raise ValueError("Wrong placed article "+r["path"]+" "+str(error))
            report["supplierReadback"].append(dict(name=r["name"],path=r["path"],boundErrorMm=error))
        if old:raise ValueError("Missing source readbacks")
        report["stock"]=[r for r in rows if r["path"].startswith(group_name+":1+")]
        mass={r["name"]:r["volumeMm3"]*2.7e-6 for r in report["stock"]}
        for name,key in (("lower fitted socket shoe","bottom"),("upper fitted socket shoe","top"),("Captured hollow load pin","pin")):
            if abs(mass[name]-plan["oneStationKg"][key])>1e-5:raise ValueError("Independent stock difference "+name)
        step=out/"inset-125.step"
        if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError("Cold export failed")
        report.update(status="cold 125-body inset corner with positive socket key",f3dSha256=stock.sha(nxt),stepSha256=stock.sha(step))
    except Exception:
        report.update(status="held",error=traceback.format_exc())
    finally:
        for doc in reversed(list(app.documents)):
            if doc.creationId not in baseline:doc.close(False)
        report["restored"]=app.documents.count==0 and app.activeDocument is None
        save("terminal")
    print(json.dumps({k:report[k] for k in ("status","f3dSha256","stepSha256","restored","error") if k in report}))

