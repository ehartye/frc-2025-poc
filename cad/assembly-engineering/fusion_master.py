"""Author, export or independently reopen the material-assigned shallow-cage alternate in Fusion."""

from pathlib import Path
import csv
import hashlib
import json
import adsk
import adsk.core as core
import adsk.fusion as fusion

CONFIGURATION="fixed-low-captive-shallow-service-1"
FILENAME="Team-1780-engineering-shallow.f3d"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open(encoding="utf8",newline="") as file:
        return list(csv.DictReader(file))


def key_for(occ,body):
    return body.name if body.name!="Body1" else occ.component.name


def physical_observations(d):
    physical=[]
    for occ in d.rootComponent.allOccurrences:
        for body in occ.bRepBodies:
            n=body.nativeObject or body
            attrs={i.name:i.value for i in n.attributes if i.groupName=="Team 1780 engineering"}
            props=n.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
            proxy=body.physicalProperties
            if not attrs:raise ValueError("Absent saved identity "+occ.fullPathName+" "+body.name)
            row=dict(occurrence=occ.fullPathName,body=body.name,identity=attrs["identity"],
                instance=attrs["instance"] or None,materialId=body.material.id,materialName=body.material.name,
                volumeCm3=props.volume,massKg=props.mass,brepVolumeCm3=n.volume,defaultProxyVolumeCm3=proxy.volume)
            if attrs.get("stock"):row["stock"]=attrs["stock"]
            bb=body.preciseBoundingBox
            row["boundsMm"]=[10*v for pt in (bb.minPoint,bb.maxPoint) for v in pt.asArray()]
            physical.append(row)
    return sorted(physical,key=lambda r:(r["occurrence"],r["body"]))


def acquire_shapes(app,path):
    doc=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(path)))
    try:
        design=fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        out={}
        cache=fusion.TemporaryBRepManager.get()
        for occ in design.rootComponent.allOccurrences:
            for body in occ.bRepBodies:
                name=occ.component.name
                if name in out:raise ValueError("Duplicate imported body "+name)
                out[name]=cache.copy(body)
        for body in design.rootComponent.bRepBodies:
            if body.name in out:raise ValueError(body.name)
            out[body.name]=cache.copy(body)
        return out
    finally:
        doc.close(False)


def insert(c,shape,name):
    if c.parentDesign.designType==fusion.DesignTypes.ParametricDesignType:
        f=c.features.baseFeatures.add()
        f.name="Shop exchange "+name
        f.startEdit()
        n=c.bRepBodies.add(shape,f)
        n.name=name
        f.finishEdit()
        return f.bodies.item(0)
    n=c.bRepBodies.add(shape)
    n.name=name
    return n


def run(context):
    cfg=json.loads(Path(r"INPUT_MANIFEST_JSON").read_text(encoding="utf-8-sig"))
    root=Path(cfg["project"])
    folder=root/"cad/assembly-engineering"
    out=Path(cfg["out"])
    app=core.Application.get()
    stage=cfg["stage"]
    rows={r["id"]:r for r in read_csv(folder/"material-register.csv")}
    path=out/FILENAME
    source=root/"cad/shallow-cage/Team-1780-shallow-cage-study.f3d"
    change=json.loads((folder/"interfaces.json").read_bytes())
    if digest(source)!=change["sourceArchiveSha256"]:raise ValueError("Initial archive changed")
    if stage in ("create","patch"):
        kit_path=folder/"station-and-jack-interfaces.step"
        if digest(kit_path)!=change["outputSha256"]:raise ValueError("Change package mismatch")
        kit=acquire_shapes(app,kit_path)
        if set(kit)!=set(change["replacement"]+change["additional"]):raise ValueError(("Imported changes have different IDs",list(kit)[:20]))
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source if stage=="create" else Path(cfg.get("fromArchive",path)))))
    try:
        d=fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        top=d.rootComponent
        if stage in ("create","patch"):
            old={}
            for occ in top.allOccurrences:
                for b in occ.component.bRepBodies:
                    key=key_for(occ,b)
                    if key in old:raise ValueError(("Duplicate issued body",key))
                    old[key]=(occ,b)
            lib=app.materialLibraries.itemByName("Fusion Material Library")
            colors={ap.name:ap for ap in d.appearances}
            fixed=next(o.component for o in top.occurrences if o.component.name=="Fixed chassis")
            matcache={}
            for key in change["fixedHeight"]["removed"]:
                if key in old:old[key][0].deleteMe()
            for i,(k,r) in enumerate(rows.items()):
                if i%100==0:(out/"progress.txt").write_text(str(i)+" "+k)
                if stage=="patch" and k not in kit and k not in cfg.get("changedMaterials",[]):
                    continue
                if k in old:
                    o,b=old[k]
                    ap=b.appearance
                    c=o.component
                else:
                    o=fixed.occurrences.addNewComponent(core.Matrix3D.create())
                    c=o.component
                    c.name=k
                    ap=next((v for key,v in colors.items() if key in ("FRC 2025 - fastener","FRC 2025 - plate")),None)
                    b=None
                if k in kit:
                    if b is not None:
                        c.features.removeFeatures.add(b)
                    local=fusion.TemporaryBRepManager.get().copy(kit[k])
                    inverse=o.transform2.copy()
                    inverse.invert()
                    fusion.TemporaryBRepManager.get().transform(local,inverse)
                    b=insert(c,local,k)
                matname=r["physicalDisplayMaterial"]
                if stage=="patch" and matname not in matcache:
                    current=d.materials.itemByName(matname)
                    if current:matcache[matname]=current
                if matname not in matcache:
                    mat=lib.materials.itemByName(matname)
                    if not mat:
                        ref=lib.materials.itemByName("Polyethylene, Low Density")
                        mat=d.materials.addByCopy(ref,matname)
                        for prop in mat.materialProperties:
                            if prop.id in ("structural_Density","thermal_Density"):
                                target=core.FloatProperty.cast(prop)
                                if target.units!="KilogramPerCubicMeter":raise ValueError((target.units,matname))
                                target.value=float(r["nativeNominalDensityKgM3"])
                    matcache[matname]=mat
                b.material=matcache[matname]
                if ap:b.appearance=ap
                # The independent reopener reads persistent attributes, volume and occurrence;
                # it does not infer the installed article from a public label or body order.
                identity=hashlib.sha256(("team-1780-engineering-v1/"+k).encode()).hexdigest()
                for key,value in {"identity":identity,"instance":r["instance"],"stock":matname if r["kind"]=="made-stock" else ""}.items():
                    b.attributes.add("Team 1780 engineering",key,value)
            if not d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(path))):
                raise RuntimeError("Native archive export failed")
        else:
            body_rows=physical_observations(d)
            if len(body_rows)!=len(rows):raise ValueError(("Incomplete observation",len(body_rows),len(rows)))
            report={"schema":"agent-frc/native-physical-properties@1","configuration":CONFIGURATION,"opened":True,
                    "archiveSha256":digest(path),"nativeBodyCount":len(body_rows),
                    "rawBodyTotalKg":sum(r["massKg"] for r in body_rows),"rootMassKg":top.physicalProperties.mass,"bodies":body_rows}
            (out/"engineering-reopen.json").write_bytes((json.dumps(report,indent=2)+"\n").encode())
            joint={}
            for j in top.asBuiltJoints:
                motion=j.jointMotion
                jm=fusion.SliderJointMotion.cast(motion)
                rotation=fusion.RevoluteJointMotion.cast(motion)
                ends={"Lift crosshead":319.4,"Spread left cage shoe":36,"Spread right cage shoe":-36,"Preload cage plate":13.584}
                record=dict(one=j.occurrenceOne.fullPathName,two=j.occurrenceTwo.fullPathName,type=motion.objectType)
                joint[j.name]=record
                if not jm and not rotation:continue
                end=ends.get(j.name,1 if jm else 10)
                readings=[]
                for pos in (0,end,0):
                    if jm:jm.slideValue=pos/10
                    else:rotation.rotationValue=pos*3.141592653589793/180
                    adsk.doEvents()
                    readings.append(dict(command=pos,readback=jm.slideValue*10 if jm else rotation.rotationValue*180/3.141592653589793,
                        a=j.occurrenceOne.transform2.asArray(),b=j.occurrenceTwo.transform2.asArray()))
                record["unit"]="mm" if jm else "degree"
                record["samples"]=readings
            (out/"engineering-joints.json").write_bytes((json.dumps(joint,indent=2)+"\n").encode())
            view=app.activeViewport
            cam=view.camera
            cam.viewOrientation=core.ViewOrientations.IsoTopRightViewOrientation
            cam.isSmoothTransition=False
            cam.isFitView=True
            view.camera=cam
            view.refresh()
            view.saveAsImageFile(str(out/"engineering-native.png"),1280,850)
    finally:
        doc.close(False)
        (out/("closed-"+stage+".txt")).write_text("Closed assigned engineering document.")
