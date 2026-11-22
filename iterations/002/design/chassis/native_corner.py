"""Build and cold-read the first fresh Red 2 corner inside Fusion."""
import hashlib
import importlib.util
import json
import math
import traceback
from pathlib import Path
import adsk
from adsk import core, fusion


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(rows):
    m=core.Matrix3D.create()
    values=[n/10 if j==3 and i<3 else n for i,row in enumerate(rows) for j,n in enumerate(row)]
    if not m.setWithArray(values):raise ValueError('Invalid factory placement')
    return m


def point(p):
    return core.Point3D.create(*[x/10 for x in p])


def design_of(doc):
    return fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))


def world_body_rows(des):
    rows=[]
    for occ in des.rootComponent.allOccurrences:
        for body in occ.component.bRepBodies:
            if not body.isSolid:raise ValueError('Surface in supplier or stock: '+occ.fullPathName)
            proxy=body.createForAssemblyContext(occ)
            bounds=proxy.preciseBoundingBox
            rows.append(dict(path=occ.fullPathName,name=body.name,
                volumeMm3=1000*body.volume,material=body.material.name,
                boundsMm=[10*k for p in (bounds.minPoint,bounds.maxPoint) for k in p.asArray()]))
    return rows


def plane(comp,axis,coordinate):
    selected=[comp.yZConstructionPlane,comp.xZConstructionPlane,comp.xYConstructionPlane][axis]
    sign=selected.geometry.normal.asArray()[axis]
    if abs(sign)<.999999:raise ValueError('Unexpected construction plane')
    new=comp.constructionPlanes.createInput()
    new.setByOffset(selected,core.ValueInput.createByReal(coordinate/sign/10))
    plane=comp.constructionPlanes.add(new);plane.isLightBulbOn=False
    return plane,sign


def tube(des,rail):
    root=des.rootComponent
    occ=root.occurrences.addNewComponent(core.Matrix3D.create())
    comp=occ.component;comp.name=rail['name'];lo=rail['minimumMm'];hi=rail['maximumMm']
    ax=rail['lengthAxis'];width=1-ax;stock,sgn=plane(comp,ax,lo[ax])
    sketch=comp.sketches.add(stock);sketch.name='2 x 1 x 0.125 inch tube section'
    for inset in (0,rail['wallMm']):
        p=lo.copy();q=hi.copy()
        p[width]+=inset;p[2]+=inset;q[width]-=inset;q[2]-=inset;q[ax]=lo[ax]
        sketch.sketchCurves.sketchLines.addTwoPointRectangle(sketch.modelToSketchSpace(point(p)),sketch.modelToSketchSpace(point(q)))
    profiles=[v for v in sketch.profiles if v.profileLoops.count==2]
    if len(profiles)!=1:raise ValueError('Expected one hollow rectangular section')
    ext=comp.features.extrudeFeatures
    bar=ext.addSimple(profiles[0],core.ValueInput.createByReal((hi[ax]-lo[ax])/sgn/10),fusion.FeatureOperations.NewBodyFeatureOperation)
    bar.name='Saw cut stock to length';body=comp.bRepBodies.item(0);body.name=rail['name']
    sketch.isLightBulbOn=False
    datum,up=plane(comp,2,lo[2]-.1);holes=comp.sketches.add(datum);holes.name='Three supplier fastener axes'
    for x,y in rail['boltCentersMm']:
        holes.sketchCurves.sketchCircles.addByCenterRadius(holes.modelToSketchSpace(point([x,y,lo[2]-.1])),rail['throughDiameterMm']/20)
    loops=core.ObjectCollection.create()
    for p in holes.profiles:loops.add(p)
    if loops.count!=3:raise ValueError('Incomplete three-hole stock drill pattern')
    cut=ext.createInput(loops,fusion.FeatureOperations.CutFeatureOperation)
    cut.participantBodies=[body]
    cut.setDistanceExtent(False,core.ValueInput.createByReal((hi[2]-lo[2]+.2)/10/up))
    ext.add(cut).name='Drill through both walls at the three 10-32 axes'
    holes.isLightBulbOn=False;occ.isGrounded=True
    original=None
    for lib in core.Application.get().materialLibraries:
        for item in lib.materials:
            key=item.name.lower().replace('-','')
            if '6061' in key and 't6' in key:
                original=item;break
        if original:break
    if not original:raise ValueError('6061-T6 stock physical material unavailable')
    material=des.materials.itemByName('002 6061-T6 tube') or des.materials.addByCopy(original,'002 6061-T6 tube')
    comp.bRepBodies.item(0).material=material


def run(context):
    app=core.Application.get();prior=app.activeDocument;before=[d.creationId for d in app.documents]
    path=Path(CONFIG['jobInputs']);out=Path(CONFIG['resultDirectory'])
    plan=json.loads((path/'chassis/front-corner.json').read_bytes());side='Front left SDS MK4i L2'
    receipt=out/'native-result.json';doc=None
    r={'status':'running','stage':'acquire','before':before,'placements':[],'iteration':plan['id']}
    def save(stage):
        r['stage']=stage;receipt.write_text(json.dumps(r,indent=2)+'\n')
    def append_archive(name,doc):
        target=out/name
        if target.exists():raise FileExistsError(target)
        des=design_of(doc)
        if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(target))):raise ValueError('Export failed')
        r.setdefault('archives',{})[name]=sha(target)
        return target
    def reopen(f):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(f)))
    try:
        if before or prior:raise ValueError('Project requested an empty station baseline')
        if receipt.exists():raise FileExistsError(receipt)
        save('validate frozen job, supplier and coordinates');NATIVE_JOB['verifyDependencies']()
        source=path/plan['source']
        if sha(source)!=plan['sourceSha256'] or sha(path/plan['option'])!=plan['optionSha256']:raise ValueError('Changed supplier source')
        plugin=Path(CONFIG['plugin'])
        spec=importlib.util.spec_from_file_location('red2_checked_placement',plugin/'fusion_checked_placement.py');guard=importlib.util.module_from_spec(spec)
        exec(compile((plugin/'fusion_checked_placement.py').read_bytes(),str(plugin/'fusion_checked_placement.py'),'exec'),guard.__dict__)
        save('create virgin native project and import unchanged 2023 manufacturer STEP')
        doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType);des=design_of(doc)
        des.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType;root=des.rootComponent;root.attributes.add('Red2Design','iteration',plan['id'])
        holder=root.occurrences.addNewComponent(core.Matrix3D.create());holder.component.name=side
        if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(source)),holder.component):raise ValueError('Supplier step import failed')
        des.activateRootComponent();adsk.doEvents()
        r['rawNativeCount']=sum(o.component.bRepBodies.count for o in root.allOccurrences)
        r['rawComponentNames']=[o.component.name for o in root.allOccurrences]
        save('resolve four excluded gear alternatives from physical native occurrences')
        if r['rawNativeCount']!=125:raise ValueError('Original native roster differs from 125 source solids')
        for name in plan['selected']['excluded']:
            matches=[o for o in root.allOccurrences if o.component.name==name]
            if len(matches)!=1:raise ValueError('Expected one supplier occurrence '+name)
            obj=matches[0];r.setdefault('removedOptions',[]).append([name,obj.fullPathName]);obj.deleteMe()
        if not des.computeAll():raise ValueError('Supplier configuration recompute failed')
        r['selectedNativeCount']=sum(o.component.bRepBodies.count for o in root.allOccurrences)
        if r['selectedNativeCount']!=121:raise ValueError('Unexpected configured roster')
        parent=append_archive('supplier-seed.f3d',doc);doc.close(False);doc=None
        doc=reopen(parent);r['supplierLocalBodies']=world_body_rows(design_of(doc));doc.close(False);doc=None
        if len(r['supplierLocalBodies'])!=121:raise ValueError('Cold supplier count changed')
        save('guard measured supplier from local source datum into robot axes')
        target=out/'001-positioned.f3d';pose=matrix(plan['worldMatrixMm'])
        def move(des):
            matches=[o for o in des.rootComponent.occurrences if o.component.name==side]
            if len(matches)!=1:raise ValueError('Lost the single configured root')
            occ=matches[0];occ.isGrounded=False;occ.transform2=pose;occ.isGrounded=True
        request=dict(occurrence=side+':1',worldTransformCm=list(pose.asArray()))
        entry=guard.checked_placement(app,parent,sha(parent),target,request,move);r['placements'].append(entry)
        if entry['status']!='pass':raise ValueError('Supplier datum placement held')
        parent=target;r['archives'][parent.name]=sha(parent)
        for i,rail in enumerate(plan['rails'],2):
            save('guard installation of editable '+rail['name'])
            target=out/('00'+str(i)+'-frame.f3d')
            request=dict(occurrence=rail['name']+':1',worldTransformCm=list(core.Matrix3D.create().asArray()))
            entry=guard.checked_placement(app,parent,sha(parent),target,request,lambda d:tube(d,rail))
            r['placements'].append(entry)
            if entry['status']!='pass':raise ValueError('Installed rail held: '+rail['name'])
            parent=target;r['archives'][parent.name]=sha(parent)
        save('cold-reopen frame and module for physical census and native-derived STEP')
        doc=reopen(parent);des=design_of(doc);r['coldBodies']=world_body_rows(des)
        if len(r['coldBodies'])!=123:raise ValueError('Final native census is incomplete')
        for rail in plan['rails']:
            row,=[v for v in r['coldBodies'] if v['name']==rail['name'] and v['path']==rail['name']+':1']
            expected=rail['minimumMm']+rail['maximumMm']
            if max(abs(x-y) for x,y in zip(expected,row['boundsMm']))>.01 or '6061-T6' not in row['material']:raise ValueError('Wrong as-built rail')
        step=out/'002-front-corner.step'
        if step.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError('Cold assembly export failed')
        r['stepSha256']=sha(step);r['lastNativeSha256']=sha(parent);r['status']='cold front-left configured module with two editable mounted lower rails'
        save('complete')
    except Exception:
        r.update(status='failed',error=traceback.format_exc())
    finally:
        for d in reversed(list(app.documents)):
            if d.creationId not in before:d.close(False)
        if prior and prior.isValid:prior.activate()
        r['after']=[d.creationId for d in app.documents]
        r['restored']=before==r['after'] and (not prior or app.activeDocument.creationId==prior.creationId)
        save(r['stage'])
    print(json.dumps({k:r[k] for k in ['status','stage','error','lastNativeSha256','stepSha256','restored'] if k in r}))
