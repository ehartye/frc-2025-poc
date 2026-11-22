"""Individually guard the two rotating washers, editable end shim and cap at the saved stow."""
import hashlib
import importlib.util
import json
import math
import traceback
from pathlib import Path

from adsk import core, fusion


def load(file,name):
    spec=importlib.util.spec_from_file_location(name,file)
    module=importlib.util.module_from_spec(spec)
    exec(compile(file.read_bytes(),str(file),'exec'),module.__dict__)
    return module


def signature(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def to_matrix(values):
    m=core.Matrix3D.create()
    assert m.setWithArray(values)
    return m


def from_mm(rows):
    return [c if j!=3 else c/10 for r in rows for j,c in enumerate(r)]+[0.,0.,0.,1.]


def frozen_step_body(app,design,path,source):
    """Copy a real vendor solid from its donor root frame before closing the donor."""
    other=None
    try:
        other=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(path)))
        supplier=fusion.Design.cast(other.products.itemByProductType('DesignProductType'))
        solids=[(s,None) for s in supplier.rootComponent.bRepBodies]
        solids.extend((s,o) for o in supplier.rootComponent.allOccurrences for s in o.component.bRepBodies)
        if len(solids)!=1 or not solids[0][0].isSolid:raise ValueError('One original supplier BRep required')
        raw,occ=solids[0]
        proxy=raw.createForAssemblyContext(occ) if occ else raw
        box=proxy.preciseBoundingBox
        actual=[10*x for point in (box.minPoint,box.maxPoint) for x in point.asArray()]
        if max(abs(x-y) for x,y in zip(actual,source['boundsMm']))>.006:raise ValueError('Supplier datums changed')
        manager=fusion.TemporaryBRepManager.get()
        standalone=manager.copy(raw)
        if occ and not manager.transform(standalone,occ.transform2):raise ValueError('Supplier root placement failed')
        return standalone
    finally:
        if other:other.close(False)
        design.parentDocument.activate()


def local_from_world(group, expected):
    inverse=group.transform2.copy()
    if not inverse.invert():raise ValueError('Noninvertible parent')
    a=inverse.asArray()
    return to_matrix([sum(a[4*i+k]*expected[4*k+j] for k in range(4))
                      for i in range(4) for j in range(4)])


def box_mm(body):
    box=body.preciseBoundingBox
    return [10*v for point in (box.minPoint,box.maxPoint) for v in point.asArray()]


def bound(actual, expected, limit, label):
    err=max(abs(a-b) for a,b in zip(actual,expected,strict=True))
    if err>limit:raise ValueError(f'{label}: max delta {err}; {actual} vs {expected}')


def witness(design, path, data):
    matches=[o for o in design.rootComponent.allOccurrences if o.fullPathName==path]
    if len(matches)!=1:raise ValueError(f'Expected one installed path {path}, got {len(matches)}')
    occ=matches[0]
    body=occ.component.bRepBodies.item(0)
    expected=from_mm(data['actualTargetWorldMatrixMm'])
    got=list(occ.transform2.asArray())
    bound(got,expected,1e-7,'occurrence world matrix')
    bounds=box_mm(body.createForAssemblyContext(occ))
    bound(bounds,data['worldBoundsMm'],.006,'root-world material bounds')
    probed=[]
    for v in body.vertices:
        local=[*v.geometry.asArray(),1]
        actual=[10*c for c in v.createForAssemblyContext(occ).geometry.asArray()]
        predicted=[10*sum(expected[4*i+k]*local[k] for k in range(4)) for i in range(3)]
        bound(actual,predicted,.005,'native material vertex relative to source')
        probed.append([local[:3],actual])
    if len(probed)<2:raise ValueError('Insufficient physical landmarks')
    return dict(path=path,rootWorldMatrixCm=got,rootWorldBoundsMm=bounds,
                sampledActualVertices=len(probed),firstAndLastVertices=[probed[0],probed[-1]])


def insert(design,name,data,app,root,base,trace):
    group=design.rootComponent.occurrences.itemByName('340 mm pan rotating installation:1')
    if not group:raise ValueError('Original rotating parent is missing')
    expectation=from_mm(data['actualTargetWorldMatrixMm'])
    native=data['nativeEditable']
    if native:
        src=root/'artifacts/red2-integrated/pan-thrust/guarded-support'/native['archive']
        if signature(src)!=native['sha256']:raise ValueError('Editable article changed')
        before={o.fullPathName for o in design.rootComponent.allOccurrences}
        new=app.importManager.importToTarget2(app.importManager.createFusionArchiveImportOptions(str(src)),group.component)
        new_items=[o for o in design.rootComponent.allOccurrences if o.fullPathName not in before]
        if not new or len(new_items)!=1:raise ValueError('Need one editable imported article')
        o=new_items[0]
        o.component.name=name
        c=o.component
        # The root-context proxy reports world coordinates, but its native
        # setter on the root-context proxy takes the requested WORLD frame.
        direct=o.nativeObject or o
        body=c.bRepBodies.item(0)
        local=local_from_world(group,expectation)
        trace.update(sourceBoundsMm=box_mm(body),initialOccurrenceWorldCm=list(o.transform2.asArray()),
                     parentWorldCm=list(group.transform2.asArray()),requestedLocalCm=list(local.asArray()),
                     sourceInstalledWorldCm=native['world'],importedName=o.fullPathName)
        bound(trace['sourceBoundsMm'],data['source']['boundsMm'],.006,'editable local BRep vs commissioned stock')
        bound(expectation,native['world'],1e-7,'archived source world datum vs independent drawing')
        # Import can create an already-grounded top occurrence; release only
        # this just-created component, register its live local transform, then
        # capture the position before tying the part to its installed shaft.
        if direct.isGroundToParent:direct.isGroundToParent=False
        o.transform2=to_matrix(expectation)
        if design.snapshots.hasPendingSnapshot:design.snapshots.add()
        path=group.fullPathName+'+'+name+':1'
        trace['positioned']=witness(design,path,data)
        # A rigid group on sibling occurrences preserves the attained relative
        # pose without ground-to-parent's possible reset to an insertion pose.
        shaft=next(o for o in group.component.occurrences if o.component.name==data['mate'])
        members=core.ObjectCollection.create()
        members.add(shaft);members.add(direct)
        rigid=group.component.rigidGroups.add(members,False)
        if not rigid:raise ValueError('Actual moving shaft attachment failed')
        rigid.name=name+' retained to '+data['mate']
        if not design.computeAll():raise ValueError('Failed to recompute actual shaft attachment')
        trace['coupled']=witness(design,path,data)
        features=dict(sketches=c.sketches.count,extrudes=c.features.extrudeFeatures.count,combines=c.features.combineFeatures.count)
        if features!=native['features'] or c.bRepBodies.count!=1 or c.bRepBodies.item(0).material.name!=native['material']:
            raise ValueError('Lost the original shop process or physical material')
    else:
        src=root/data['source']['file']
        if signature(src)!=data['source']['sha256']:raise ValueError('Original rotating washer changed')
        body=frozen_step_body(app,design,src,data['source'])
        # Local neutral placement is in the pan frame, whereas the guard expects root
        # coordinates. Invert the ACTUAL retained rotating occurrence at saved stow.
        local=local_from_world(group,expectation)
        o=group.component.occurrences.addNewComponent(local)
        c=o.component;c.name=name;o.isGroundToParent=True
        feature=c.features.baseFeatures.add();feature.name='Exact 6655K17 shaft washer'
        feature.startEdit()
        try:
            created=c.bRepBodies.add(body,feature)
            if not created:raise ValueError('Supplier BRep transfer failed')
            created.name=name
        finally:feature.finishEdit()
        s=c.bRepBodies.item(0);s.name=name
        mat=design.materials.itemByName('6655K17 supplier steel (grade unspecified)')
        if not mat:raise ValueError('Missing already verified actual bearing steel family')
        s.material=mat
        if s.material.name!=mat.name or not .0077<s.physicalProperties.mass/s.volume<.0081:
            raise ValueError('Purchased steel assignment failed')
        volume=s.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume*1000
        if abs(volume-data['source']['volumeMm3'])>.01:raise ValueError('Supplier volume changed')
        c.attributes.add('Source','shaft washer STEP SHA256',data['source']['sha256'])
    base.check_stow(base.pose(design))


def motion_witness(design,base,plan):
    moving=['340 mm pan rotating installation:1+'+n+':1' for n in plan['order']]
    fixed=['340 mm pan fixed installation:1+'+n+':1' for n in (
        'Piloted bilateral bearing support','6655K17 inboard stationary housing race',
        '6655K17 outboard stationary housing race')]
    def vertices(path):
        occ=next(o for o in design.rootComponent.allOccurrences if o.fullPathName==path)
        body=occ.component.bRepBodies.item(0)
        return [[10*z for z in v.createForAssemblyContext(occ).geometry.asArray()] for v in body.vertices]
    start={k:vertices(k) for k in moving+fixed}
    if any(len(pts)<2 for pts in start.values()):raise ValueError('Need identifiable material vertices on every attachment')
    readings=[]
    for angle in (-54.,-38.,-54.,-70.):
        r=math.radians(angle)
        base.set_joint(design,r)
        real=base.pose(design)
        if abs(real['jointRad']-r)>1e-7 or real['landmarkErrorMm']>.003:raise ValueError('Actual pan did not attain the signed coordinate')
        delta=math.radians(angle+70)
        def rotated(p):
            x,y,z=p
            return [205+(x-205)*math.cos(delta)+(z-580)*math.sin(delta),y,
                    580-(x-205)*math.sin(delta)+(z-580)*math.cos(delta)]
        rows={}
        for k,first in start.items():
            now=vertices(k)
            expected=[rotated(v) for v in first] if k in moving else first
            if len(now)!=len(first):raise ValueError('Native material topology changed '+k)
            err=max(math.dist(a,b) for a,b in zip(now,expected))
            travel=max(math.dist(a,b) for a,b in zip(now,first))
            if err>.006 or (k in moving and angle!=-70 and travel<.3):
                raise ValueError(f'Incorrect angular attachment {k}: {err}, {travel}')
            rows[k]=dict(count=len(now),maxVertexErrorMm=err,maxExcursionMm=travel,
                         materialVertexAtStowMm=first[0],observedVertexWorldMm=now[0])
        readings.append(dict(angleDeg=angle,floorWorldMm=real['landmarkWorldMm'],bodies=rows))
    return readings


def run(context):
    root=Path(CONFIG['jobInputs']);destination=Path(CONFIG['resultDirectory'])
    destination.mkdir(exist_ok=True)
    app=core.Application.get()
    active=app.activeDocument;before=[d.creationId for d in app.documents];doc=None
    report={'before':before,'placements':[]}
    def save(): (destination/(CONFIG['requestId']+'.json')).write_text(json.dumps(report,indent=2))
    try:
        scripts=Path(CONFIG['plugin'])
        base=load(root/'cad/red2-split-drop/pan_machined_parent.py','machined_parent')
        guard=load(scripts/'fusion_checked_placement.py','moving_commercial_placement')
        a=load(scripts/'fusion_authority.py','canonical_robot')
        for path,sha in CONFIG['inputs'].items():
            if signature(root/path)!=sha:raise ValueError('Frozen input changed '+path)
        report['authority']=a.verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        folder=root/'artifacts/red2-integrated/pan-thrust'
        plan=read(folder/'moving-stack-install-plan.json')
        first=read(folder/'moving-stack/pan-r3-moving-stack-01.json')
        passed,=first['placements']
        parent=folder/'moving-stack/checked-298.f3d'
        if passed['acceptedSha256']!=plan['parentF3DSha256'] or passed['successorSha256']!=signature(parent) or passed['status']!='pass' or passed['cold']['solids']!=298 or len(plan['order'])!=4:raise ValueError('Wrong accepted parent or commissioned stock')
        prefix='340 mm pan rotating installation:1+'
        for index,name in enumerate(plan['order'][1:],299):
            data=plan['placed'][name];mate=data['mate']
            if data['seated']['gapMm']>.001 or any(v['commonMm3']>.001 for v in data['close'].values()):
                raise ValueError('Offline placement did not qualify')
            report['stage']='guard '+name;save()
            path=prefix+name+':1';mate_path=prefix+mate+':1'
            req=dict(occurrence=path,worldTransformCm=from_mm(data['actualTargetWorldMatrixMm']),
                     interfaces=[dict(kind='bearing',maxGapMm=.001,bodies=[[path,name],[mate_path,mate]],
                                    basis='Measured finite bearing face at the nominal centered shaft stack; axial stops are checked separately.')])
            new=destination/(f'checked-{index}.f3d')
            NATIVE_JOB['verifyDependencies']()
            report.setdefault('registration',{})[name]=trace={}
            def install(design):
                try:return insert(design,name,data,app,root,base,trace)
                finally:save()
            entry=guard.checked_placement(app,parent,signature(parent),new,req,install)
            report['placements'].append(entry);save()
            if entry['status']!='pass':raise ValueError('Physical placement held '+name)
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(new)))
            design=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            pose=base.pose(design);base.check_stow(pose)
            inventory=base.census(design)
            if len(inventory)!=index or path+'/'+name not in inventory:raise ValueError('Incomplete reopened roster')
            entry['cold']={'solids':len(inventory),'stowedFloorMm':pose['landmarkWorldMm'],'stowRad':pose['jointRad']}
            entry['placedSourceWitness']=witness(design,path,data)
            if index==301:
                step=new.with_suffix('.step')
                if step.exists() or not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
                    raise ValueError('Cold derivative unavailable')
                report['archiveSha256']=signature(new);report['stepSha256']=signature(step);save()
                # Move the disposable, cold, locally reopened study. The archive
                # and derived STEP were both issued at the verified saved stow.
                report['stage']='check fixed and rotating material across signed joint reversals';save()
                report['signedAttachment']=motion_witness(design,base,plan)
                base.check_stow(base.pose(design))
            doc.close(False);doc=None;save();parent=new
        report['status']='movable shaft washers, cap and shim cold-verified at stow'
    except Exception:
        report.update(status='failed',error=traceback.format_exc())
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if active and active.isValid:active.activate()
        report['restored']=before==[d.creationId for d in app.documents];save()
    print(json.dumps({k:report[k] for k in ('status','stage','archiveSha256','stepSha256','error','restored') if k in report}))
