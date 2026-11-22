"""Cold test nested editable shim registration with an independently moving parent and shaft."""
import json
import math
import traceback
from pathlib import Path
import adsk
from adsk import core,fusion


def run(context):
    from hashlib import sha256
    app=core.Application.get()
    root=Path(CONFIG['jobInputs'])
    out=Path(CONFIG['resultDirectory'])
    prior=app.activeDocument
    before=[d.creationId for d in app.documents]
    state={'before':before}
    doc=None
    def persist():
        (out/'nested-shaft-shim.json').write_text(json.dumps(state,indent=2))
    def sha(p):return sha256(p.read_bytes()).hexdigest()
    def m(values):
        res=core.Matrix3D.create()
        assert res.setWithArray(values)
        return res
    def mul(a,b):
        return [sum(a[i*4+k]*b[k*4+j] for k in range(4)) for i in range(4) for j in range(4)]
    def inverse(t):
        v=m(t)
        assert v.invert()
        return list(v.asArray())
    def turn(deg,origin,axis,offset=(0,0,0)):
        v=core.Matrix3D.create()
        assert v.setToRotation(math.radians(deg),core.Vector3D.create(*axis),core.Point3D.create(*origin))
        ans=list(v.asArray())
        for i in range(3):ans[i*4+3]+=offset[i]
        return ans
    def safe(got,want,eps,reason):
        err=max(abs(a-b) for a,b in zip(got,want,strict=True))
        if err>eps:raise ValueError(f'{reason} max {err} : {got} vs {want}')
    def openfile(p):
        d=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
        return d,fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
    def export(d,path,comp=None):
        opts=d.exportManager.createFusionArchiveExportOptions(str(path),comp) if comp else d.exportManager.createFusionArchiveExportOptions(str(path))
        if path.exists() or not d.exportManager.execute(opts):raise ValueError('Fresh F3D expected')
        return sha(path)
    def occmap(d):return {o.fullPathName:o for o in d.rootComponent.allOccurrences}
    def features(c):return [c.sketches.count,c.features.extrudeFeatures.count,c.features.combineFeatures.count]
    def body_info(c):
        s=c.bRepBodies.item(0)
        b=s.preciseBoundingBox
        return dict(features=features(c),material=s.material.name,boundsMm=[10*v for pt in (b.minPoint,b.maxPoint) for v in pt.asArray()],vertices=s.vertices.count)
    def readbody(d,path,w):
        o=occmap(d)[path]
        safe(list(o.transform2.asArray()),w,1e-7,'Root occurrence '+path)
        s=o.component.bRepBodies.item(0); samples=[]
        for v in s.vertices:
            p=[*v.geometry.asArray(),1]
            expected=[10*sum(p[k]*w[4*i+k] for k in range(4)) for i in range(3)]
            actual=[10*x for x in v.createForAssemblyContext(o).geometry.asArray()]
            safe(actual,expected,.005,'Native material landmarks '+path)
            samples.append([p[:3],actual])
        if len(samples)<2:raise ValueError('Insufficient landmarks')
        return dict(**body_info(o.component),worldCm=list(o.transform2.asArray()),firstLast=[samples[0],samples[-1]])
    try:
        for key,exp in CONFIG['inputs'].items():
            if sha(root/key)!=exp:raise ValueError('Changed input '+key)
        recipe=Path(CONFIG['plugin'])/'fusion_authority.py'
        mod={'__file__':str(recipe),'__name__':'_coupon_authority'}
        exec(compile(recipe.read_bytes(),str(recipe),'exec'),mod)
        state['authority']=mod['verify'](root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        d=root/'artifacts/red2-integrated/pan-thrust'
        plan=json.loads((d/'moving-stack-install-plan.json').read_bytes())
        shim='Match-ground 302 stainless thrust shim'
        shaft='Idler headed stub hex shank'
        group='Coupon rotating cartridge'
        source=d/'moving-stack/checked-298.f3d'
        state.update(stage='Extract original editable shaft',parentSha256=sha(source));persist()
        doc,des=openfile(source)
        so=occmap(des)['340 mm pan rotating installation:1+'+shaft+':1']
        original=body_info(so.component)
        shaft_file=out/'original-shaft.f3d'
        state['shaftSource']={'sha256':export(des,shaft_file,so.component),'original':original}
        doc.close(False);doc=None
        doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        p0=turn(28,(0,0,0),(0,0,1),(13.,-8.,7.))
        target=plan['placed'][shim]['nativeEditable']['world']
        local=mul(inverse(p0),target)
        if max(abs(x-y) for x,y in zip(local,core.Matrix3D.create().asArray()))<1:raise ValueError('Need genuinely different native frames')
        g=des.rootComponent.occurrences.addNewComponent(m(p0))
        g.component.name=group
        shaft_opts=app.importManager.createFusionArchiveImportOptions(str(shaft_file))
        prev=set(occmap(des))
        app.importManager.importToTarget2(shaft_opts,des.rootComponent)
        imported,=[o for k,o in occmap(des).items() if k not in prev]
        imported.component.name=shaft
        source_comp=imported.component
        os=g.component.occurrences.addExistingComponent(source_comp,m(local))
        if not os or not imported.deleteMe():raise ValueError('Could not create a single editable shaft under the new parent')
        os.isGroundToParent=True
        shaft_path,=[k for k,o in occmap(des).items() if o.component.name==shaft]
        state['installedShaft']=readbody(des,shaft_path,target)
        if body_info(os.component)!=original:raise ValueError('Shaft edit history, material or local stock changed')
        shim_file=d/'guarded-support'/plan['placed'][shim]['nativeEditable']['archive']
        before_import=set(occmap(des));state['stage']='Register nested shim';persist()
        app.importManager.importToTarget2(app.importManager.createFusionArchiveImportOptions(str(shim_file)),g.component)
        o,=[o for k,o in occmap(des).items() if k not in before_import]
        o.component.name=shim
        path=o.fullPathName
        native=o.nativeObject or o
        if native.isGroundToParent:native.isGroundToParent=False
        state['shimImported']={'worldCm':list(o.transform2.asArray()),'localStock':body_info(o.component),'path':path}
        state['registration']={'ownerWorldCm':p0,'expectedLocalCm':local,'expectedWorldCm':target}
        persist()
        o.transform2=m(target)
        if des.snapshots.hasPendingSnapshot:des.snapshots.add()
        state['shimPositioned']=readbody(des,path,target)
        c=core.ObjectCollection.create();c.add(os);c.add(native)
        group_rigid=g.component.rigidGroups.add(c,False)
        if not group_rigid:raise ValueError('Shaft/shim siblings must remain rigid')
        group_rigid.name='Actual shaft and shim'
        if not des.computeAll():raise ValueError('Failed to recompute')
        state['asBuilt']=readbody(des,path,target)
        saved=out/'nested-shaft-shim.f3d'
        state.update(stage='Cold independent verification',archiveSha256=export(des,saved));persist()
        doc.close(False);doc=None
        doc,des=openfile(saved)
        state['cold']={};gp=occmap(des)[group+':1']
        if gp.isGroundToParent:gp.isGroundToParent=False
        if gp.isGrounded:gp.isGrounded=False
        for angle in [0,17,-11,0]:
            delta=turn(angle,(20.5,0,58),(0,1,0))
            w=mul(delta,target)
            if angle or state['cold']:
                gp.transform2=m(mul(delta,p0))
                if des.snapshots.hasPendingSnapshot:des.snapshots.add()
                if not des.computeAll():raise ValueError('Failed after signed parent transform')
            state['cold'][str(len(state['cold']))]=dict(angleDeg=angle,shaft=readbody(des,shaft_path,w),shim=readbody(des,path,w))
            persist()
        state.update(status='pass',stage='Root proxy, material, sibling attachment and signed movement verified')
    except Exception:
        state.update(status='failed',error=traceback.format_exc())
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if prior and prior.isValid:prior.activate()
        state['restored']=before==[d.creationId for d in app.documents];persist()
    print(json.dumps({k:state[k] for k in ('status','stage','restored','error','archiveSha256') if k in state}))
