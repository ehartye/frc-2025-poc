"""Resume the cold-reopened 121-body source. Verify the actual root and material across recompute."""
import importlib.util
import itertools
import json
import math
import traceback
from pathlib import Path
import adsk
from adsk import core,fusion


def module(path,key):
    spec=importlib.util.spec_from_file_location(key,path);mod=importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(),str(path),'exec'),mod.__dict__)
    return mod


def run(context):
    app=core.Application.get();before=[d.creationId for d in app.documents];prior=app.activeDocument
    inputs=Path(CONFIG['jobInputs']);out=Path(CONFIG['resultDirectory']);base=module(inputs/'chassis/native_corner.py','corner_stock')
    plan=json.loads((inputs/'chassis/front-corner.json').read_bytes());m=plan['worldMatrixMm'];wanted=base.matrix(m)
    original=json.loads((inputs/'private-inputs/first-native-result.json').read_bytes())
    seed=inputs/'private-inputs/supplier-seed.f3d'
    guard=module(Path(CONFIG['plugin'])/'fusion_checked_placement.py','guard_corner')
    state={'status':'running','stage':'start','before':before,'placements':[],'sourceNativeSha256':base.sha(seed),'datum':{},'archives':{}}
    dest=out/'native-result.json';doc=None;target_name='Front left SDS MK4i L2:1'
    def save():
        dest.write_text(json.dumps(state,indent=2)+'\n')
    def stage(text):
        state['stage']=text;save()
    def reopen(file):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(file)))
    def world_report(des):
        rows=base.world_body_rows(des)
        actual={(v['path'],v['name']):v for v in rows}
        errors={}
        for row in original['supplierLocalBodies']:
            v=actual[(row['path'],row['name'])]['boundsMm'];a=row['boundsMm']
            # The front-left source transform permutes positive coordinate axes.
            expected=[a[i]+m[j][3] for j,i in enumerate([2,0,1])]+[a[3+i]+m[j][3] for j,i in enumerate([2,0,1])]
            errors[row['path']+'/'+row['name']]=max(abs(x-y) for x,y in zip(expected,v))
        if len(errors)!=121:raise ValueError('Incomplete original material witness roster')
        return dict(solids=len(rows),matches=sum(x<.025 for x in errors.values()),maximumBoundsErrorMm=max(errors.values()),
                    failed={k:v for k,v in errors.items() if v>=.025})
    try:
        if before or prior or dest.exists():raise ValueError('Unexpected initial station or output')
        stage('check source checkpoint and immutable stage')
        NATIVE_JOB['verifyDependencies']()
        if base.sha(seed)!=original['archives']['supplier-seed.f3d'] or len(original['supplierLocalBodies'])!=121:raise ValueError('Wrong source native configuration')
        def move(des):
            occ=des.rootComponent.occurrences.itemByName(target_name)
            def observe(key):
                item=state['datum'][key]=dict(name=occ.fullPathName,world=list(occ.transform2.asArray()),
                    initial=list(occ.initialTransform.asArray()),grounded=occ.isGrounded,groundToParent=occ.isGroundToParent,
                    pendingSnapshot=des.snapshots.hasPendingSnapshot)
                # Observe real material at three non-collinear fastening axes.
                origins=[]
                for o in des.rootComponent.allOccurrences:
                    if '91251A355' not in o.component.name:continue
                    b=o.component.bRepBodies.item(0);pt=b.vertices.item(0).createForAssemblyContext(o).geometry
                    origins.append([o.fullPathName,*[v*10 for v in pt.asArray()]])
                if len(origins)!=6:raise ValueError('Missing original frame screw landmarks')
                item['sixActualScrewVerticesMm']=origins;save()
            observe('before')
            if occ.isGrounded:occ.isGrounded=False
            if occ.isGroundToParent:occ.isGroundToParent=False
            observe('freed')
            occ.transform2=wanted;observe('set')
            if des.snapshots.hasPendingSnapshot:des.snapshots.add()
            observe('captured')
            if not des.computeAll():raise ValueError('Failed to recompute moved source')
            observe('afterRecompute')
            read=world_report(des);state['warmDatum']=read;save()
            if max(abs(x-y) for x,y in zip(occ.transform2.asArray(),wanted.asArray()))>1e-7 or read['matches']!=121:
                raise ValueError('Source failed actual material world registration before placement gate')
        stage('guard model coordinate transfer and record six actual material points before and after compute')
        first=out/'registered-121.f3d'
        result=guard.checked_placement(app,seed,base.sha(seed),first,dict(occurrence=target_name,worldTransformCm=list(wanted.asArray())),move)
        state['placements'].append(result);save()
        if result['status']!='pass':raise ValueError('Guard rejected the placed module')
        state['archives'][first.name]=base.sha(first);parent=first
        doc=reopen(parent);state['coldDatum']=world_report(base.design_of(doc));doc.close(False);doc=None
        if state['coldDatum']['matches']!=121:raise ValueError('Module changed after reopening')
        for i,rail in enumerate(plan['rails'],1):
            stage('guard '+rail['name']+' and add its explicit as-built plate attachment')
            target=out/('frame-'+str(121+i)+'.f3d')
            def add_rail(des):
                base.tube(des,rail);root=des.rootComponent
                bar=root.occurrences.itemByName(rail['name']+':1')
                factory=root.occurrences.itemByName(target_name)
                if i==2:
                    if bar.isGrounded:bar.isGrounded=False
                    if bar.isGroundToParent:bar.isGroundToParent=False
                joint=root.asBuiltJoints.createInput(bar,factory,None)
                joint.setAsRigidJointMotion()
                root.asBuiltJoints.add(joint).name=rail['name']+' to factory plates'
            result=guard.checked_placement(app,parent,base.sha(parent),target,dict(occurrence=rail['name']+':1',worldTransformCm=list(core.Matrix3D.create().asArray())),add_rail)
            state['placements'].append(result);save()
            if result['status']!='pass':raise ValueError('Guard rejected installation of '+rail['name'])
            state['archives'][target.name]=base.sha(target);parent=target
        stage('cold 123 solid source, both joints, section profile and material readback')
        doc=reopen(parent);des=base.design_of(doc);rows=base.world_body_rows(des);state['coldBodies']=rows
        state['coldDatumWithRails']=world_report(des)
        if len(rows)!=123 or state['coldDatumWithRails']['matches']!=121:raise ValueError('Installed module datum changed')
        state['joints']=[j.name for j in des.rootComponent.asBuiltJoints]
        for rail in plan['rails']:
            o=des.rootComponent.occurrences.itemByName(rail['name']+':1')
            row,=[v for v in rows if v['path']==o.fullPathName and v['name']==rail['name']]
            ext=rail['maximumMm'];bottom=rail['minimumMm'];wall=rail['wallMm'];dia=rail['throughDiameterMm']
            area=25.4*50.8-(25.4-2*wall)*(50.8-2*wall)
            expected=(ext[rail['lengthAxis']]-bottom[rail['lengthAxis']])*area-3*math.pi*(dia/2)**2*2*wall
            if max(abs(x-y) for x,y in zip(row['boundsMm'],bottom+ext))>.01 or abs(row['volumeMm3']-expected)>.1 or '6061-T6' not in row['material'] or o.component.features.extrudeFeatures.count!=2:raise ValueError('Wrong cold machined stock '+rail['name'])
            if rail['name']+' to factory plates' not in state['joints']:raise ValueError('Incomplete rail attachment')
        step=out/'registered-123.step'
        if step.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError('Fresh assembly derivative failed')
        state.update(status='123 cold solids, two as-built plate attachments and editable machined rails',stepSha256=base.sha(step),nativeSha256=base.sha(parent))
        stage('complete')
    except Exception:state.update(status='failed',error=traceback.format_exc())
    finally:
        for doc in reversed(list(app.documents)):
            if doc.creationId not in before:doc.close(False)
        if prior and prior.isValid:prior.activate()
        state['after']=[d.creationId for d in app.documents];state['restored']=state['after']==before and (not prior or app.activeDocument.creationId==prior.creationId);save()
    print(json.dumps({k:state[k] for k in ('status','stage','nativeSha256','stepSha256','restored','error') if k in state}))
