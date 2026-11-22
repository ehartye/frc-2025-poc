"""Machine the opposite three bolt axes in both existing rails; fit the second purchased corner."""
import importlib.util
import json
import math
import traceback
from pathlib import Path

from adsk import core, fusion


def load_script(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def drill(base, component, rail):
    low, high = rail['minimumMm'], rail['maximumMm']
    bottom = low[2] - .1
    plane, sign = base.plane(component, 2, bottom)
    sketch = component.sketches.add(plane)
    sketch.name = 'Opposite module screw pattern from cold supplier axes'
    centers = [p for p,s in zip(rail['boltCentersMm'], rail['sources']) if s['station'] != 'FL']
    for x,y in centers:
        sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch.modelToSketchSpace(base.point([x,y,bottom])),rail['throughDiameterMm']/20)
    profiles = core.ObjectCollection.create()
    for p in sketch.profiles: profiles.add(p)
    if profiles.count != 3 or component.bRepBodies.count!=1:raise ValueError('Unexpected drill scope')
    ext = component.features.extrudeFeatures
    cut = ext.createInput(profiles,fusion.FeatureOperations.CutFeatureOperation)
    cut.participantBodies=[component.bRepBodies.item(0)]
    cut.setDistanceExtent(False,core.ValueInput.createByReal((high[2]-low[2]+.2)/10/sign))
    ext.add(cut).name='Drill the three opposite module axes through both tube walls'
    sketch.isLightBulbOn=False


def indexed(rows):
    result={(r['path'],r['name']):r for r in rows}
    if len(result)!=len(rows): raise ValueError('Repeated occurrence or body identity')
    return result


def run(context):
    app=core.Application.get()
    before=[d.creationId for d in app.documents];prior=app.activeDocument
    inp=Path(CONFIG['jobInputs'])/'iterations/002/design'
    out=Path(CONFIG['resultDirectory'])
    base=load_script(inp/'chassis/native_corner.py','stock_source')
    guard=load_script(Path(CONFIG['plugin'])/'fusion_checked_placement.py','guard_slot')
    plan=json.loads((inp/'chassis/socket-holes.json').read_bytes())
    source=inp/'cad/private/frame-123.f3d'
    retained=inp/'cad/private/native-result.json'
    result={'before':before,'status':'running','parentSha256':base.sha(source)}
    receipt=out/'report.json'
    def stage(name):
        result['stage']=name;receipt.write_text(json.dumps(result,indent=2)+'\n')
    def reopen(p):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
    def check(des):
        current=indexed(base.world_body_rows(des))
        originals=indexed(json.loads(retained.read_bytes())['coldBodies'])
        old={r['name']:r for r in plan['rails'].values() if r['original']}
        measures={}
        if len(current)!=123 or set(originals)!=set(current):raise ValueError('Original occurrence roster changed')
        for key,r in current.items():
            original=originals[key]
            if r['material']!=original['material'] or max(abs(a-b) for a,b in zip(r['boundsMm'],original['boundsMm']))>.001:raise ValueError('Installed datum/material changed: '+str(key))
            diff=original['volumeMm3']-r['volumeMm3']
            if r['name'] in old and key[0]==r['name']+':1':
                rail=old[r['name']];expected=3*math.pi*(rail['throughDiameterMm']/2)**2*2*rail['wallMm']
                comp=des.rootComponent.occurrences.itemByName(key[0]).component
                if comp.features.extrudeFeatures.count!=3:raise ValueError('Editable stock history missing')
                axes={}
                for face in comp.bRepBodies.item(0).faces:
                    surf=core.Cylinder.cast(face.geometry)
                    if not surf or abs(surf.radius*10-rail['throughDiameterMm']/2)>.001:continue
                    if abs(surf.axis.z)<.99999:continue
                    center=tuple(round(k*10,3) for k in surf.origin.asArray()[:2])
                    axes[center]=axes.get(center,0)+1
                if axes!={tuple(p):2 for p in rail['boltCentersMm']}:raise ValueError('Missing one or more cylindrical holes across the two tube walls: '+repr(axes))
                measures[key[0]]={'removedMm3':diff,'expectedMm3':expected,'axes':[[*k,v] for k,v in axes.items()]}
            else:expected=0
            if abs(diff-expected)>.03:raise ValueError('Unexpected removed volume: '+repr((key,diff,expected)))
        return measures
    try:
        if before or prior or receipt.exists():raise ValueError('Fresh empty station and receipt required')
        NATIVE_JOB['verifyDependencies']()
        if base.sha(source)!=plan['nativeSha256']:raise ValueError('Incorrect master')
        stage('machine retained two stock rails from active master')
        doc=reopen(source);des=base.design_of(doc)
        for rail in plan['rails'].values():
            if rail['original']:drill(base,des.rootComponent.occurrences.itemByName(rail['name']+':1').component,rail)
        if not des.computeAll():raise ValueError('Machining recompute failed')
        result['warmRails']=check(des);stage('save and reopen machined predecessor')
        machined=out/'drilled.f3d'
        if machined.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(machined))):raise ValueError('Machined archive export failed')
        doc.close(False);doc=reopen(machined);des=base.design_of(doc);result['coldRails']=check(des)
        result['machinedSha256']=base.sha(machined);doc.close(False)
        stage('guard second 121-body supplier module against machined assembly')
        root_name='Front left SDS MK4i L2';source_path=root_name+':1';target_path=root_name+':2'
        delta=base.matrix([[0,1,0,5],[-1,0,0,5],[0,0,1,0],[0,0,0,1]])
        def insert(des):
            root=des.rootComponent
            source_occ=root.occurrences.itemByName(source_path)
            wanted=source_occ.transform2.copy();wanted.transformBy(delta)
            new=root.occurrences.addExistingComponent(source_occ.component,wanted)
            if new.fullPathName!=target_path:raise ValueError('Unexpected second occurrence path: '+new.fullPathName)
            new.attributes.add('Red2Design','station','front-right')
            if des.snapshots.hasPendingSnapshot:des.snapshots.add()
            joint=root.asBuiltJoints.createInput(new,root.occurrences.itemByName('Front transverse socket rail:1'),None)
            joint.setAsRigidJointMotion();root.asBuiltJoints.add(joint).name='Front-right plates to front transverse rail'
        candidate=out/'front-244.f3d'
        result['placement']=guard.checked_placement(app,machined,base.sha(machined),candidate,{'occurrence':target_path,'sourceOccurrence':source_path,'worldDeltaFromSourceCm':list(delta.asArray())},insert)
        if result['placement']['status']!='pass':raise ValueError('Second module held by guarded installed comparison')
        stage('cold physical comparison of both actual module instances')
        doc=reopen(candidate);des=base.design_of(doc);rows=base.world_body_rows(des)
        lookup=indexed(rows);errors={}
        for r in rows:
            if not r['path'].startswith(source_path+'+'):continue
            other=lookup[(target_path+r['path'][len(source_path):],r['name'])];b=r['boundsMm']
            expected=[5+b[1],5-b[3],b[2],5+b[4],5-b[0],b[5]]
            errors[r['path']+'/'+r['name']]=max(abs(x-y) for x,y in zip(expected,other['boundsMm']))
            if abs(other['volumeMm3']-r['volumeMm3'])>.01 or other['material']!=r['material']:raise ValueError('Source article changed')
        result.update(bodyCount=len(rows),sourcePlacements=errors,asBuiltJoints=[j.name for j in des.rootComponent.asBuiltJoints])
        if len(rows)!=244 or len(errors)!=121 or max(errors.values())>.002 or len(result['asBuiltJoints'])!=3:raise ValueError('Incomplete independent installed readback')
        step=out/'front-244.step'
        if step.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError('Cold STEP export failed')
        result.update(nativeSha256=base.sha(candidate),stepSha256=base.sha(step),status='two purchased front modules on machined forward rail; one installed longitudinal')
        stage('complete')
    except Exception:result.update(status='failed',error=traceback.format_exc())
    finally:
        for doc in reversed(list(app.documents)):
            if doc.creationId not in before:doc.close(False)
        if prior and prior.isValid:prior.activate()
        result['after']=[d.creationId for d in app.documents]
        result['restored']=result['after']==before and (not prior or app.activeDocument.creationId==prior.creationId)
        stage(result.get('stage','initial'))
    print(json.dumps({k:result[k] for k in ('stage','status','error','restored','nativeSha256','stepSha256') if k in result}))
