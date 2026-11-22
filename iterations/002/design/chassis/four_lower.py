"""Commission the four lower module stations from the checked two-front Fusion assembly."""
import importlib.util
import json
import math
import shutil
import traceback
from pathlib import Path

from adsk import core


def module_at(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    exec(compile(path.read_bytes(),str(path),'exec'),module.__dict__)
    return module


def member_axes(root,body_plan):
    screws={}
    tubes={}
    def axes(body):
        for face in body.faces:
            surf=core.Cylinder.cast(face.geometry)
            if surf and abs(surf.axis.z)>.999999:
                yield tuple(10*k for k in surf.origin.asArray()[:2]),surf.radius*10
    tubes_at={'Port longitudinal socket rail:1':'Port longitudinal socket rail',
              'Port longitudinal socket rail:2':'Starboard longitudinal socket rail',
              'Front transverse socket rail:1':'Front transverse socket rail',
              'Front transverse socket rail:2':'Rear transverse socket rail'}
    for occ in root.allOccurrences:
        name=occ.fullPathName
        for body in occ.component.bRepBodies:
            if name in tubes_at:
                nr=tubes_at[name]; source=body_plan[nr]; proxy=body.createForAssemblyContext(occ)
                centers=list(axes(proxy));measured={}
                for xy in source['boltCentersMm']:
                    fitted=[c for c,r in centers if abs(r-2.65)<.001 and math.dist(c,xy)<.005]
                    if len(fitted)!=2:raise ValueError('Tube must contain one bore through each wall: '+str((name,xy,len(fitted))))
                    measured[str(xy)]=fitted
                tubes[name]=measured
            elif '91251A355' in name or '91251A355' in body.name:
                proxy=body.createForAssemblyContext(occ);candidates=list(axes(proxy))
                for tube in body_plan.values():
                    for xy,q in zip(tube['boltCentersMm'],tube['sources']):
                        key=f"{q['station']}/{xy[0]},{xy[1]}"
                        # Each shaft needs one cylindrical face positioned at the correct actual installed screw axis.
                        physical=[c for c,r in candidates if 1.5<r<5 and math.dist(c,xy)<.015]
                        if physical:
                            if key in screws:raise ValueError('Two installed fastener occurrences at one axis: '+key)
                            screws[key]=dict(occurrence=name,body=body.name,centerMm=physical[0],errorMm=math.dist(physical[0],xy))
    return tubes,screws


def run(context):
    app=core.Application.get();before=[d.creationId for d in app.documents];prior=app.activeDocument
    inp=Path(CONFIG['jobInputs']);out=Path(CONFIG['resultDirectory'])
    d=inp/'iterations/002/design';bin=Path(CONFIG['plugin'])
    base=module_at(d/'chassis/native_corner.py','stock')
    guard=module_at(bin/'fusion_checked_placement.py','guard')
    authority=module_at(bin/'fusion_authority.py','authority')
    report=dict(status='running',before=before,stages=[])
    def save(stage):
        report['stage']=stage;(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    def reopen(file):
        return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(file)))
    try:
        if before or prior:raise ValueError('Reserved native station is not empty')
        NATIVE_JOB['verifyDependencies']()
        reg=authority.register_path(inp);lineage=reg.parent/'issue-002/lineage.json'
        report['start']=authority.verify(inp,reg,lineage);parent=authority.resolve(inp,authority.load(lineage)['archive'])
        half=base.matrix([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
        longitudinal='Port longitudinal socket rail';transverse='Front transverse socket rail';module='Front left SDS MK4i L2'
        jobs=[
            (longitudinal+':1',longitudinal+':2',module+':2','Starboard rail to front-right plates','starboard','s245.f3d'),
            (module+':1',module+':3',longitudinal+':2','Rear-right plates to starboard rail','rear-right','s366.f3d'),
            (transverse+':1',transverse+':2',module+':3','Rear rail to rear-right plates','rear','s367.f3d'),
            (module+':2',module+':4',transverse+':2','Rear-left plates to rear rail','rear-left','lower-488.f3d')]
        for src,dst,host,joint_name,station,filename in jobs:
            def place(des):
                root=des.rootComponent;donor=root.occurrences.itemByName(src);parent_occ=root.occurrences.itemByName(host)
                pose=donor.transform2.copy();pose.transformBy(half)
                occ=root.occurrences.addExistingComponent(donor.component,pose)
                if occ.fullPathName!=dst:raise ValueError('Uncommissioned occurrence identity '+occ.fullPathName)
                occ.attributes.add('Red2Design','station',station)
                if des.snapshots.hasPendingSnapshot:des.snapshots.add()
                joint=root.asBuiltJoints.createInput(occ,parent_occ,None)
                joint.setAsRigidJointMotion();root.asBuiltJoints.add(joint).name=joint_name
            save('Guarding '+station)
            nextfile=out/filename
            record=guard.checked_placement(app,parent,base.sha(parent),nextfile,{'sourceOccurrence':src,'occurrence':dst,'worldDeltaFromSourceCm':list(half.asArray())},place)
            report['stages'].append(dict(station=station,fromSha256=base.sha(parent),outcome=record))
            if record['status']!='pass':raise ValueError('Guard held '+station)
            parent=nextfile;save('Saved '+station)
        save('Cold reopen and measure every real lower occurrence')
        doc=reopen(parent);des=base.design_of(doc);rows=base.world_body_rows(des)
        keyed={(row['path'],row['name']):row for row in rows}
        if len(keyed)!=len(rows) or len(rows)!=488:raise ValueError('Incomplete or ambiguous body census')
        checks={}
        for src,dst,*rest in jobs:
            matched=[]
            for (path,name),row in keyed.items():
                if path!=src and not path.startswith(src+'+'):continue
                other=keyed[(dst+path[len(src):],name)];b=row['boundsMm']
                expected=[-b[3],-b[4],b[2],-b[0],-b[1],b[5]]
                error=max(abs(x-y) for x,y in zip(expected,other['boundsMm']))
                if error>.003 or abs(row['volumeMm3']-other['volumeMm3'])>.01 or row['material']!=other['material']:
                    raise ValueError('Displaced, rematerialized, or changed installed article: '+str((dst,path,name,error)))
                matched.append(error)
            checks[dst]=dict(count=len(matched),maxWorldBoundsErrorMm=max(matched))
        joints=[j.name for j in des.rootComponent.asBuiltJoints]
        if sorted(r['count'] for r in checks.values())!=[1,1,121,121] or len(joints)!=7:raise ValueError('Source or joint coverage')
        report.update(coldBodyCount=len(rows),coldPlacements=checks,asBuiltJoints=joints,nativeSha256=base.sha(parent))
        save('Saved full body census, measuring screw axes')
        plan=json.loads((d/'chassis/socket-holes.json').read_bytes())['rails']
        tubes,screws=member_axes(des.rootComponent,plan);report.update(tubeBores=tubes,installedMountScrews=screws)
        if len(tubes)!=4 or len(screws)!=24:raise ValueError('Installed hole/shaft pairs incomplete: '+str((len(tubes),len(screws))))
        doc.close(False)
        save('Cold export from independently measured 488-body draft register')
        mirror=out/'mirror';listing=json.loads((d/'chassis/export-244-inputs.json').read_bytes())
        for row in listing:
            if '/cad/' in row['file']:continue
            to=mirror/row['file'];to.parent.mkdir(exist_ok=True,parents=True)
            shutil.copyfile(inp/row['file'],to)
        (mirror/'iterations/002/design/cad/private').mkdir(parents=True,exist_ok=True)
        new_reg=authority.register_path(mirror);spec=authority.load(reg)
        dest='iterations/002/design/cad/private/lower-488.f3d'
        (mirror/dest).parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(parent,mirror/dest)
        spec.update(configuration='Four named 121-body NEO SDS MK4i L2 modules and four editable six-hole 6061-T6 lower socket rails at parked azimuth',revision='002-lower-488-r3',source={'kind':'offline','archive':{'file':dest,'sha256':base.sha(parent)}},basis='Retain 24 source motor screw bottoming controls. All 488 bodies cold-counted; observed 24 vertical mounting screw axes through 48 actual rail walls and all four projected-to-native corner transformations. Bumper, mass closure, upper, wiring and operating loads remain separate.')
        new_reg.write_text(json.dumps(spec,indent=2)+'\n')
        new_issue=new_reg.parent/'issue-003';report['draftLineage']=authority.export_issue(app,mirror,new_reg,new_issue)
        report['verifiedDraft']=authority.verify(mirror,new_reg,new_issue/'lineage.json')
        report['status']='four real corners and four rails; complete measured parking datum'
        save('complete')
    except Exception:report.update(status='failed',error=traceback.format_exc())
    finally:
        for doc in reversed(list(app.documents)):
            if doc.creationId not in before:doc.close(False)
        if prior and prior.isValid:prior.activate()
        report['after']=[d.creationId for d in app.documents]
        report['restored']=report['after']==before and (not prior or app.activeDocument.creationId==prior.creationId)
        save(report.get('stage','acquire'))
    print(json.dumps({k:report[k] for k in ('status','stage','nativeSha256','restored','error') if k in report}))
