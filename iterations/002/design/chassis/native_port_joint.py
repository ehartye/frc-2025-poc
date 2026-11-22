"""Machine both shared longitudinal rails, then commission the serviceable port joint on a disposable copy."""
from pathlib import Path
import json
import math
import shutil
import traceback
from adsk import core, fusion

STATIONS=(-65.,-35.,35.,65.)
Y=328.9375
Z=94.63336287672979
NAME='Port terminal cartridge'
STEEL='ATI 17-4 H1150, 7.86 g per cubic cm'
AL='002 6061-T6 tube'
N=fusion.FeatureOperations.NewBodyFeatureOperation
CUT=fusion.FeatureOperations.CutFeatureOperation
JOIN=fusion.FeatureOperations.JoinFeatureOperation


def module(path):
    ns={'__name__':'_port_'+path.stem,'__file__':str(path)}
    exec(compile(path.read_bytes(),str(path),'exec'),ns)
    import types
    return types.SimpleNamespace(**ns)


def run(context):
    app=core.Application.get();out=Path(CONFIG['resultDirectory']);inp=Path(CONFIG['jobInputs'])
    project=inp/'iterations/002/design';scripts=Path(CONFIG['plugin'])
    stock=module(project/'chassis/native_corner.py');authority=module(scripts/'fusion_authority.py')
    guard=module(scripts/'fusion_checked_placement.py')
    report=dict(stages=[],before=[d.creationId for d in app.documents]);prior=app.activeDocument
    def save(stage):
        report['stage']=stage;(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    def reopen(p):return app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
    def issue(doc,p):
        des=stock.design_of(doc)
        if p.exists() or not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(p))):
            raise ValueError('Failed new private archive '+str(p))
    def draw(comp,label,axis,at,profiles,travel,operation,body=None):
        plane,sign=stock.plane(comp,axis,at)
        sk=comp.sketches.add(plane);sk.name=label;curves=sk.sketchCurves
        for geom in profiles:
            if geom[0]=='circle':
                _,center,r=geom
                curves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(stock.point(center)),r/10)
            else:
                points=[sk.modelToSketchSpace(stock.point(p)) for p in geom]
                for a,b in zip(points,points[1:]+points[:1]):curves.sketchLines.addByTwoPoints(a,b)
        loops=core.ObjectCollection.create()
        for profile in sk.profiles:loops.add(profile)
        if loops.count!=len(profiles):raise ValueError('Ambiguous sketch '+label)
        ext=comp.features.extrudeFeatures;req=ext.createInput(loops,operation)
        if body is not None and operation==CUT:req.participantBodies=[body]
        req.setDistanceExtent(False,core.ValueInput.createByReal(travel/sign/10))
        feat=ext.add(req);feat.name=label;sk.isLightBulbOn=False
        if comp.bRepBodies.count!=1:raise ValueError('Unexpected stock count after '+label)
        return comp.bRepBodies.item(0)
    def circles(axis,at,xy,r):
        return [('circle',[(at if j==axis else pair[j-(j>axis)]) for j in range(3)],r) for pair in xy]
    def points(axis,at,uv):return [[at if j==axis else pair[j-(j>axis)] for j in range(3)] for pair in uv]
    def rect(axis,at,u,v,w,h):return points(axis,at,[(u,v),(w,v),(w,h),(u,h)])
    def bolt_joint(comp,one,two,name):
        req=comp.asBuiltJoints.createInput(one,two,None);req.setAsRigidJointMotion();comp.asBuiltJoints.add(req).name=name
    def material(des,body,steel):
        if not steel:
            mat=des.materials.itemByName(AL)
            if not mat:raise ValueError('Lost registered original 6061-T6 material')
        else:
            mat=des.materials.itemByName(STEEL)
            if not mat:
                items=[m for lib in app.materialLibraries for m in lib.materials]
                source=next((m for m in items if '17-4' in m.name),None) or next((m for m in items if 'stainless steel' in m.name.lower()),None)
                if not source:raise ValueError('No editable stainless material asset for commissioned grade')
                mat=des.materials.addByCopy(source,STEEL);body.material=mat
                rho=body.physicalProperties.mass/body.volume
                for prop in mat.materialProperties:
                    if prop.id in ('structural_Density','thermal_Density'):
                        p=core.FloatProperty.cast(prop)
                        p.value=7860. if p.units=='KilogramPerCubicMeter' else p.value*.00786/rho
                if abs(body.physicalProperties.mass/body.volume-.00786)>.00002:raise ValueError('H1150 density not applied')
        body.material=mat
    def threads(comp,body,axis,radius,designation,count):
        faces=core.ObjectCollection.create()
        for face in body.faces:
            surf=core.Cylinder.cast(face.geometry)
            if surf and abs(surf.axis.asArray()[axis])>.99999 and abs(surf.radius*10-radius)<.0001:
                faces.add(face)
        if faces.count!=count:raise ValueError('Thread face count '+str((designation,faces.count,count)))
        tool=comp.features.threadFeatures;info=tool.createThreadInfo(True,'ANSI Unified Screw Threads',designation,'2B')
        request=tool.createInput(faces,info);request.isModeled=True;request.isFullLength=True
        feat=tool.add(request);feat.name='Tap '+designation+'-2B, modeled'
    def assembly(des):
        root=des.rootComponent
        group=root.occurrences.addNewComponent(core.Matrix3D.create());part=group.component;part.name=NAME
        original=root.occurrences.itemByName('Port longitudinal socket rail:1')
        def make(name):
            item=part.occurrences.addNewComponent(core.Matrix3D.create());item.component.name=name
            return item,item.component
        unit,comp=make('Removable port terminal 1 x 3 x .125')
        b=draw(comp,'Saw terminal to 254 mm',0,-127,[rect(0,-127,314.2,101.6,339.6,177.8)],254,N)
        b=draw(comp,'Hollow 1 x 3 stock',0,-128,[rect(0,-128,317.375,104.775,336.425,174.625)],256,CUT,b)
        xy=[(x,Y) for x in STATIONS]
        b=draw(comp,'Ream four shoulder seats',2,101.5,circles(2,101.5,xy,6.4),3.5,CUT,b)
        b=draw(comp,'Four wrench access windows',2,174.5,circles(2,174.5,xy,7),3.5,CUT,b)
        b.name='6061-T6 port terminal';material(des,b,False)
        recv,comp=make('Matched H1150 captive receiver')
        top=Z-3.175;l=321.45;r=339.25
        section=[(l,top-10),(r,top-10),(r,top-5),(r-5,top),(l+5,top),(l,top-5)]
        b=draw(comp,'Saw 154 mm with two 5 mm corner reliefs',0,-77,[points(0,-77,section)],154,N)
        b=draw(comp,'Four integral turned locating shoulders',2,top-5,circles(2,top-5,xy,6.325),17.5,JOIN,b)
        b=draw(comp,'Tap drills for quarter inch anchors',2,top-11,circles(2,top-11,xy,2.5527),25,CUT,b)
        b=draw(comp,'Tap drills for side retainers',1,321.3,circles(1,321.3,[(x,top-7.1) for x in (-50,50)],2.0574),16,CUT,b)
        b.name='A564 Type 630 H1150 receiver';material(des,b,True)
        # A bar without its tap is retained as diagnostic only if native threading fails.
        if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(out/'port-before-tapping.f3d'))):
            raise ValueError('Pre-tapping archive did not save')
        save('Cold-reference parent measured; tapping true receiver')
        threads(comp,b,2,2.5527,'1/4-20 UNC',4)
        threads(comp,comp.bRepBodies.item(0),1,2.0574,'10-32 UNF',2)
        bolt_joint(part,recv,unit,'Pilots couple four terminal holes to receiver')
        for x in STATIONS:
            for typ,lo,hi,y0,y1,w,rad,msteel in [
                ('Compression spacer',Z,101.6,Y-9.65,Y+9.65,14,6.4,False),
                ('Head load washer',104.775,107.95,318,335.8,11,3.35,True)]:
                item,c=make(typ+' '+str(x))
                b=draw(c,'Machine flat stock',2,lo,[rect(2,lo,x-w,y0,x+w,y1)],hi-lo,N)
                b=draw(c,'Finish clearance bore',2,lo-.1,circles(2,lo-.1,[(x,Y)],rad),hi-lo+.2,CUT,b)
                b.name=typ;material(des,b,msteel)
                bolt_joint(part,item,unit,'Registered '+typ+' '+str(x))
        if des.snapshots.hasPendingSnapshot:des.snapshots.add()
        bolt_joint(root,group,original,'Port lower rail to positive four-shoulder anchor')
        # Add original supplier definitions with a parent-local matrix derived from actual native occurrence.
        for label,plan in purchase_plan.items():
            donor=next((occ for occ in root.allOccurrences if occ.fullPathName==plan['nativeSource']),None)
            if not donor:raise ValueError('Missing original registered supplier occurrence '+plan['nativeSource'])
            for p in plan['placements']:
                pose=donor.transform2.copy();pose.transformBy(stock.matrix(p['worldDeltaMm']))
                group_inverse=group.transform2.copy();group_inverse.invert();pose.transformBy(group_inverse)
                item=part.occurrences.addExistingComponent(donor.component,pose)
                if des.snapshots.hasPendingSnapshot:des.snapshots.add()
                bolt_joint(part,item,recv,p['name'])
        # Raw constructed negative remains identifiable if guard rejects any true thread phase.
        checkpoint=out/'port-constructed.f3d'
        if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(checkpoint))):raise ValueError('Missing as-constructed diagnostic')
        save('Guarding constructed port cartridge')
    try:
        if report['before'] or prior:raise ValueError('Native station not at Home')
        NATIVE_JOB['verifyDependencies']()
        reg=authority.register_path(inp);line=reg.parent/'issue-003/lineage.json'
        report['authority']=authority.verify(inp,reg,line)
        source=authority.resolve(inp,authority.load(line)['archive'])
        chassis=project/'chassis'
        phased=chassis/'port-clocked-placements.json'
        purchase_input=phased if phased.exists() else chassis/'port-purchase-placements.json'
        purchase_plan=json.loads(purchase_input.read_bytes())
        report['purchasePlacementInput']=stock.sha(purchase_input)
        doc=reopen(source);des=stock.design_of(doc);root=des.rootComponent
        original=stock.world_body_rows(des)
        longitudinal=root.occurrences.itemByName('Port longitudinal socket rail:1');comp=longitudinal.component
        if len(original)!=488 or comp.bRepBodies.count!=1:raise ValueError('Wrong canonical stock')
        xy=[(x,Y) for x in STATIONS];b=comp.bRepBodies.item(0)
        b=draw(comp,'Match ream four pilot holes through upper face and corner blend',2,Z+.2,circles(2,Z+.2,xy,6.4),-12.2,CUT,b)
        b=draw(comp,'Drill lateral keeper access holes in datum wall',1,318.1,circles(1,318.1,[(x,Z-3.175-7.1) for x in (-50,50)],2.60),3.5,CUT,b)
        des.computeAll();parent=out/'lower-488-machined.f3d';issue(doc,parent);doc.close(False)
        doc=reopen(parent);des=stock.design_of(doc);now=stock.world_body_rows(des)
        if len(now)!=488:raise ValueError('Machining changed body population')
        old={(r['path'],r['name']):r for r in original};changed=[]
        for row in now:
            prev=old[row['path'],row['name']]
            if max(abs(a-b) for a,b in zip(row['boundsMm'],prev['boundsMm']))>.003 or row['material']!=prev['material']:raise ValueError('Unrelated geometry shifted')
            delta=prev['volumeMm3']-row['volumeMm3']
            if abs(delta)>.01:changed.append((row['path'],delta))
        expected=4*math.pi*6.4**2*3.175+2*math.pi*2.60**2*3.175
        if len(changed)!=2 or any(abs(d-expected)>.02 or not name.startswith('Port longitudinal socket rail:') for name,d in changed):
            raise ValueError('Shared rail machining altered uncommissioned material '+str(changed))
        report['machined']=dict(archiveSha256=stock.sha(parent),changes=changed,expectedRemovedMm3=expected)
        doc.close(False);save('Reopened machined parent; preparing service cartridge')
        final=out/'port-joint.f3d'
        record=guard.checked_placement(app,parent,stock.sha(parent),final,{'occurrence':NAME+':1','worldTransformCm':list(core.Matrix3D.create().asArray())},assembly)
        report['guard']=record
        if record['status']!='pass':raise ValueError('Full installed placement held: '+record['status'])
        doc=reopen(final);des=stock.design_of(doc);rows=stock.world_body_rows(des)
        if len(rows)!=504:raise ValueError('Expected 488 + sixteen physical articles')
        added=[r for r in rows if r['path'].startswith(NAME+':1+')]
        if len(added)!=16:raise ValueError('Unexpected cartridge inventory')
        report['coldCartridge']=added
        if phased.exists():
            witnesses=[]
            for plan in purchase_plan.values():
                source_occurrence=next(o for o in des.rootComponent.allOccurrences if o.fullPathName==plan['nativeSource'])
                for placement in plan['placements']:
                    seat=placement['headSeatXYZMm']
                    matches=[r for r in added if source_occurrence.component.name+':' in r['path']
                             and abs((r['boundsMm'][0]+r['boundsMm'][3])/2-seat[0])<.05]
                    if len(matches)!=1:raise ValueError('Ambiguous physical screw '+placement['name'])
                    occ=next(o for o in des.rootComponent.allOccurrences if o.fullPathName==matches[0]['path'])
                    expected=source_occurrence.transform2.copy()
                    expected.transformBy(stock.matrix(placement['worldDeltaMm']))
                    # Three non-collinear off-axis points detect both axial slip and angular drift.
                    errors=[]
                    for xyz in ((0,0,0),(12,0,0),(0,13,17)):
                        p=stock.point(xyz);q=p.copy();p.transformBy(expected);q.transformBy(occ.transform2)
                        errors.append(p.distanceTo(q)*10)
                    if max(errors)>.003:raise ValueError('Cold screw pose moved '+str((placement['name'],errors)))
                    witnesses.append(dict(name=placement['name'],worldPath=occ.fullPathName,
                                          phaseDeg=placement['commissionedPhaseDeg'],pointErrorMm=errors))
            if len(witnesses)!=6:raise ValueError('Incomplete screw pose roster')
            report['coldPurchaseTransforms']=witnesses
        idx=out/'port-joint.step'
        if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(idx))):raise ValueError('Failed cold diagnostic export')
        report.update(status='cold installed port checkpoint',nativeSha256=stock.sha(final),stepSha256=stock.sha(idx));doc.close(False)
        if phased.exists():
            # Replay authority with a private project mirror. The working project's selection changes only after review.
            mirror=out/'mirror'
            for relative in ('iterations/active.json','iterations/001/iteration.json','iterations/001/retrospective.json',
                             'iterations/seals/001.json','iterations/002/iteration.json'):
                destination=mirror/relative
                destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(inp/relative,destination)
            (mirror/'iterations/002/design').mkdir()
            register=authority.register_path(mirror)
            archived=register.parent/'private/port-504.f3d'
            archived.parent.mkdir(parents=True);shutil.copyfile(final,archived)
            spec=authority.load(reg)
            spec.update(configuration='Four original SDS L2 modules, two machined longitudinal rails, two transverse rails, one 16-body port terminal cartridge and six phased fully seated fasteners',
                        revision='002-port-anchor-504-r4',
                        basis='All six unloaded modeled threads and seated occurrences checked in the installed assembly. The shared original motor screw and steering key issues, preload, bumper retention and operating duty remain open.',
                        source=dict(kind='offline',archive=dict(file=archived.relative_to(mirror).as_posix(),sha256=stock.sha(archived))))
            register.write_text(json.dumps(spec,indent=2)+'\n')
            issue_folder=register.parent/'issue-004'
            report['lineage']=authority.export_issue(app,mirror,register,issue_folder)
            report['verifiedAuthority']=authority.verify(mirror,register,issue_folder/'lineage.json')
        save('complete')
    except Exception:report.update(status='held',error=traceback.format_exc())
    finally:
        for d in reversed(list(app.documents)):
            if d.creationId not in report['before']:d.close(False)
        if prior and prior.isValid:prior.activate()
        report.update(after=[d.creationId for d in app.documents],restored=not app.activeDocument and app.documents.count==0)
        save(report.get('stage','acquire'))
    print(json.dumps({k:report[k] for k in ('status','stage','nativeSha256','restored','error') if k in report}))

