"""Create a new editable shop-geometry study, then cold-reopen and exercise it."""
import hashlib
import importlib.util
import json
import math
import traceback
from pathlib import Path
from adsk import core,fusion
import adsk

def mm(x):return core.ValueInput.createByString(str(x)+' mm')
def point(x,y,z):return core.Point3D.create(x/10,y/10,z/10)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def make(des,rev,suppliers):
    root=des.rootComponent
    retained=rev in ('r2','r3','r4','r5')
    coupled=rev in ('r3','r4','r5')
    root.attributes.add('Project','scope','Red 2 split-drop V0 shop geometry fixture')
    des.userParameters.add('PlateGauge',mm(6),'mm','Original test-plate stock; final drawing and material selection remain open.')
    des.userParameters.add('SideWallGauge',mm(12),'mm','Shop rail gauge for the concept fixture.')
    member={}
    materials={}
    library=core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    if library:
        candidates={}
        for item in library.materials:
            n=item.name.lower()
            if '6061' in n and ('t6' in n or 't-6' in n):candidates[n]=item
            if 'uhmw' in n or 'ultra high molecular' in n:materials['polyethylene']=item
        if candidates:materials['aluminum']=candidates[min(candidates,key=len)]
    def component(k):
        occurrence=root.occurrences.addNewComponent(core.Matrix3D.create())
        occurrence.component.name=k
        member[k]=occurrence
        return occurrence.component
    def profile(comp,name,poly,normal,offset,thickness):
        plane={'x':comp.yZConstructionPlane,'y':comp.xZConstructionPlane,'z':comp.xYConstructionPlane}[normal]
        vec=plane.geometry.normal
        sign=getattr(vec,normal)
        if abs(abs(sign)-1)>.00001:raise ValueError('Unexpected construction normal')
        anchor=comp.constructionPlanes.createInput()
        anchor.setByOffset(plane,mm(offset/sign))
        shifted=comp.constructionPlanes.add(anchor)
        shifted.name=name+' stock face'
        sk=comp.sketches.add(shifted);sk.name=name+' editable cut profile'
        proj=[]
        for u,v in poly:
            w={'x':lambda:point(offset,u,v),'y':lambda:point(u,offset,v),'z':lambda:point(u,v,offset)}[normal]()
            proj.append(sk.modelToSketchSpace(w))
        first=previous=None
        for i in range(len(proj)):
            a=previous if previous else proj[0]
            b=first if i+1==len(proj) else proj[i+1]
            ln=sk.sketchCurves.sketchLines.addByTwoPoints(a,b)
            if first is None:first=ln.startSketchPoint
            previous=ln.endSketchPoint
        if sk.profiles.count!=1:raise ValueError(name+' failed to close')
        value=core.ValueInput.createByString(('' if sign>0 else '-')+thickness)
        ext=comp.features.extrudeFeatures.addSimple(sk.profiles.item(0),value,fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.name=name+' thickness';ext.bodies.item(0).name=name
        kind='polyethylene' if 'UHMW' in name else 'aluminum'
        if kind in materials:ext.bodies.item(0).material=materials[kind]
        sk.isLightBulbOn=False;shifted.isLightBulbOn=False
        return ext.bodies.item(0)
    def bar(comp,name,x0,y0,x1,y1,z,h='PlateGauge'):
        profile(comp,name,[(x0,y0),(x1,y0),(x1,y1),(x0,y1)],'z',z,h)
    def drill(comp,centers):
        start=comp.constructionPlanes.createInput();start.setByOffset(comp.xYConstructionPlane,mm(479))
        sk=comp.sketches.add(comp.constructionPlanes.add(start));sk.name='Keeper through bolt clearance holes'
        for x,y in centers:sk.sketchCurves.sketchCircles.addByCenterRadius(point(x,y,0),.26)
        profiles=core.ObjectCollection.create()
        for prof in sk.profiles:profiles.add(prof)
        holes=comp.features.extrudeFeatures.addSimple(profiles,mm(14),fusion.FeatureOperations.CutFeatureOperation)
        holes.name='Drill keeper and anchor plates 5.2 mm through';sk.isLightBulbOn=False
    def vendor(comp,sku,label,values,digest,suffix='.STEP'):
        file=suppliers/(sku+suffix)
        if sha(file)!=digest:raise ValueError('Supplier file changed: '+sku)
        mat=core.Matrix3D.create();mat.setWithArray(values)
        occurrence=comp.occurrences.addNewComponent(mat)
        occurrence.component.name=label+' '+sku
        manager=core.Application.get().importManager
        if not manager.importToTarget(manager.createSTEPImportOptions(str(file)),occurrence.component):raise ValueError('Supplier import failed')
        occurrence.component.attributes.add('Supplier source','sha256',digest)
        if rev in ('r4','r5'):
            def mount(part):
                part.isGroundToParent=True
                for child in part.component.occurrences:mount(child)
            mount(occurrence)
        return occurrence
    fixed=component('Fixture bench rails')
    carriage=component('Longitudinal carriage')
    member['Fixture bench rails'].isGroundToParent=True
    guide=240 if retained else 202
    fore=106 if retained else 0
    for side,sign in [('Left',1),('Right',-1)]:
        y=sign*guide
        bar(fixed,side+' custom bench rail',-288,y-7,330,y+7,492)
        bar(fixed,side+' machined UHMW runner',-288,y-7,330,y+7,498)
        # Keep 1 mm beyond the front track's x=311 face. The R4 extra 8 mm
        # physically crossed the sloping scoring panel at the low signed dock.
        end=312 if rev=='r5' else 320
        bar(carriage,side+' moving guide plate',-260,y-7,end,y+7,504)
        if retained:
            foot=[(234,486),(273,486),(273,492),(257,492),(257,517.6),(244,517.6),(244,511.6),(247.6,511.6),(247.6,492),(234,492)]
            # Begin after the rear bridge. Side plate face sits 0.6 mm clear;
            # an underside wear strip establishes the top running gap.
            profile(fixed,side+' longitudinal keeper',[(sign*y,z) for y,z in foot],'x',-268,'598 mm')
            lo,hi=sorted((sign*244,sign*247.5))
            bar(fixed,side+' upper UHMW wear face',-268,lo,330,hi,510.6,'1 mm')
            lo,hi=sorted((sign*233,sign*273))
            bar(fixed,side+' machined anchor shoe',-268,lo,330,hi,480)
    if retained:
        centers=[(x,y) for y in (-265,265) for x in (-245,-55,125,298)]
        drill(fixed,centers)
        for i,(x,y) in enumerate(centers):
            vendor(fixed,'REV-29-2947','Keeper screw '+str(i+1),[1,0,0,x/10,0,1,0,y/10,0,0,1,48.37577,0,0,0,1],
                   'a54f396b0c1695ec7cc77275b526e7f365f9e83466be3b2eaca014d3312de052')
            vendor(fixed,'REV-29-1016','Keeper locknut '+str(i+1),[1,0,0,x/10,0,0,1,y/10,0,-1,0,48,0,0,0,1],
                   'bc0a4ee3749b4d93d5e313a6d328297825c5117828396b8593b274760afbf38d')
    bar(fixed,'Fixed rear bridge',-288,-guide-7,-270,guide+7,486)
    tie=242 if retained else 209
    footspan=242 if retained else 215
    bar(carriage,'Moving rear tie',-240,-tie,-226,tie,510)
    if coupled:
        for name,cx in [('Aft transverse slide surface',-43),('Front transverse slide surface',292)]:
            cut=[(-19,510),(19,510),(19,538.6),(12.5,538.6),(12.5,518.6),(15.6,518.6),
                 (15.6,514),(-15.6,514),(-15.6,518.6),(-12.5,518.6),(-12.5,538.6),(-19,538.6)]
            profile(carriage,name,[(cx+x,z) for x,z in cut],'y',-footspan,'484 mm')
    else:
        bar(carriage,'Aft transverse slide surface',-57,-footspan,-29,footspan,510,'8 mm')
        bar(carriage,'Front transverse slide surface',172+fore,-footspan,200+fore,footspan,510,'8 mm')
    # Short support skins begin at the end of the intended fixed feed ramp.
    # R2's fasteners retain only the fore-aft keeper and its anchor shoe.
    aft=602 if rev=='r5' else 585
    top=[(-65,aft),(205,550),(290,595),(315,698)]
    lower=lambda x: aft-12-(x+65)*(aft-550)/270
    if retained:
        # Relocate the *foot* together with its crossbar, outside the fall path.
        outline=top+[(315,686),(303,636.56),(303,518),(281,518),(281,578.2353),(205,538),
                     (-32,lower(-32)),(-32,518),(-54,518),(-54,lower(-54)),(-65,aft-12)]
        if coupled:
            for cx in (-43,292):
                at=outline.index((cx+11,518))
                outline[at:at+2]=[(cx+11,518),(cx+15.25,518),(cx+15.25,514),
                                  (cx-15.25,514),(cx-15.25,518),(cx-11,518)]
    else:
        outline=top+[(315,686),(290,583),(205,538),(197,lower(197)),(197,518),(175,518),
                     (175,lower(175)),(-32,lower(-32)),(-32,518),(-54,518),(-54,lower(-54)),(-65,aft-12)]
    for side,y in [('Left',110),('Right',-122)]:
        c=component(side+' splitting support')
        profile(c,side+' pipe rail',outline,'y',y,'SideWallGauge')
    if coupled:
        file=Path(CONFIG['project'])/'cad/red2-split-drop/r3_drive.py'
        loader=importlib.util.spec_from_file_location('private_opposed_rack',file)
        factory=importlib.util.module_from_spec(loader);loader.loader.exec_module(factory)
        factory.make(des,member,component,profile,bar,vendor,'r4' if rev=='r5' else rev)
    if rev=='r5':
        extra=Path(CONFIG['project'])/'cad/red2-split-drop/r5_guides.py'
        loader=importlib.util.spec_from_file_location('registered_red2_guides',extra)
        guides=importlib.util.module_from_spec(loader);loader.loader.exec_module(guides)
        guides.make(des,member,profile,bar)
    geo=fusion.JointGeometry.createByPoint(root.originConstructionPoint)
    def slider(name,moving,parent,axis,lo,hi):
        j=root.asBuiltJoints.createInput(member[moving],member[parent],geo)
        j.setAsSliderJointMotion(axis)
        joint=root.asBuiltJoints.add(j);joint.name=name
        lim=joint.jointMotion.slideLimits
        lim.minimumValue=lo/10;lim.isMinimumValueEnabled=True
        lim.maximumValue=hi/10;lim.isMaximumValueEnabled=True
        return joint
    dirs=fusion.JointDirections
    slider('Present to L1','Longitudinal carriage','Fixture bench rails',dirs.XAxisJointDirection,0,404)
    left=slider('Left support opening','Left splitting support','Longitudinal carriage',dirs.YAxisJointDirection,0,90)
    right=slider('Right support opening','Right splitting support','Longitudinal carriage',dirs.YAxisJointDirection,-90,0)
    if coupled:factory.link(des,member,left,right)

def load_witness(des,results,pose,piece_step):
    root=des.rootComponent
    target=results['scenarios']['nominal']['targetCenterMm']
    if pose=='seated':
        a,b=(-52 if CONFIG.get('revision')=='r5' else -35)/270,45/85
        x=205+57.15*(math.hypot(1,a)-math.hypot(1,b))/(b-a)
        target=[404+x,0,550+a*(x-205)+57.15*math.hypot(1,a)]
    cage=root.occurrences.addNewComponent(core.Matrix3D.create());comp=cage.component
    comp.name='CORAL A at timed gravity transfer'
    base=comp.xZConstructionPlane;sign=base.geometry.normal.y
    anchor=comp.constructionPlanes.createInput()
    anchor.setByOffset(base,core.ValueInput.createByReal(-15.08125/sign))
    sk=comp.sketches.add(comp.constructionPlanes.add(anchor))
    center=sk.modelToSketchSpace(point(target[0],-150.8125,target[2]))
    for radius in (57.15,50.8):sk.sketchCurves.sketchCircles.addByCenterRadius(center,radius/10)
    shaped=next(p for p in sk.profiles if p.profileLoops.count==2)
    ext=comp.features.extrudeFeatures.addSimple(shaped,core.ValueInput.createByReal(30.1625/sign),fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.bodies.item(0).name='CORAL-A finite annulus';sk.isLightBulbOn=False
    game=cage.bRepBodies.item(0)
    if piece_step.exists():raise ValueError('Fresh game piece exchange output required')
    if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(piece_step),comp)):
        raise ValueError('Game piece STEP export failed')
    prescribed={'Present to L1':404,'Left support opening':0 if pose in ('jam','seated') else 90}
    if CONFIG.get('revision') not in ('r3','r4','r5'):prescribed['Right support opening']=0 if pose=='seated' else -90
    joints={j.name:j for j in root.allAsBuiltJoints}
    for name,value in prescribed.items():joints[name].jointMotion.slideValue=value/10
    adsk.doEvents()
    brep=fusion.TemporaryBRepManager.get()
    def world(body,occurrence):
        solid=brep.copy(body.nativeObject)
        if not brep.transform(solid,occurrence.transform2):raise ValueError('Cannot place pipe witness operand')
        return solid
    witnesses=[]
    def box(s):
        b=s.preciseBoundingBox
        return [10*v for p in (b.minPoint,b.maxPoint) for v in p.asArray()]
    for o in root.allOccurrences:
        if o.fullPathName==cage.fullPathName:continue
        for b in o.bRepBodies:
            objects=core.ObjectCollection.create();objects.add(game);objects.add(b)
            setting=des.createInterferenceInput(objects);setting.areCoincidentFacesIncluded=False
            hits=des.analyzeInterference(setting)
            if hits is None:raise RuntimeError('Incomplete native pipe check')
            volume=sum(item.interferenceBody.volume*1000 for item in hits)
            common=world(game,cage)
            if not brep.booleanOperation(common,world(b,o),fusion.BooleanTypes.IntersectionBooleanType):
                raise ValueError('Unresolved temporary pipe intersection')
            temp_volume=1000*common.volume if common.isValid and common.lumps.count else 0
            witnesses.append(dict(body=o.fullPathName+'/'+b.name,boundsMm=box(b),volumeMm3=volume,temporaryCommonMm3=temp_volume))
    expected='Welded carriage with two captive guide tracks' if CONFIG.get('revision') in ('r4','r5') else 'Front transverse slide surface'
    bridge=next(r for r in witnesses if r['body'].endswith('/'+expected))
    if abs(bridge['boundsMm'][0]-{'r1':576,'r2':682,'r3':677,'r4':144,'r5':144}[CONFIG.get('revision','r1')])>.01:
        raise ValueError('Loaded state lost the presentation pose: '+str(bridge))
    return dict(pose=pose,worldCenterMm=target,pipe='coral-A',axis=[0,1,0],source='r1-gamepiece-control.json',
                pieceStep=dict(file=piece_step.name,sha256=sha(piece_step),centerMm=target,outerDiameterMm=114.3,
                               innerDiameterMm=101.6,lengthMm=301.625),
                timeAfterFreeReleaseS=None if pose=='seated' else results['scenarios']['nominal']['timeAfterFreeReleaseS'],pairs=witnesses,
                scope='One registered finite-pipe state. Late-release and all-path clearance, continued support and scoring still require separate checks.')

def inspect(des):
    root=des.rootComponent
    joint={j.name:j.jointMotion for j in root.allAsBuiltJoints}
    keys=['Present to L1','Left support opening','Right support opening']
    linked=CONFIG.get('revision') in ('r3','r4','r5')
    states=[]
    challenges=[('home',[0,0,0]),('present',[404,0,0]),('open',[404,90,-90]),
                ('before-release',[404,28,-28]) if linked else ('jam-left',[404,0,-90]),('reset',[0,0,0])]
    for label,commands in challenges:
        for k,v in zip(keys[:2] if linked else keys,commands):joint[k].slideValue=v/10
        adsk.doEvents()
        values=[10*joint[k].slideValue for k in keys]
        if any(abs(a-b)>.001 for a,b in zip(values,commands)):raise ValueError(str(values))
        parts={};placements={}
        for occ in root.allOccurrences:
            if CONFIG.get('revision') in ('r4','r5'):placements[occ.fullPathName]=list(occ.transform2.asArray())
            for body in occ.bRepBodies:
                bb=body.boundingBox
                parts[occ.fullPathName+'/'+body.name]=[10*q for p in (bb.minPoint,bb.maxPoint) for q in p.asArray()]
        states.append(dict(pose=label,commandsMm=commands,readbackMm=values,bodies=parts))
        if placements:states[-1]['worldTransformsCm']=placements
        if linked:states[-1]['pinionAngleRadians']=joint['Jaw pinion rotation'].rotationValue
    # Native coordinates must move in the commanded physical direction.
    home,present,opened,_,reset=states
    def delta(a,b,part,expected):
        k=next(k for k in a['bodies'] if k.endswith('/'+part))
        actual=[b['bodies'][k][i]-a['bodies'][k][i] for i in range(3)]
        if max(abs(x-y) for x,y in zip(actual,expected))>.02:raise ValueError(part+' world travel '+str(actual))
    moving_base='Welded carriage with two captive guide tracks' if CONFIG.get('revision') in ('r4','r5') else 'Left moving guide plate'
    for k in (moving_base,'Left pipe rail','Right pipe rail'):
        delta(home,present,k,[404,0,0]);delta(home,reset,k,[0,0,0])
    delta(present,opened,'Left pipe rail',[0,90,0]);delta(present,opened,'Right pipe rail',[0,-90,0])
    if CONFIG.get('revision') in ('r4','r5'):
        # All nested components in this fixture are explicitly fixed mounts.
        # The root cross-carriage, jaw roots and rotor have independent motions.
        def mult(a,b):return [sum(a[4*i+k]*b[4*k+j] for k in range(4)) for i in range(4) for j in range(4)]
        def inverse(a):
            r=[a[4*j+i] if i<3 and j<3 else float(i==j) for i in range(4) for j in range(4)]
            for i in range(3):r[4*i+3]=-sum(r[4*i+k]*a[4*k+3] for k in range(3))
            return r
        fixed=home['worldTransformsCm']
        for state in states:
            world=state['worldTransformsCm'];errors=[]
            for child in fixed:
                if '+' not in child:continue
                owner=child.split('+')[0]
                target=mult(mult(world[owner],inverse(fixed[owner])),fixed[child])
                measured=world[child]
                mmerr=10*math.dist([target[i] for i in (3,7,11)],[measured[i] for i in (3,7,11)])
                rot=max(abs(target[i]-measured[i]) for i in (0,1,2,4,5,6,8,9,10))
                errors.append(dict(child=child,owner=owner,translationMm=mmerr,rotationMatrixError=rot))
            state['rigidAttachmentChecks']=errors
            if any(x['translationMm']>.02 or x['rotationMatrixError']>1e-6 for x in errors):
                raise ValueError('Fixed attachment moved relative to its owner: '+str(max(errors,key=lambda x:x['translationMm']+x['rotationMatrixError'])))
    actual_materials={o.fullPathName+'/'+b.name:(b.material.name if b.material else None) for o in root.allOccurrences for b in o.bRepBodies}
    return dict(states=states,parametric=des.designType==fusion.DesignTypes.ParametricDesignType,materialReadback=actual_materials,
                sketches=sum(o.component.sketches.count for o in root.allOccurrences),
                extrusions=sum(o.component.features.extrudeFeatures.count for o in root.allOccurrences),
                parameters=[p.name for p in des.userParameters],asBuiltJoints=len(joint),motionLinks=root.motionLinks.count if linked else 0)

def run(context):
    app=core.Application.get();old=app.activeDocument
    before=[d.creationId for d in app.documents]
    root=Path(CONFIG['project']).resolve();out=root/'artifacts/split-drop-native'
    master=root/'cad/fusion-authority.json'
    module=Path(CONFIG['plugin'])/'fusion_authority.py'
    loader=importlib.util.spec_from_file_location('registered_design_authority',module)
    tools=importlib.util.module_from_spec(loader);loader.loader.exec_module(tools)
    rev=CONFIG.get('revision','r1')
    archive=out/('red2-split-drop-fixture-'+rev+'.f3d')
    doc=None;data={'before':before,'beforeActive':old.creationId if old else None,'stage':CONFIG['stage']}
    try:
        data['authority']=tools.verify(root,master,root/'artifacts/native-master-review/master-issue-03/lineage.json')
        if CONFIG['stage']=='census':
            path=Path(CONFIG['plugin'])/'fusion_issue_inventory.py'
            loader=importlib.util.spec_from_file_location('current_fixture_census',path)
            scanner=importlib.util.module_from_spec(loader);loader.loader.exec_module(scanner)
            jnames=['Present to L1','Left support opening','Right support opening']
            data['scans']=[]
            challenges=[('home',[]),('present',[404,0,0]),('clear',[404,90,-90]),
                        ('before-release',[404,28,-28]) if rev in ('r3','r4','r5') else ('jam-left',[404,0,-90])]
            for name,values in challenges:
                if rev in ('r3','r4','r5'):values=values[:2]
                pose={'id':name,'joints':[dict(type='asBuilt',occurrence='@root',name=k,coordinate='slideValue',nativeValue=x/10) for k,x in zip(jnames,values)]}
                target=out/((rev+'-' if rev!='r1' else '')+name+'-native.json')
                data['scans'].append(scanner.capture(app,archive,target,batch=256,pose=pose))
            data['sha256']=sha(archive);data['status']='pass'
            if not all(x['complete'] for x in data['scans']):raise ValueError('Census incomplete')
            print(json.dumps({'status':'pass','sha256':data['sha256'],'scans':data['scans']}))
            return
        if CONFIG['stage'] in ('material-preflight','finalize'):
            filename=root/'cad/red2-split-drop/r5_guides.py'
            loader=importlib.util.spec_from_file_location('material_issue',filename)
            guides=importlib.util.module_from_spec(loader);loader.loader.exec_module(guides)
            checkpoint=out/('red2-split-drop-fixture-'+rev+'-geometry.f3d')
            source=out/'red2-split-drop-fixture-r4.f3d' if CONFIG['stage']=='material-preflight' else checkpoint
            if sha(source)!=CONFIG['parentFixtureSha256']:raise ValueError('Unexpected parent fixture')
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
            des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            guides.assign_materials(des,app.materialLibraries.itemByName('Fusion Material Library'))
            if CONFIG['stage']=='material-preflight':
                data['actual']=[dict(occurrence=o.fullPathName,body=b.name,material=b.material.name,densityKgCm3=b.physicalProperties.mass/b.volume)
                                for o in des.rootComponent.allOccurrences for b in o.bRepBodies if any(v in o.fullPathName for v in ('REV-29','REV-41','WCP-'))]
                data['parentFixtureSha256']=sha(source);data['status']='pass'
                print(json.dumps(dict(status='pass',assigned=len(data['actual']))))
                return
            if archive.exists():raise ValueError('Fresh issued fixture path required')
            if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(archive))):raise ValueError('Corrected material export failed')
        elif CONFIG['stage']=='build':
            dest=out/('red2-split-drop-fixture-'+rev+'-geometry.f3d') if rev=='r5' else archive
            if dest.exists():raise ValueError('Choose a fresh fixture output before starting')
            doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
            des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            des.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType
            if des.designType!=fusion.DesignTypes.ParametricDesignType:des.designType=fusion.DesignTypes.ParametricDesignType
            make(des,rev,out/'supplier')
            if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(dest))):raise ValueError('Native fixture export failed')
            if rev=='r5':
                data['geometryCheckpoint']=dest.relative_to(root).as_posix();data['sha256']=sha(dest);data['status']='pass'
                print(json.dumps(dict(status='pass',geometryCheckpoint=data['geometryCheckpoint'],sha256=data['sha256'])))
                return
        else:
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
            des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        data['native']=inspect(des)
        if CONFIG.get('exportStep'):
            step=out/(rev+'-export.step')
            if step.exists():raise ValueError('Fresh exchange output required')
            if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError('STEP export failed')
            data['step']={'file':step.relative_to(root).as_posix(),'sha256':sha(step),'pose':'reset'}
        if CONFIG['stage']=='load':
            data['loadedWitness']=load_witness(des,json.loads((out/'r1-gamepiece-control.json').read_bytes()),CONFIG.get('witness','open'),out/(CONFIG['requestId']+'-coral-A.step'))
        if CONFIG['stage']=='jam':
            file=root/'cad/red2-split-drop/r3_drive.py'
            loader=importlib.util.spec_from_file_location('registered_native_jam',file)
            drive=importlib.util.module_from_spec(loader);loader.loader.exec_module(drive)
            data['diagnosisSource']=sha(file);data['jam']=drive.jam(des)
        data['sha256']=sha(archive);data['status']='pass'
    except Exception:
        data['status']='failed';data['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        data['after']=[d.creationId for d in app.documents]
        data['afterActive']=app.activeDocument.creationId if app.activeDocument else None
        data['restored']=before==data['after'] and data['afterActive']==data['beforeActive']
        (out/(CONFIG['requestId']+'.json')).write_text(json.dumps(data,indent=2))
    print(json.dumps({k:v for k,v in data.items() if k in ['status','error','sha256','restored']}))
