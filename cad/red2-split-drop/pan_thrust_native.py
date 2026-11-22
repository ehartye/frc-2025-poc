"""Machine and reopen the bilateral idler support on a disposable copy of the R2 robot."""
import hashlib
import importlib.util
import itertools
import json
import traceback
from pathlib import Path

import adsk
from adsk import core, fusion


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


def machine(design, plan, geom, machining):
    assembly = design.rootComponent
    moving = assembly.occurrences.itemByName('340 mm pan rotating installation:1')
    fixed = assembly.occurrences.itemByName('340 mm pan fixed installation:1')
    if not moving or not fixed:
        raise ValueError('Missing pan installation parents')
    old_parts = [('One-piece stepped shaft','Idler headed stub hex shank',moving),
                 ('Cross-drilled moving socket','Starboard driven hex socket',moving),
                 ('Drilled starboard bearing tower','Starboard bearing tower',fixed)]
    new_parts = [('Stationary diaphragm with four bosses','Piloted bilateral bearing support',fixed),
                 ('Hardened removable end cap','Turned 6061-T6 removable cap',moving),
                 ('Match-ground axial shim','Match-ground 302 stainless thrust shim',moving)]
    parts = {}
    for label, previous, parent in old_parts:
        o = parent.component.occurrences.itemByName(previous+':1')
        if not o or o.component.bRepBodies.count != 1:
            raise ValueError('Missing source blank '+previous)
        parts[label] = o.component
    for label, name, parent in new_parts:
        o = parent.component.occurrences.addNewComponent(core.Matrix3D.create())
        o.component.name = name
        o.isGroundToParent = True
        parts[label] = o.component
    for key, comp in parts.items():
        comp.attributes.add('Bilateral thrust machining','part',key)
        comp.attributes.add('Bilateral thrust machining','expectedVolumeMm3',str(plan['stock'][key]['volumeMm3']))
    NEW = fusion.FeatureOperations.NewBodyFeatureOperation
    CUT = fusion.FeatureOperations.CutFeatureOperation

    def feature(comp, profile, travel, label, operation):
        body0 = comp.bRepBodies.item(0) if comp.bRepBodies.count and operation != 'new' else None
        if operation == 'cut':
            if not body0 or comp.bRepBodies.count != 1:
                raise ValueError('One target required for '+label)
            result = machining.extrusion(comp,profile,travel,CUT,geom,body0)
        else:
            result = machining.extrusion(comp,profile,travel,NEW,geom,None)
        result.name = label
        if operation == 'join':
            # Never have Fusion implicitly choose a neighboring stock to join.
            if not body0 or result.bodies.count != 1:
                raise ValueError('One explicit added solid required for '+label)
            tools=core.ObjectCollection.create()
            tools.add(result.bodies.item(0))
            args=comp.features.combineFeatures.createInput(body0,tools)
            args.operation=fusion.FeatureOperations.JoinFeatureOperation
            args.isKeepToolBodies=False
            comp.features.combineFeatures.add(args).name='Join '+label+' to its stock'
        if comp.bRepBodies.count != 1:
            raise ValueError('One physical stock must remain after '+label)

    def circle(comp, axis, start, length, radius, label, operation, inside=None):
        pl, sign = geom.plane(comp, axis, start[axis])
        sk = comp.sketches.add(pl)
        sk.name = label+' profile'
        center = sk.modelToSketchSpace(geom.point(start))
        sk.sketchCurves.sketchCircles.addByCenterRadius(center,radius/10)
        if inside is not None:
            sk.sketchCurves.sketchCircles.addByCenterRadius(center,inside/10)
        loops=2 if inside is not None else 1
        profiles=[p for p in sk.profiles if p.profileLoops.count==loops]
        if len(profiles)!=1:raise ValueError('Ambiguous profile '+label)
        feature(comp,profiles[0],length/sign,label,operation)
        sk.isLightBulbOn=False

    shaft,socket,tower,support,cap,shim = (parts[k] for k in [
        'One-piece stepped shaft','Cross-drilled moving socket','Drilled starboard bearing tower',
        'Stationary diaphragm with four bosses','Hardened removable end cap','Match-ground axial shim'])
    u0,u1,u2,u3 = plan['stackPlanesUmm']
    uj = plan['rigidCapSeatUmm']
    useat = plan['towerScrewSeatUmm']
    def axial(comp,u,r,len_mm,label,op,inside=None):
        circle(comp,1,[205,-u,580],-len_mm,r,label,op,inside)
    axial(shaft,202,11.35,u0-202,'Machine integral thrust shoulder','join')
    axial(shaft,u0,6.33,uj-u0,'Finish shaft pilot journal','join')
    axial(shaft,uj-10,1.5,11,'Axial cap screw thread envelope','cut')
    circle(shaft,2,[205,-177,564],24,1.65,'Radial cross screw clearance','cut')
    circle(socket,2,[205,-177,564],24,1.65,'Cross screw socket thru clearance','cut')
    circle(socket,2,[205,-177,588],10,1.5,'Cross screw socket thread envelope','cut')
    contour=[[178,-u2,563],[232,-u2,563],[232,-u2,597],[178,-u2,597]]
    profile,sign=geom.profile(support,1,contour,'Machine diaphragm plate')
    feature(support,profile,(u2-u1)/sign,'Machine diaphragm plate','new')
    axial(support,u1-.5,7.15,5.,'Housing washer relief','cut')
    for n,u in enumerate([u1-.60,u2],1):
        axial(support,u,13.6,.60,'Housing register lip '+str(n),'join',inside=11.95)
    for n,(x,z) in enumerate(itertools.product([184.,226.],[570.,590.]),1):
        circle(tower,1,[x,-190.,z],-10,1.5,'Tower ear thread envelope '+str(n),'cut')
        circle(support,1,[x,-198.15,z],-(u1-198.15),4.5,'Support boss '+str(n),'join')
        circle(support,1,[x,-198.,z],-(u2-198.+.5),1.7,'Support screw clearance '+str(n),'cut')
        circle(support,1,[x,-useat,z],-(u2-useat+.1),3.1,'Socket head recess '+str(n),'cut')
    axial(cap,u3,11.35,3.,'Turn cap abutment','new',inside=1.65)
    axial(shim,uj,5.5,plan['selectedShimMm'],'Punch and finish sizing shim','new',inside=1.7)
    grade_al=design.materials.itemByName('2025 6061-T6')
    if not grade_al:raise ValueError('Parent lacks commissioned 6061-T6 stock')
    for k,comp in parts.items():
        body=comp.bRepBodies.item(0)
        if k!='Match-ground axial shim':
            body.material=grade_al
        else:
            label='Precision Brand 22999 full-hard 302 stainless shim'
            template=design.materials.itemByName('2025 stainless shim')
            if not template:
                raise ValueError('Expected a design-owned stainless family material')
            grade=design.materials.itemByName(label) or design.materials.addByCopy(template,label)
            body.material=grade
            original_density=body.physicalProperties.mass/body.volume
            changed=set()
            for prop in grade.materialProperties:
                if prop.id in ('structural_Density','thermal_Density'):
                    value=core.FloatProperty.cast(prop)
                    if value is None:raise ValueError('Missing scalar material density')
                    if value.units=='KilogramPerCubicMeter':value.value=7900.
                    else:value.value*=.0079/original_density
                    changed.add(prop.id)
            if not changed or abs(body.physicalProperties.mass/body.volume-.00790)>.00001:
                raise ValueError('Specified 302 material density readback failed')
            comp.attributes.add('Bilateral thrust machining','specifiedMaterial','Precision Brand 22999 full-hard cold rolled Type 302 stainless; ATI 302 nominal density 7.90 g/cm3. No borrowed yield allowable.')
        body.name=comp.name
    return parts


def inventory(design, plan):
    rows={}
    for occ in design.rootComponent.allOccurrences:
        att=occ.component.attributes.itemByName('Bilateral thrust machining','part')
        if not att:continue
        k=att.value
        if k in rows or occ.component.bRepBodies.count!=1:raise ValueError('Unmatched article '+k)
        raw=occ.component.bRepBodies.item(0)
        vol=1000*raw.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume
        expected=plan['stock'][k]['volumeMm3']
        rows[k]=dict(path=occ.fullPathName,body=raw.name,volumeMm3=vol,expectedVolumeMm3=expected,densityKgCm3=raw.physicalProperties.mass/raw.volume,
                     material=raw.material.name,deltaMm3=vol-expected)
        if abs(vol-expected)>max(.04,expected*2e-5):raise ValueError('Wrong machined stock '+str(rows[k]))
    if set(rows)!=set(plan['stock']):raise ValueError('Missing machined articles')
    total=design.rootComponent.bRepBodies.count+sum(o.component.bRepBodies.count for o in design.rootComponent.allOccurrences)
    if total!=297:raise ValueError('Expected 297 after adding three new shop parts; saw '+str(total))
    if '302' not in rows['Match-ground axial shim']['material'] or abs(rows['Match-ground axial shim']['densityKgCm3']-.0079)>.00001:
        raise ValueError('Unqualified authored shim material')
    return dict(parts=rows,totalBodies=total,
                shimGradeVerified='302' in rows['Match-ground axial shim']['material'])


def run(context):
    app=core.Application.get()
    before=[d.creationId for d in app.documents]
    active=app.activeDocument
    root=Path(CONFIG['jobInputs'])
    supplier=root/'artifacts/red2-integrated/pan-thrust'
    station=Path(CONFIG['resultDirectory'])
    station.mkdir(exist_ok=True)
    info=dict(before=before)
    def record():
        (station/(CONFIG['requestId']+'.json')).write_text(json.dumps(info,indent=2))
    def open_archive(file):
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(file)))
        return doc,fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    def driver(design):
        j=design.rootComponent.asBuiltJoints.itemByName('Pan shaft pitch')
        if not j:raise ValueError('Cold source lost the actual pan revolute')
        return j.jointMotion
    doc=None
    try:
        for file,want in CONFIG['inputs'].items():
            if sha(root/file)!=want:raise ValueError('Changed pinned input '+file)
        scripts=root/'cad/red2-split-drop'
        geom=module('r3_planes',scripts/'chassis_native.py')
        machining=module('r3_scoped',scripts/'pan_scoped_stock.py')
        authority=module('r3_authority',Path(CONFIG['plugin'])/'fusion_authority.py')
        info['authority']=authority.verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        plan=read(supplier/'r3-native-plan.json')
        if plan['authority']['sha256']!=info['authority']['archiveSha256'] or plan['expectedSolids']!=325:
            raise ValueError('Missing exact parent or 325-article scope')
        parent=root/'artifacts/red2-integrated/pan-native/integrated-340mm-pan-r2.f3d'
        state=station/'integrated-340mm-pan-r3-machined.f3d'
        step=state.with_suffix('.step')
        if sha(parent)!=plan['parentArchive'] or state.exists() or step.exists():
            raise ValueError('Parent changed or the new issue already exists')
        doc,design=open_archive(parent)
        old=driver(design).rotationValue
        info.update(stage='machine six articles',sourceStowRadians=old);record()
        driver(design).rotationValue=0
        adsk.doEvents()
        machine(design,plan,geom,machining)
        info['machined']=inventory(design,plan)
        driver(design).rotationValue=old
        adsk.doEvents()
        info['stage']='save machined private robot'
        record()
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(state))):
            raise ValueError('Archive export failed')
        info['nativeSha256']=sha(state);record()
        doc.close(False);doc=None
        NATIVE_JOB['verifyDependencies']()
        doc,design=open_archive(state)
        info['cold']=inventory(design,plan)
        if abs(driver(design).rotationValue-old)>1e-6:raise ValueError('Cold stow changed')
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
            raise ValueError('Cold STEP export failed')
        info.update(stage='cold machined robot',stepSha256=sha(step),status='machining-verified')
    except Exception:
        info.update(status='failed',error=traceback.format_exc())
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if active and active.isValid:active.activate()
        info['restored']=before==[d.creationId for d in app.documents]
        record()
    print(json.dumps({k:info[k] for k in ('stage','status','restored','nativeSha256','stepSha256','error') if k in info}))
