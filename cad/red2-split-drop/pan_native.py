"""Build the project-owned articulated pan on a new copy of the cold R2 chassis."""
import hashlib
import importlib.util
import json
import math
import traceback
from pathlib import Path
import adsk
from adsk import core,fusion

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result

def sha(file):return hashlib.sha256(file.read_bytes()).hexdigest()

def read(file):return json.loads(file.read_bytes())

def install(app,design,stock,drive,root_dir,geom):
    root=design.rootComponent
    new=fusion.FeatureOperations.NewBodyFeatureOperation;cut=fusion.FeatureOperations.CutFeatureOperation
    def child(comp,name,ground=True,mat=None):
        obj=comp.occurrences.addNewComponent(mat or core.Matrix3D.create())
        obj.component.name=name
        obj.isGroundToParent=ground
        return obj
    def sections(comp,part,profiles,name,op):
        axis=part['axis']
        first,sgn=geom.profile(comp,axis,profiles[0],name+' first cut section')
        if all(abs((u[k]-v[k]))<1e-8 for u,v in zip(profiles[0],profiles[1]) for k in range(3) if k!=axis):
            delta=profiles[1][0][axis]-profiles[0][0][axis]
            feature=comp.features.extrudeFeatures.addSimple(first,geom.mm(delta/sgn),op)
        else:
            last,_=geom.profile(comp,axis,profiles[1],name+' end section')
            loft=comp.features.loftFeatures.createInput(op)
            loft.loftSections.add(first);loft.loftSections.add(last)
            feature=comp.features.loftFeatures.add(loft)
        feature.name=name
    fixed=child(root,'340 mm pan fixed installation')
    moving=child(root,'340 mm pan rotating installation',False)
    for row in stock['items']:
        owner=moving if row['moving'] else fixed
        comp=child(owner.component,row['id']).component
        comp.attributes.add('Pan stock','id',row['id'])
        comp.attributes.add('Pan stock','process',row['process'])
        sections(comp,row,row['sections'],'Fabricate '+row['id'],new)
        if row['void']:sections(comp,row,row['void'],'Hollow '+row['id'],cut)
        for hole in row['bores']:
            geom.bore(comp,hole['start'],hole['direction'],hole['radius'],hole['depth'],'Bore '+row['id'],cut)
        for hole in row['hexes']:
            xx,zz=hole['center'];r=hole['flats']/math.sqrt(3)
            pts=[[xx+r*math.cos(math.radians(i*60+30)),zz+r*math.sin(math.radians(i*60+30))] for i in range(6)]
            sections(comp,{'axis':1},[[[x,y,z] for x,z in pts] for y in (hole['y0'],hole['y1'])],'Broach socket flats',cut)
        if comp.bRepBodies.count!=1:raise ValueError('Unexpected stock solid count '+row['id'])
        body=comp.bRepBodies.item(0);body.name=row['id']
        mat=design.materials.itemByName(row['material'])
        if mat is None or not mat.isValid:raise ValueError('Missing saved R2 material '+row['material'])
        body.material=mat
    # Each bought component is imported independently. Nested reusable source
    # definitions on R1 changed the installed transform of repeated purchases.
    for row in drive['components']:
        name=row['id'];source=root_dir/row['source']
        if sha(source)!=row['sourceSha256']:raise ValueError('Changed purchase issue '+name)
        mobile=name=='REV-41-1620' or name.startswith('M3x8 adapter') or row.get('moving',False)
        comp=child((moving if mobile else fixed).component,name+' pan purchase').component
        comp.attributes.add('Pan purchase','id',name)
        comp.attributes.add('Pan purchase','source',row['sourceSha256'])
        m=row['frameMm'];datum=stock['driveDatumYmm']
        xyz=[m[0][:3]+[205+m[0][3]],m[2][:3]+[datum+m[2][3]],[-v for v in m[1][:3]]+[580-m[1][3]]]
        native=core.Matrix3D.create()
        vals=[v/10 if j==3 else v for r in xyz for j,v in enumerate(r)]+[0,0,0,1]
        assert native.setWithArray(vals)
        wrapper=child(comp,'Registered '+name,mat=native)
        if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(source)),wrapper.component):
            raise ValueError('Could not import genuine source '+name)
        for occ in wrapper.component.allOccurrences:occ.isGroundToParent=True
    at,sgn=geom.plane(root,2,580)
    sketch=root.sketches.add(at);sketch.name='Actual pan axle axis at X205 Z580'
    pt=sketch.sketchPoints.add(sketch.modelToSketchSpace(geom.point([205,0,580])))
    ref=fusion.JointGeometry.createByPoint(pt);sketch.isLightBulbOn=False
    inputs=root.asBuiltJoints.createInput(moving,fixed,ref)
    inputs.setAsRevoluteJointMotion(fusion.JointDirections.YAxisJointDirection)
    joint=root.asBuiltJoints.add(inputs);joint.name='Pan shaft pitch'
    return joint

def inspect(design,stock,drive,check_foundation,plugin,geom):
    root=design.rootComponent
    joint=root.asBuiltJoints.itemByName('Pan shaft pitch')
    if joint is None:raise ValueError('Missing editable revolute')
    # Independently visit an actual tip corner of the floor's native BRep.
    floor=next(o for o in root.allOccurrences if o.component.name=='430 mm pan floor')
    body=floor.component.bRepBodies.item(0)
    end=next(v for v in body.vertices if math.dist(v.geometry.asArray(),[54.5,16.1,58.0])<.00001)
    def world():
        proxy=end.createForAssemblyContext(floor)
        return [10*q for q in proxy.geometry.asArray()]
    def command(rad):
        joint.jointMotion.rotationValue=rad;adsk.doEvents()
        if abs(joint.jointMotion.rotationValue-rad)>.00001:raise ValueError('Joint refused physical coordinate')
        return world()
    trial=command(math.radians(5))
    sign=1 if trial[2]>580 else -1
    states=[]
    for label,deg in [('stowed',70),('mid',34),('discharge',-2),('stowed-reset',70)]:
        actual=command(sign*math.radians(deg))
        ideal=[205+340*math.cos(math.radians(deg)),161,580+340*math.sin(math.radians(deg))]
        error=math.dist(ideal,actual)
        states.append(dict(pose=label,angleDeg=deg,jointRadians=joint.jointMotion.rotationValue,actualTipMm=actual,expectedTipMm=ideal,errorMm=error))
        if error>.02:raise ValueError('Wrong installed hinge frame '+str(states))
    command(0)
    shop={};purchases={};old={}
    for o in root.allOccurrences:
        if not o.bRepBodies.count:continue
        inv=o.component.attributes.itemByName('Issue roster','article')
        for b in o.bRepBodies:
            if not b.isSolid:raise ValueError('Unexpected native surface '+o.fullPathName)
            att=o.component.attributes.itemByName('Pan stock','id')
            if att:
                name=att.value;shop[name]=dict(material=b.material.name,massKg=b.physicalProperties.mass,body=b.name,occurrence=o.fullPathName)
                continue
            for part in drive['components']:
                if ('+'+part['id']+' pan purchase:') in o.fullPathName:
                    purchases.setdefault(part['id'],[]).append(dict(path=o.fullPathName,body=b.name,volumeMm3=b.volume*1000))
                    break
            else:
                old[o.fullPathName+'/'+b.name]=b.material.name
    if len(old)!=148 or set(shop)!={p['id'] for p in stock['items']} or set(purchases)!={p['id'] for p in drive['components']}:
        raise ValueError('Cold body coverage mismatch '+repr((len(old),len(shop),len(purchases),set(p['id'] for p in drive['components'])-set(purchases))))
    for part in stock['items']:
        if shop[part['id']]['material']!=part['material']:raise ValueError('Cold stock material mismatch '+part['id'])
    old_pts=check_foundation.supplier_witness(design,read(ROOT/'artifacts/red2-integrated/native-plan.json'),plugin)
    if max(v['maximumSourcePlacementErrorMm'] for v in old_pts.values())>.001:raise ValueError('Retained chassis placement changed')
    command(sign*math.radians(70))
    return dict(poses=states,shop=shop,purchases=purchases,foundationBodies=len(old),foundationPurchasedLandmarks=3*len(old_pts),
        movingStockKg=sum(shop[p['id']]['massKg'] for p in stock['items'] if p['moving']),fixedStockKg=sum(shop[p['id']]['massKg'] for p in stock['items'] if not p['moving']))

def run(context):
    global ROOT
    app=core.Application.get();old=app.activeDocument;before=[d.creationId for d in app.documents]
    ROOT=Path(CONFIG['project']);out=ROOT/'artifacts/red2-integrated'
    src=out/'red2-integrated-foundation-r2.f3d';folder=out/'pan-native';archive=folder/'integrated-340mm-pan-r1.f3d';step=archive.with_suffix('.step')
    plugin=Path(CONFIG['plugin']);stockfile=out/'pan-shop-plan.json';drivefile=out/'pan-drive-registration.json'
    log=dict(before=before);doc=None
    def record():(folder/(CONFIG['requestId']+'.json')).write_text(json.dumps(log,indent=2),encoding='utf8')
    def opening(source):
        d=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
        return d,fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
    try:
        auth=module('source_authority',plugin/'fusion_authority.py')
        log['authority']=auth.verify(ROOT,ROOT/'cad/fusion-authority.json',ROOT/'artifacts/native-master-review/master-issue-03/lineage.json')
        if any(sha(f)!=CONFIG[k] for f,k in [(src,'foundationSha256'),(stockfile,'stockSha256'),(drivefile,'driveSha256')]):raise ValueError('Local source issue changed')
        stock,drive=read(stockfile),read(drivefile)
        if log['authority']['archiveSha256']!=drive['authority']['sha256'] or stock['foundationSha256']!=sha(src):raise ValueError('Incorrect authority')
        modules=ROOT/'cad/red2-split-drop';geom=module('pan_native_sketches',modules/'chassis_native.py')
        if archive.exists() or step.exists():raise ValueError('Native issue already exists')
        log['stage']='open-foundation';record()
        doc,design=opening(src)
        log['stage']='install';record()
        joint=install(app,design,stock,drive,ROOT,geom)
        floor=next(o for o in design.rootComponent.allOccurrences if o.component.name=='430 mm pan floor')
        tip=next(v for v in floor.component.bRepBodies.item(0).vertices if math.dist(v.geometry.asArray(),[54.5,16.1,58])<.00001)
        joint.jointMotion.rotationValue=math.radians(5);adsk.doEvents()
        sign=1 if tip.createForAssemblyContext(floor).geometry.z>58 else -1
        joint.jointMotion.rotationValue=sign*math.radians(70);adsk.doEvents()
        if abs(tip.createForAssemblyContext(floor).geometry.z*10-(580+340*math.sin(math.radians(70))))>.02:
            raise ValueError('Pre-export actual pan tip did not reach the stowed location')
        log['stage']='capture-stowed-native';record()
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(archive))):raise ValueError('Native issue failed')
        log['archiveSha256']=sha(archive);record()
        doc.close(False);doc=None
        doc,design=opening(archive);log['stage']='cold-inspection';record()
        log['cold']=inspect(design,stock,drive,module('pan_foundation_checks',modules/'chassis_realign.py'),plugin,geom)
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):raise ValueError('Cold parked export failed')
        log['stepSha256']=sha(step);log['status']='pass'
    except Exception:
        log['status']='failed';log['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        log['restored']=before==[d.creationId for d in app.documents] and (not old or old.creationId==app.activeDocument.creationId)
        record()
    print(json.dumps({k:v for k,v in log.items() if k in ('status','stage','error','restored','archiveSha256','stepSha256')}))
