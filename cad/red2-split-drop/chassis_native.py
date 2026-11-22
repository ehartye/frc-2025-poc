"""Create and reopen the separate editable 2025 chassis. Runs inside Fusion."""
import hashlib
import importlib.util
import json
import traceback
from pathlib import Path
from adsk import core,fusion


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def mm(x):return core.ValueInput.createByString(str(x)+' mm')
def point(v):return core.Point3D.create(*[x/10 for x in v])
def load(p):return json.loads(p.read_bytes())


def plane(comp,axis,offset):
    ref=[comp.yZConstructionPlane,comp.xZConstructionPlane,comp.xYConstructionPlane][axis]
    normal=ref.geometry.normal.asArray()[axis]
    if abs(abs(normal)-1)>1e-8:raise ValueError('Non-Cartesian sketch plane')
    if abs(offset)<1e-8:return ref,normal
    inp=comp.constructionPlanes.createInput()
    inp.setByOffset(ref,mm(offset/normal))
    result=comp.constructionPlanes.add(inp);result.isLightBulbOn=False
    return result,normal


def profile(comp,axis,poly,name):
    face,sign=plane(comp,axis,poly[0][axis])
    sk=comp.sketches.add(face);sk.name=name
    pts=[sk.modelToSketchSpace(point(p)) for p in poly]
    first=prev=None
    for i,p in enumerate(pts):
        line=sk.sketchCurves.sketchLines.addByTwoPoints(prev or p,first if i==len(pts)-1 else pts[i+1])
        if first is None:first=line.startSketchPoint
        prev=line.endSketchPoint
    if sk.profiles.count!=1:raise ValueError('Open stock profile '+name)
    sk.isLightBulbOn=False
    return sk.profiles.item(0),sign


def prism(comp,poly,z,travel,name,operation):
    low=[[x,y,z] for x,y in poly]
    bottom,sign=profile(comp,2,low,name+' bottom sketch')
    if any(abs(k)>1e-8 for k in travel[:2]):
        top,_=profile(comp,2,[[v[i]+travel[i] for i in range(3)] for v in low],name+' upper sketch')
        loft=comp.features.loftFeatures.createInput(operation)
        loft.loftSections.add(bottom);loft.loftSections.add(top)
        feat=comp.features.loftFeatures.add(loft)
    else:
        feat=comp.features.extrudeFeatures.addSimple(bottom,mm(travel[2]/sign),operation)
    feat.name=name


def bore(comp,start,direction,radius,depth,label,operation):
    axis=next(i for i,v in enumerate(direction) if abs(v)>.9999)
    p,sign=plane(comp,axis,start[axis])
    sk=comp.sketches.add(p);sk.name=label+' circle'
    sk.sketchCurves.sketchCircles.addByCenterRadius(sk.modelToSketchSpace(point(start)),radius/10)
    feat=comp.features.extrudeFeatures.addSimple(sk.profiles.item(0),mm(depth*direction[axis]/sign),operation)
    feat.name=label;sk.isLightBulbOn=False


def make(doc,app,root,plan):
    des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    des.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType
    if des.designType!=fusion.DesignTypes.ParametricDesignType:des.designType=fusion.DesignTypes.ParametricDesignType
    assembly=des.rootComponent
    assembly.attributes.add('Engineering source','scope','Separate private chassis foundation. Full engineering master transfer pending complete robot review.')
    new=fusion.FeatureOperations.NewBodyFeatureOperation;cut=fusion.FeatureOperations.CutFeatureOperation
    owners={};donors={}
    def group(label):
        if label in owners:return owners[label]
        occ=assembly.occurrences.addNewComponent(core.Matrix3D.create())
        occ.component.name=label;occ.isGroundToParent=True
        owners[label]=occ.component
        return owners[label]
    def transform(raw):
        m=core.Matrix3D.create()
        if raw:
            values=[v/10 if i==3 else v for r in raw for i,v in enumerate(r)]+[0,0,0,1]
            if not m.setWithArray(values):raise ValueError('Invalid installed matrix')
        return m
    def obtain(owner,spec):
        occ=owner.occurrences.addNewComponent(transform(spec.get('matrixMm')))
        occ.component.name=spec['id'];occ.isGroundToParent=True
        return occ.component
    for spec in plan['items']:
        parent=group(spec['group']);comp=obtain(parent,spec);name=spec['id']
        kind=spec['kind'];material_name=spec.get('grade')
        comp.attributes.add('Issue roster','article',name)
        comp.attributes.add('Issue roster','physical material',material_name or spec['materialBasis'])
        if kind in ('plate','bought'):
            f=root/spec['article']
            if sha(f)!=spec['articleSha256']:raise ValueError('Changed supplier '+name)
            if f not in donors:
                source=comp.occurrences.addNewComponent(core.Matrix3D.create())
                source.component.name=f.stem+' manufacturer source'
                if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(f)),source.component):
                    raise ValueError('Failed source '+name)
                donors[f]=source.component
            else:comp.occurrences.addExistingComponent(donors[f],core.Matrix3D.create())
            for child in comp.allOccurrences:child.isGroundToParent=True
            comp.attributes.add('Supplier source','sha256',sha(f))
        if kind=='stock':
            r=spec['stock'];h=r['heightMm']
            vec=r.get('sectionTranslationMm',[0,0,h])
            prism(comp,r['planMm'],r['zMinMm'],vec,'Cut '+name+' stock',new)
            if 'openCavityPlanMm' in r:
                z0,z1=r['cavityZMm'];prism(comp,r['openCavityPlanMm'],z0,[vec[0],vec[1],z1-z0],'Hollow '+name,cut)
            for port in spec.get('ports',[]):
                bore(comp,port['centerMm'],port['axis'],port['radiusMm'],port['depthMm'],'Weld insert bore '+port['boss'],cut)
            comp.attributes.add('Fabrication','process',r['process'])
        if kind in ('saddle','shim'):
            lo,hi=spec['stockBoundsMm']
            poly=[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]
            prism(comp,poly,lo[2],[0,0,hi[2]-lo[2]],'Cut '+kind+' stock',new)
            for item in spec['holes']:
                bore(comp,item['startMm'],item['axis'],item['radiusMm'],item['depthMm'],item['process'],cut)
        if kind=='boss':
            bore(comp,[-48,0,0],[-1,0,0],6.5,14,'Turn stock to diameter 13',new)
            bore(comp,[-48,0,0],[-1,0,0],2.0447,12,'Tap #10-32 to 10 mm full thread',cut)
        for body in comp.bRepBodies:
            body.name=name
    return des


def assign(app,des,plan,report):
    if not app.activeDocument or app.activeDocument.creationId!=des.parentDocument.creationId:
        raise ValueError('Target foundation is not the active native design')
    library=app.materialLibraries.itemByName('Fusion Material Library')
    materials=list(library.materials)
    def material(terms):
        matches=[m for m in materials if all(t in m.name.lower() for t in terms)]
        if not matches:raise ValueError('Missing stock base '+repr(terms))
        return min(matches,key=lambda m:len(m.name))
    # Explicit data/inference separation: an article whose density was never
    # specified cannot contribute a qualified physical mass or load capacity.
    grades={}
    for name,terms in [('6061-T6',['6061','t6']),('5052-H32 aluminum',['5052','h32']),('Sealed Air Ethafoam 220, 37 kg/m3',['polyethylene, low density']),('stainless shim',['stainless']),
                       ('REV alloy steel, black oxide, grade unspecified',['steel, alloy'])]:
        grades[name]=des.materials.addByCopy(material(terms),'2025 '+name)
    items={r['id']:r for r in plan['items']}
    owners={o.fullPathName:o.component.attributes.itemByName('Issue roster','article').value
            for o in des.rootComponent.allOccurrences if o.component.attributes.itemByName('Issue roster','article')}
    counts={}
    for occ in des.rootComponent.allOccurrences:
        if not occ.component.bRepBodies.count:continue
        key,=[v for k,v in owners.items() if occ.fullPathName==k or occ.fullPathName.startswith(k+'+')]
        grade=items[key].get('grade')
        if not grade:continue
        this=des.materials.itemByName('2025 '+grade)
        if this is None or not this.isValid:raise ValueError('Material missing from active design '+grade)
        for body in occ.component.bRepBodies:
            report['lastAssignment']=dict(occurrence=occ.fullPathName,body=body.name,rosterId=key,grade=grade,materialId=this.id,
                materialParentType=this.parent.objectType,designRoot=des.rootComponent.id,bodyDesignRoot=body.parentComponent.parentDesign.rootComponent.id,
                document=app.activeDocument.creationId,whoseDesign=des.parentDocument.creationId)
            if body.parentComponent.parentDesign.rootComponent.id!=des.rootComponent.id:
                raise ValueError('Body belongs to a different native design')
            if body.material.id!=this.id:body.material=this
            if body.material.id!=this.id:raise ValueError('Material assignment mismatch '+key)
            if grade.startswith('Sealed Air'):
                initial=body.physicalProperties.mass/body.volume
                for prop in this.materialProperties:
                    if prop.id in ('structural_Density','thermal_Density'):
                        q=core.FloatProperty.cast(prop)
                        q.value=37. if q.units=='KilogramPerCubicMeter' else q.value*.000037/initial
                if abs(body.physicalProperties.mass/body.volume-.000037)>1e-7:raise ValueError('Incorrect bulk foam density')
            counts[grade]=counts.get(grade,0)+1
    report['assignedOccurrenceCounts']=counts
    if sum(counts.values())!=108:raise ValueError('Incomplete physical material roster')


def inspect(des,plan):
    obs={};items={r['id']:r for r in plan['items']}
    owners={o.fullPathName:o.component.attributes.itemByName('Issue roster','article').value
            for o in des.rootComponent.allOccurrences if o.component.attributes.itemByName('Issue roster','article')}
    for occ in des.rootComponent.allOccurrences:
        if not occ.bRepBodies.count:continue
        key,=[v for k,v in owners.items() if occ.fullPathName==k or occ.fullPathName.startswith(k+'+')]
        for body in occ.bRepBodies:
            if key in obs or not body.isSolid:raise ValueError('Non-solid or duplicate '+key)
            box=body.preciseBoundingBox
            measured=[10*x for pt in (box.minPoint,box.maxPoint) for x in pt.asArray()]
            vol=body.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume*1000
            row=items[key];diff=max(abs(x-y) for x,y in zip(measured,row['expectedBoundsMm']))
            err=abs(vol-row['expectedVolumeMm3'])
            obs[key]=dict(path=occ.fullPathName,name=body.name,boundsMm=measured,volumeMm3=vol,
                          maxBoundsDifferenceMm=diff,volumeDifferenceMm3=err,
                          materialId=body.material.id,fusionPhysicalMaterial=body.material.name,gradeBasis=row.get('grade') or row['materialBasis'],
                          datumPass=diff<.035 and err<max(.04,abs(vol)*.00001) and (not row.get('grade') or body.material.name=='2025 '+row['grade']))
    if set(obs)!=set(items):raise ValueError('Missing native roster '+repr(set(items)-set(obs)))
    return obs


def run(context):
    app=core.Application.get();old=app.activeDocument
    before=[d.creationId for d in app.documents];doc=None
    root=Path(CONFIG['project']);out=root/'artifacts/red2-integrated';plan_file=out/'native-plan.json'
    revision=CONFIG.get('revision','r1')
    if revision not in ('r1','r2'):raise ValueError('Unregistered foundation issue')
    archive=out/('red2-integrated-foundation-'+revision+'.f3d');step=archive.with_suffix('.step')
    live=dict(before=before,stage=CONFIG['stage'])
    try:
        if sha(plan_file)!=CONFIG['planSha256']:raise ValueError('Changed chassis plan')
        plan=load(plan_file)
        modpath=Path(CONFIG['plugin'])/'fusion_authority.py'
        spec=importlib.util.spec_from_file_location('foundation_authority',modpath)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        live['authority']=module.verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        if live['authority']['archiveSha256']!=plan['authority']['sha256']:raise ValueError('Authority changed')
        checkpoint=out/'red2-integrated-foundation-r1-geometry.f3d'
        if CONFIG['stage']=='build':
            if checkpoint.exists():raise ValueError('Foundation geometry checkpoint already exists')
            doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
            des=make(doc,app,root,plan)
            if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(checkpoint))):raise ValueError('Native checkpoint failed')
            live['checkpointSha256']=sha(checkpoint)
            doc.close(False);doc=None
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(checkpoint)))
            des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            assign(app,des,plan,live)
            if archive.exists():raise ValueError('Already issued')
            if not des.exportManager.execute(des.exportManager.createFusionArchiveExportOptions(str(archive))):raise ValueError('Native archive export failed')
        else:
            if sha(archive)!=CONFIG['archiveSha256']:raise ValueError('Wrong native archive')
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
            des=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            live['bodies']=inspect(des,plan)
            if CONFIG['stage']=='inspect':
                if sha(step)!=CONFIG['stepSha256']:raise ValueError('The bound first cold derivative changed')
            else:
                if step.exists():raise ValueError('Fresh STEP output required')
                if not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(step),des.rootComponent)):raise ValueError('Exchange failed')
            live['stepSha256']=sha(step)
            if not all(b['datumPass'] for b in live['bodies'].values()):raise ValueError('Cold body comparison failed')
        live['archiveSha256']=sha(archive);live['planSha256']=sha(plan_file);live['status']='pass'
    except Exception:
        live['status']='failed';live['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        live['restored']=before==[d.creationId for d in app.documents] and (not old or app.activeDocument.creationId==old.creationId)
        (out/(CONFIG['requestId']+'.json')).write_text(json.dumps(live,indent=2))
    print(json.dumps({k:v for k,v in live.items() if k in ['status','error','restored','archiveSha256']}))
