"""Repair automatic boolean participation and the two original bearing placements."""
import hashlib,json,math,traceback,importlib.util
from pathlib import Path
import adsk
from adsk import core,fusion
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_bytes())
def get_module(name,p):
    sp=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);return m

def inventory(design,rows):
    by={p['id']:p for p in rows}
    result={n:[] for n in by}
    for o in design.rootComponent.allOccurrences:
        att=o.component.attributes.itemByName('Pan stock','id')
        if not att:continue
        if att.value not in result:raise ValueError('Unregistered stock '+att.value)
        for i,b in enumerate(o.bRepBodies):
            box=b.preciseBoundingBox
            result[att.value].append(dict(path=o.fullPathName,localOrdinal=i,body=b.name,
                volumeMm3=b.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy).volume*1000,
                material=b.material.name,massKg=b.physicalProperties.mass,
                worldBoundsMm=[10*x for point in (box.minPoint,box.maxPoint) for x in point.asArray()]))
    return result

def repair_cuts(design,proof,stock,log,record):
    parts={p['id'] for p in stock['items']};root=design.rootComponent
    initial=inventory(design,stock['items'])
    shaft='Idler headed stub hex shank'
    old=sorted(initial[shaft],key=lambda p:p['worldBoundsMm'][1])
    if len(old)!=2 or abs(old[1]['worldBoundsMm'][1]-old[0]['worldBoundsMm'][4]-6.3)>.001:raise ValueError('Known cold split not present')
    log['knownShaftGapBeforeMm']=old[1]['worldBoundsMm'][1]-old[0]['worldBoundsMm'][4]
    # Scope each existing shop cut to its own workpiece. Inspect the body
    # while the timeline is immediately before the operation, as Fusion requires.
    cuts=[]
    for occ in root.allOccurrences:
        if occ.component.name not in parts:continue
        owner=occ.component
        for features in (owner.features.extrudeFeatures,owner.features.loftFeatures):
            for feat in features:
                if feat.operation==fusion.FeatureOperations.CutFeatureOperation:
                    cuts.append((feat.timelineObject.index,owner,feat))
    if len(cuts)<10:raise ValueError('Incomplete native feature roster')
    log['cutRoster']=[dict(stock=c.name,cut=f.name,timeline=i) for i,c,f in cuts];record()
    done=[]
    for index,owner,feature in sorted(cuts,key=lambda q:-q[0]):
        log['lastCut']=dict(item=owner.name,feature=feature.name,index=index);record()
        feature.timelineObject.rollTo(True)
        if owner.bRepBodies.count!=1:raise ValueError('Expected one owning blank before cut '+owner.name)
        target=owner.bRepBodies.item(0)
        feature.participantBodies=[target]
        done.append(dict(stock=owner.name,feature=feature.name))
    design.timeline.moveToEnd();adsk.doEvents()
    log['scopedCuts']=done
    result=inventory(design,stock['items'])
    for row in stock['items']:
        name=row['id'];actual=result[name];expected=proof['allStock'][name]['sourceVolumeMm3']
        if len(actual)!=1 or abs(actual[0]['volumeMm3']-expected)>max(.035,expected*1e-5):
            raise ValueError('Corrected native stock differs '+str((name,len(actual),[v['volumeMm3'] for v in actual],expected)))
    log['correctedStockPhysicalBodies']=len(result);record()

def add_purchase(app,owner,row,native_matrix):
    spec=ROOT/row['source']
    if sha(spec)!=row['sourceSha256']:raise ValueError('Changed supplier input '+row['id'])
    group=owner.occurrences.addNewComponent(core.Matrix3D.create());group.component.name=row['id']+' pan purchase'
    group.isGroundToParent=True;group.component.attributes.add('Pan purchase','id',row['id'])
    group.component.attributes.add('Pan purchase','source',row['sourceSha256'])
    registered=group.component.occurrences.addNewComponent(native_matrix);registered.component.name='Registered '+row['id']
    registered.isGroundToParent=True
    if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(spec)),registered.component):
        raise ValueError('Genuine supplier import failed '+row['id'])
    for child in registered.component.allOccurrences:child.isGroundToParent=True

def placement(raw):
    m=core.Matrix3D.create()
    if not m.setWithArray([v/10 if j==3 else v for row in raw for j,v in enumerate(row)]+[0,0,0,1]):raise ValueError('Improper native matrix')
    return m
def drive_to_root(m):
    return [m[0][:3]+[205+m[0][3]],m[2][:3]+[216+m[2][3]],[-v for v in m[1][:3]]+[580-m[1][3]]]

def run(context):
    global ROOT
    ROOT=Path(CONFIG['project']);scripts=ROOT/'cad/red2-split-drop';out=ROOT/'artifacts/red2-integrated';station=out/'pan-native'
    base=station/'integrated-340mm-pan-r1.f3d';checkpoint=station/'integrated-340mm-pan-r2-stock.f3d'
    archive=station/'integrated-340mm-pan-r2.f3d';derivative=archive.with_suffix('.step')
    app=core.Application.get();old=app.activeDocument;before=[d.creationId for d in app.documents];doc=None
    log={'before':before};plugin=Path(CONFIG['plugin'])
    def record():(station/(CONFIG['requestId']+'.json')).write_text(json.dumps(log,indent=2),encoding='utf8')
    def open_native(source):
        d=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
        return d,fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
    def save(design,file):
        if file.exists():raise ValueError('Refuse to replace native checkpoint '+file.name)
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(file))):raise ValueError('Native checkpoint failed '+file.name)
        log[file.stem]=sha(file);record()
    try:
        log['authority']=get_module('r2_pan_authority',plugin/'fusion_authority.py').verify(ROOT,ROOT/'cad/fusion-authority.json',ROOT/'artifacts/native-master-review/master-issue-03/lineage.json')
        plan=read(out/'pan-shop-plan.json');drive=read(out/'pan-drive-registration.json')
        checks=read(station/'first-party-stock-r1.json');bearing=read(out/'pan-bearing-placement.json');bolts=read(out/'pan-assembly-fasteners.json')['preparedForNextNativeIssue']
        for name,digest in CONFIG['inputs'].items():
            if sha(ROOT/name)!=digest:raise ValueError('Pinned input changed '+name)
        if sorted(checks['failed'])!=['430 mm pan floor','Idler headed stub hex shank']:raise ValueError('Expected two R1 negatives')
        if archive.exists() or checkpoint.exists() or derivative.exists():raise ValueError('An R2 issue or checkpoint already exists')
        log['stage']='cold-R1-control';record()
        doc,design=open_native(base);root=design.rootComponent;joint=root.asBuiltJoints.itemByName('Pan shaft pitch')
        if joint is None:raise ValueError('R1 has no driven axis')
        saved=joint.jointMotion.rotationValue
        joint.jointMotion.rotationValue=0;adsk.doEvents()
        log['stage']='limit-booleans-to-their-owning-stock';record()
        repair_cuts(design,checks,plan,log,record)
        # Durable native recovery boundary before installing any bought parts.
        joint=root.asBuiltJoints.itemByName('Pan shaft pitch');joint.jointMotion.rotationValue=saved;adsk.doEvents()
        save(design,checkpoint)
        joint.jointMotion.rotationValue=0;adsk.doEvents()
        groups={o.component.name:o for o in root.occurrences};fixed=groups['340 mm pan fixed installation'].component;moving=groups['340 mm pan rotating installation'].component
        updated=[]
        for side,sign in [('drive',1),('idler',-1)]:
            name='REV-21-1915 '+side
            actual=next(o for o in fixed.occurrences if o.component.name==name+' pan purchase')
            if not actual.deleteMe():raise ValueError('Could not retire the incorrect bearing '+name)
            case=next(row for row in bearing['parts'][side] if row['case']=='seated-clocked' and row['panAngleDeg']==0)
            current=next(row for row in drive['components'] if row['id']==name)
            add_purchase(app,moving,current,placement(case['installedMatrixMm']))
            updated.append(name)
        for row in bolts:
            log['stage']='installing '+row['id'];record()
            add_purchase(app,fixed,row,placement(drive_to_root(row['frameMm'])))
        log['correctedBearings']=updated
        joint.jointMotion.rotationValue=saved;adsk.doEvents()
        log['stage']='save-corrected-stowed-assembly';record();save(design,archive)
        doc.close(False);doc=None;doc,design=open_native(archive)
        log['stage']='cold-readback';record()
        log['allShopBodies']=inventory(design,plan['items'])
        if any(len(b)!=1 or abs(b[0]['volumeMm3']-checks['allStock'][n]['sourceVolumeMm3'])>max(.035,b[0]['volumeMm3']*1e-5) for n,b in log['allShopBodies'].items()):
            raise ValueError('Cold single-stock source comparison failed')
        native=get_module('r2_pan_inspection',scripts/'pan_native.py');native.ROOT=ROOT
        drive['components']+=bolts
        log['cold']=native.inspect(design,plan,drive,get_module('r2_pan_chassis',scripts/'chassis_realign.py'),plugin,get_module('r2_pan_stock',scripts/'chassis_native.py'))
        body_counts = {name:len(rows) for name,rows in log['cold']['purchases'].items()}
        if len(body_counts)!=28 or sum(body_counts.values())!=78 or len(log['allShopBodies'])!=68 or log['cold']['foundationBodies']!=148 or log['cold']['foundationPurchasedLandmarks']!=168:
            raise ValueError('Unexpected installed physical roster '+repr(body_counts))
        log['totalInstalledSolidBodies']=148+68+sum(body_counts.values())
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(derivative),design.rootComponent)):raise ValueError('Cold R2 STEP failed')
        log['derivativeSha256']=sha(derivative);log['status']='pass'
    except Exception:
        log['status']='failed';log['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        log['restored']=before==[d.creationId for d in app.documents] and (not old or old.creationId==app.activeDocument.creationId)
        record()
    print(json.dumps({k:v for k,v in log.items() if k in ['stage','status','error','restored','derivativeSha256']}))
