"""Build the 340 mm pan with explicit, coupon-tested stock participants on the verified chassis."""
import hashlib
import importlib.util
import json
import math
import traceback
from pathlib import Path

import adsk
from adsk import core, fusion


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_stock(design, stock, originals, tools):
    actual=tools.inventory(design,stock['items'])
    if len(actual)!=68:
        raise ValueError('Expected exactly 68 distinct installed blanks')
    for part in stock['items']:
        item=part['id']
        expected=originals['allStock'][item]['sourceVolumeMm3']
        rows=actual[item]
        if len(rows)!=1 or rows[0]['material']!=part['material'] or abs(rows[0]['volumeMm3']-expected)>max(.035,expected*1e-5):
            raise ValueError('Machined stock differs from design '+str((item,rows,expected)))
    return actual


def run(context):
    app=core.Application.get()
    old=app.activeDocument
    before=[d.creationId for d in app.documents]
    root=Path(CONFIG['project'])
    out=root/'artifacts/red2-integrated'
    station=out/'pan-native'
    doc=None
    result=dict(before=before)
    def record():
        (station/(CONFIG['requestId']+'.json')).write_text(json.dumps(result,indent=2))
    def open_design(file):
        d=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(file)))
        return d,fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
    def save(design,file):
        if file.exists():
            raise ValueError('Refusing duplicate issue '+file.name)
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(file))):
            raise ValueError('Native export failed '+file.name)
        result[file.stem]=sha(file)
        record()
    def child(parent,name,ground):
        occ=parent.occurrences.addNewComponent(core.Matrix3D.create())
        occ.component.name=name
        occ.isGroundToParent=ground
        return occ
    try:
        for name,digest in CONFIG['inputs'].items():
            if sha(root/name)!=digest:raise ValueError('Changed dependency '+name)
        scripts=root/'cad/red2-split-drop'
        plugin=Path(CONFIG['plugin'])
        authority=module('r2_stock_authority',plugin/'fusion_authority.py')
        result['authority']=authority.verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        geom=module('r2_sketches',scripts/'chassis_native.py')
        tools=module('r2_stock_tools',scripts/'pan_r2.py')
        tools.ROOT=root
        machining=module('r2_target_machining',scripts/'pan_scoped_stock.py')
        original=module('r2_installed_readback',scripts/'pan_native.py')
        original.ROOT=root
        landmarks=module('r2_chassis_landmarks',scripts/'chassis_realign.py')
        stock=read(out/'pan-shop-plan.json')
        drive=read(out/'pan-drive-registration.json')
        independent=read(station/'first-party-stock-r1.json')
        bearing=read(out/'pan-bearing-placement.json')
        more=read(out/'pan-assembly-fasteners.json')['preparedForNextNativeIssue']
        coupon=read(out/'pan-scope-coupon/two-stock-bore-01.json')
        foundation=out/'red2-integrated-foundation-r2.f3d'
        if result['authority']['archiveSha256']!=stock['authority']['sha256'] or stock['foundationSha256']!=sha(foundation):
            raise ValueError('Assembly source registration mismatch')
        if coupon['status']!='pass' or abs(coupon['cases']['automatic']['gapMm']-6.3)>.001:
            raise ValueError('Missing two-stock source control')
        if not abs(coupon['cases']['explicit']['absentFromStockMm3']['Idler headed stub hex shank'])<.035:
            raise ValueError('Failed source-isolation control')
        checkpoint=station/'integrated-340mm-pan-r2-stock.f3d'
        archive=station/'integrated-340mm-pan-r2.f3d'
        step=archive.with_suffix('.step')
        if any(p.exists() for p in (archive,checkpoint,step)):
            raise ValueError('Retain earlier native issue before starting')
        result['stage']='open-verified-R2-foundation'
        record()
        doc,design=open_design(foundation)
        assembly=design.rootComponent
        fixed=child(assembly,'340 mm pan fixed installation',True)
        moving=child(assembly,'340 mm pan rotating installation',False)
        for index,part in enumerate(stock['items'],1):
            result['stage']='machine '+str(index)+' of 68'
            result['article']=part['id']
            record()
            parent=moving if part['moving'] else fixed
            comp=child(parent.component,part['id'],True).component
            comp.attributes.add('Pan stock','id',part['id'])
            comp.attributes.add('Pan stock','process',part['process'])
            machining.machine(comp,part,geom)
            material=design.materials.itemByName(part['material'])
            if not material or not material.isValid:raise ValueError('Missing issued material '+part['id'])
            comp.bRepBodies.item(0).material=material
        plane,_=geom.plane(assembly,2,580)
        sketch=assembly.sketches.add(plane)
        sketch.name='Actual pan axle axis at X205 Z580'
        point=sketch.sketchPoints.add(sketch.modelToSketchSpace(geom.point([205,0,580])))
        ref=fusion.JointGeometry.createByPoint(point)
        sketch.isLightBulbOn=False
        args=assembly.asBuiltJoints.createInput(moving,fixed,ref)
        args.setAsRevoluteJointMotion(fusion.JointDirections.YAxisJointDirection)
        joint=assembly.asBuiltJoints.add(args)
        joint.name='Pan shaft pitch'
        result['stage']='save isolated stock and editable joint'
        result['machinedStock']=check_stock(design,stock,independent,tools)
        save(design,checkpoint)
        for row in drive['components']+more:
            key=row['id']
            result['stage']='install '+key
            record()
            mobile=key=='REV-41-1620' or key.startswith('M3x8 adapter') or row.get('moving',False)
            if key in ('REV-21-1915 drive','REV-21-1915 idler'):
                side=key.split()[-1]
                config=next(item for item in bearing['parts'][side] if item['case']=='seated-clocked' and item['panAngleDeg']==0)
                matrix=config['installedMatrixMm']
                mobile=True
            else:
                matrix=tools.drive_to_root(row['frameMm'])
            tools.add_purchase(app,(moving if mobile else fixed).component,row,tools.placement(matrix))
        joint.jointMotion.rotationValue=-math.radians(70)
        adsk.doEvents()
        result['stage']='save installed pan stowed'
        save(design,archive)
        doc.close(False)
        doc=None
        result['stage']='cold-verify all individual bodies and measured joint poses'
        record()
        doc,design=open_design(archive)
        result['allShopBodies']=check_stock(design,stock,independent,tools)
        drive['components']+=more
        result['cold']=original.inspect(design,stock,drive,landmarks,plugin,geom)
        counts={name:len(rows) for name,rows in result['cold']['purchases'].items()}
        if len(counts)!=28 or sum(counts.values())!=78 or result['cold']['foundationBodies']!=148 or result['cold']['foundationPurchasedLandmarks']!=168:
            raise ValueError('Unexpected cold installed roster '+repr(counts))
        result['totalInstalledSolidBodies']=148+68+sum(counts.values())
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
            raise ValueError('Cold R2 STEP export failed')
        result['derivativeSha256']=sha(step)
        result['status']='pass'
    except Exception:
        result['status']='failed'
        result['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        result['restored']=before==[d.creationId for d in app.documents]
        record()
    print(json.dumps({k:result[k] for k in ['stage','status','restored','derivativeSha256','error'] if k in result}))
