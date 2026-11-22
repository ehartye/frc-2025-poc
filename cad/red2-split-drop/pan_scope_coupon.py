"""Two independent cold-opened fixtures for the original tower/shaft machining sequence."""
import hashlib
import importlib.util
import json
import traceback
from pathlib import Path

from adsk import core, fusion


def module(name,file):
    spec=importlib.util.spec_from_file_location(name,file)
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(context):
    root=Path(CONFIG['project'])
    out=root/'artifacts/red2-integrated/pan-scope-coupon'
    app=core.Application.get()
    before=[d.creationId for d in app.documents]
    active=app.activeDocument
    doc=None
    result=dict(before=before,cases={})
    def record():
        (out/(CONFIG['requestId']+'.json')).write_text(json.dumps(result,indent=2))
    try:
        for name,digest in CONFIG['inputs'].items():
            if sha(root/name)!=digest:
                raise ValueError('Changed pinned input '+name)
        script=root/'cad/red2-split-drop'
        geom=module('coupon_sketches',script/'chassis_native.py')
        machining=module('coupon_stock',script/'pan_scoped_stock.py')
        auth=module('coupon_authority',Path(CONFIG['plugin'])/'fusion_authority.py')
        result['authority']=auth.verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        stock=json.loads((root/'artifacts/red2-integrated/pan-shop-plan.json').read_bytes())
        original=json.loads((root/'artifacts/red2-integrated/pan-native/first-party-stock-r1.json').read_bytes())
        parts={p['id']:p for p in stock['items']}
        keys=['Idler headed stub hex shank','Starboard bearing tower']
        def design_of(d):
            return fusion.Design.cast(d.products.itemByProductType('DesignProductType'))
        for mode in ('automatic','explicit'):
            archive=out/(CONFIG['requestId']+'-'+mode+'.f3d')
            step=archive.with_suffix('.step')
            if archive.exists() or step.exists():raise ValueError('Choose a fresh fixture issue')
            result['stage']=mode+' fabrication'
            record()
            doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
            design=design_of(doc)
            design.designIntent=fusion.DesignIntentTypes.HybridDesignIntentType
            if design.designType!=fusion.DesignTypes.ParametricDesignType:
                design.designType=fusion.DesignTypes.ParametricDesignType
            groups={}
            for name in ('rotating group','fixed group'):
                occ=design.rootComponent.occurrences.addNewComponent(core.Matrix3D.create())
                occ.component.name=name
                groups[name]=occ.component
            for key,group in zip(keys,groups.values()):
                occ=group.occurrences.addNewComponent(core.Matrix3D.create())
                occ.component.name=key
                occ.component.attributes.add('Pan coupon','stock',key)
                machining.machine(occ.component,parts[key],geom,scoped=mode=='explicit')
            if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(archive))):
                raise ValueError('Failed native export')
            doc.close(False)
            doc=None
            result['stage']=mode+' cold readback'
            record()
            doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
            design=design_of(doc)
            bodies={k:[] for k in keys}
            for occ in design.rootComponent.allOccurrences:
                stamp=occ.component.attributes.itemByName('Pan coupon','stock')
                if not stamp:continue
                for body in occ.bRepBodies:
                    box=body.preciseBoundingBox
                    props=body.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                    bodies[stamp.value].append(dict(occurrence=occ.fullPathName,body=body.name,revision=body.revisionId,
                        boundsMm=[10*v for point in (box.minPoint,box.maxPoint) for v in point.asArray()],
                        volumeMm3=props.volume*1000))
            if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):
                raise ValueError('Failed fresh STEP')
            deltas={k:original['allStock'][k]['sourceVolumeMm3']-sum(row['volumeMm3'] for row in bodies[k]) for k in keys}
            result['cases'][mode]=dict(archiveSha256=sha(archive),stepSha256=sha(step),bodies=bodies,absentFromStockMm3=deltas)
            if mode=='automatic':
                shaft=sorted(bodies[keys[0]],key=lambda row:row['boundsMm'][1])
                if len(shaft)!=2:
                    raise ValueError('Original adjacent shaft split did not reproduce')
                gap=shaft[1]['boundsMm'][1]-shaft[0]['boundsMm'][4]
                result['cases'][mode]['gapMm']=gap
                if abs(gap-6.3)>.001 or abs(deltas[keys[0]]-879.9918)>.05:
                    raise ValueError('Different adjacent cut geometry')
            elif any(len(bodies[k])!=1 or abs(deltas[k])>.035 for k in keys):
                raise ValueError('Explicit target did not preserve original geometry')
            doc.close(False)
            doc=None
            record()
        result['status']='pass'
    except Exception:
        result['status']='failed'
        result['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if active and active.isValid:active.activate()
        result['restored']=before==[d.creationId for d in app.documents]
        record()
    print(json.dumps({k:result[k] for k in ['stage','status','restored','error'] if k in result}))
