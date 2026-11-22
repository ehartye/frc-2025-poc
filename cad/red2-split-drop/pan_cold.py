"""Cold reopen the already-issued integrated pan with the immutable released checker."""
import json
import hashlib
import traceback
import importlib.util
from pathlib import Path
from adsk import core,fusion

def mod(name,location):
    spec=importlib.util.spec_from_file_location(name,location);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def run(context):
    root=Path(CONFIG['project']);out=root/'artifacts/red2-integrated';station=out/'pan-native'
    scripts=root/'cad/red2-split-drop';plugin=Path(CONFIG['plugin'])
    app=core.Application.get();old=app.activeDocument;before=[d.creationId for d in app.documents]
    doc=None;log=dict(before=before)
    def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
    def load(f):return json.loads(f.read_bytes())
    def record():(station/(CONFIG['requestId']+'.json')).write_text(json.dumps(log,indent=2),encoding='utf8')
    try:
        source=station/'integrated-340mm-pan-r1.f3d';prior=load(station/'pan-installation-01.json')
        if not prior.get('restored') or sha(source)!=CONFIG['archiveSha256'] or sha(source)!=prior['archiveSha256']:raise ValueError('Last native request and archive are not at their closed checkpoint')
        log['archiveSha256']=sha(source)
        log['authority']=mod('pan_authority',plugin/'fusion_authority.py').verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        if sha(out/'pan-shop-plan.json')!=CONFIG['stockSha256'] or sha(out/'pan-drive-registration.json')!=CONFIG['driveSha256']:raise ValueError('Issue plan differs from the saved native issue')
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
        design=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        native=mod('cold_pan',scripts/'pan_native.py');native.ROOT=root
        log['stage']='cold-articulation-and-registration';record()
        log['cold']=native.inspect(design,load(out/'pan-shop-plan.json'),load(out/'pan-drive-registration.json'),mod('pan_foundation',scripts/'chassis_realign.py'),plugin,mod('pan_stock',scripts/'chassis_native.py'))
        step=source.with_suffix('.step')
        if step.exists():raise ValueError('Derivative already issued')
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(step),design.rootComponent)):raise ValueError('Unable to export from the cold native')
        log['stepSha256']=sha(step);log['status']='pass'
    except Exception:log['status']='failed';log['error']=traceback.format_exc()
    finally:
        if doc and doc.creationId not in before:doc.close(False)
        if old and old.isValid:old.activate()
        log['restored']=before==[d.creationId for d in app.documents] and (not old or app.activeDocument.creationId==old.creationId)
        record()
    print(json.dumps({k:v for k,v in log.items() if k in ('status','error','restored','archiveSha256','stepSha256')}))
