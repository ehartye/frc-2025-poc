"""Run a bounded first-party interference batch on the cold standalone foundation."""
import hashlib
import importlib.util
import json
import traceback
from pathlib import Path
from adsk import core

def run(context):
    app=core.Application.get();before=[d.creationId for d in app.documents]
    old=app.activeDocument
    root=Path(CONFIG['project']);out=root/'artifacts/red2-integrated';plugin=Path(CONFIG['plugin'])
    revision=CONFIG.get('revision','r1')
    if revision not in ('r1','r2'):raise ValueError('Unregistered foundation issue')
    archive=out/('red2-integrated-foundation-'+revision+'.f3d')
    def module(name):
        spec=importlib.util.spec_from_file_location(name,plugin/(name+'.py'))
        m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
    data=dict(before=before)
    try:
        data['authority']=module('fusion_authority').verify(root,root/'cad/fusion-authority.json',root/'artifacts/native-master-review/master-issue-03/lineage.json')
        archiveSha=hashlib.sha256(archive.read_bytes()).hexdigest()
        if archiveSha!=CONFIG['archiveSha256']:raise ValueError('Unexpected foundation source')
        data['archiveSha256']=archiveSha
        data['result']=module('fusion_issue_inventory').capture(app,archive,out/(revision+'-parked-inclusive-native.json'),
            batch=CONFIG['batch'],pose={'id':'parked-foundation','joints':[]})
        data['status']='pass'
    except Exception:data['status']='failed';data['error']=traceback.format_exc()
    finally:
        data['after']=[d.creationId for d in app.documents]
        data['restored']=before==data['after'] and (not old or old.creationId==app.activeDocument.creationId)
        (out/(CONFIG['requestId']+'.json')).write_text(json.dumps(data,indent=2))
    print(json.dumps(data))
