"""Measure a bounded batch on the single registered complete engineering source."""
import importlib.util
import json
import traceback
from pathlib import Path
from adsk import core

def run(context):
    app=core.Application.get()
    root=Path(CONFIG['project']).resolve()
    folder=root/'artifacts/complete-census'
    def load(name):
        spec=importlib.util.spec_from_file_location(name,Path(CONFIG['plugin'])/(name+'.py'))
        m=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m
    def state():
        return dict(documents=[d.creationId for d in app.documents],active=app.activeDocument.creationId if app.activeDocument else None)
    authority=load('fusion_authority')
    inventory=load('fusion_issue_inventory')
    reg=root/'cad/fusion-authority.json'
    original=app.activeDocument
    record=dict(before=state(),pairBudget=CONFIG['batch'])
    try:
        record['authority']=authority.verify(root,reg,root/'artifacts/native-master-review/master-issue-03/lineage.json')
        archive=authority.resolve(root,authority.load(reg)['source']['archive'])
        record['scan']=inventory.capture(app,archive,folder/'parked-native.json',batch=CONFIG['batch'],pose={'id':'saved','joints':[]})
        record['status']='measured'
    except Exception:
        record['status']='failed'
        record['error']=traceback.format_exc()
    finally:
        if original and original.isValid:original.activate()
        record['after']=state()
        record['restored']=record['before']==record['after']
        (folder/(CONFIG['requestId']+'.json')).write_text(json.dumps(record,indent=2))
    print(json.dumps(record))
