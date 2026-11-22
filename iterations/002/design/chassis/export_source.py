from pathlib import Path
from hashlib import sha256
import importlib.util
import json
import shutil
import traceback
from adsk import core


def run(context):
    app=core.Application.get();prior=app.activeDocument;before=[d.creationId for d in app.documents]
    out=Path(CONFIG['resultDirectory']);inp=Path(CONFIG['jobInputs']);target=out/'mirror'
    r={'before':before,'status':'starting'}
    try:
        if before or prior or target.exists():raise ValueError('Fresh empty station and fresh private issue required')
        NATIVE_JOB['verifyDependencies']()
        listing=json.loads((inp/'iterations/002/design/chassis/export-inputs.json').read_bytes())
        for row in listing:
            src=inp/row['file'];dest=target/row['file']
            if sha256(src.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Changed canonical input')
            dest.parent.mkdir(exist_ok=True,parents=True);shutil.copyfile(src,dest)
        helper=Path(CONFIG['plugin'])/'fusion_authority.py'
        spec=importlib.util.spec_from_file_location('issued_authority',helper);authority=importlib.util.module_from_spec(spec)
        exec(compile(helper.read_bytes(),str(helper),'exec'),authority.__dict__)
        issue=target/'iterations/002/design/cad/issue-001';reg=authority.register_path(target)
        r['result']=authority.export_issue(app,target,reg,issue)
        r['lineageSha256']=authority.digest(issue/'lineage.json');r['status']='exported from active native master'
    except Exception:r.update(status='failed',error=traceback.format_exc())
    finally:
        for d in reversed(list(app.documents)):
            if d.creationId not in before:d.close(False)
        if prior and prior.isValid:prior.activate()
        r['after']=[d.creationId for d in app.documents];r['restored']=before==r['after'] and (not prior or app.activeDocument.creationId==prior.creationId)
        (out/'result.json').write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:r[k] for k in ('status','lineageSha256','restored','error') if k in r}))
