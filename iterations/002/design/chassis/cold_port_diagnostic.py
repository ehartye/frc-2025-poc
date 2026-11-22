"""Cold export the held construct for phase diagnosis; never promote it or modify the source archive."""
import hashlib
import json
from pathlib import Path
import traceback
from adsk import core,fusion


def module(path):
    import types
    ns={'__name__':'_cold_'+path.stem,'__file__':str(path)}
    exec(compile(path.read_bytes(),str(path),'exec'),ns)
    return types.SimpleNamespace(**ns)


def run(context):
    app=core.Application.get()
    before=[d.creationId for d in app.documents];prior=app.activeDocument
    report=dict(before=before);source=Path(CONFIG['jobInputs']);out=Path(CONFIG['resultDirectory'])
    study=source/'iterations/002/design';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    auth=module(Path(CONFIG['plugin'])/'fusion_authority.py');base=module(study/'chassis/native_corner.py')
    def save(phase):report['phase']=phase;(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    try:
        if before or prior:raise ValueError('Native station not empty')
        NATIVE_JOB['verifyDependencies']()
        reg=auth.register_path(source);report['authority']=auth.verify(source,reg,reg.parent/'issue-003/lineage.json')
        link=json.loads((study/'chassis/port-504-diagnostic.json').read_bytes())
        archive=study/'cad/private/port-504-negative.f3d'
        if sha(archive)!=link['files']['port-constructed.f3d']:raise ValueError('Wrong diagnostic source')
        save('Opening exact held 504')
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(archive)))
        des=base.design_of(doc);root=des.rootComponent
        rows=base.world_body_rows(des)
        if len(rows)!=504 or len({(r['path'],r['name']) for r in rows})!=504:raise ValueError('Cold physical census mismatch')
        report['worldBodies']=rows
        stock_rows=[r for r in rows if r['path'].startswith('Port terminal cartridge:1+')]
        if len(stock_rows)!=16:raise ValueError('Incomplete new anchor')
        report['anchor']=stock_rows;features=[];supplier={}
        for occ in root.allOccurrences:
            path=occ.fullPathName
            if not path.startswith('Port terminal cartridge:1+'):continue
            if 'Matched H1150 captive receiver' in path:
                for feat in occ.component.features.threadFeatures:
                    info=feat.threadInfo
                    features.append(dict(name=feat.name,designation=info.threadDesignation,threadClass=info.threadClass,
                         isInternal=info.isInternal,isModeled=feat.isModeled,health=str(feat.healthState),
                         isRightHanded=getattr(feat,'isRightHanded',None)))
            if '91251A542' in path or '97763A232' in path:
                b=occ.component.bRepBodies.item(0).createForAssemblyContext(occ)
                supplier[path]=dict(body=b.name,faces=b.faces.count,volumeMm3=b.volume*1000,
                    worldTransformCm=list(occ.transform2.asArray()))
        report.update(threads=features,purchased=supplier)
        if len(supplier)!=6 or len(features)!=2 or not all(r['isModeled'] and r['isInternal'] for r in features):
            raise ValueError('Missing modeled female thread or supplier body')
        exp=out/'port-504-negative.step'
        if exp.exists() or not des.exportManager.execute(des.exportManager.createSTEPExportOptions(str(exp))):raise ValueError('Incomplete diagnostic exchange')
        report.update(archiveSha256=sha(archive),diagnosticStepSha256=sha(exp),status='held geometry cold-reopened and exported')
        doc.close(False)
    except Exception:report.update(status='incomplete',error=traceback.format_exc())
    finally:
        for d in reversed(list(app.documents)):
            if d.creationId not in before:d.close(False)
        if prior and prior.isValid:prior.activate()
        report.update(after=[d.creationId for d in app.documents],restored=not app.activeDocument and app.documents.count==0)
        save('terminal')
    print(json.dumps({k:report[k] for k in ('status','restored','archiveSha256','diagnosticStepSha256','error') if k in report}))

