"""Cold snapshot of real BRep vertices, then repair displaced purchased articles on a private branch."""
import hashlib
import importlib.util
import json
import traceback
from pathlib import Path
from adsk import core, fusion


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(values):
    native = core.Matrix3D.create()
    rows = [v / 10 if j == 3 else v for row in values for j, v in enumerate(row)]
    assert native.setWithArray(rows + [0, 0, 0, 1])
    return native


def supplier_witness(design, plan, plugin):
    location = plugin / 'fusion_issue_inventory.py'
    spec = importlib.util.spec_from_file_location('foundation_placement_inventory', location)
    inventory = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(inventory)
    by_name = {item['id']: item for item in plan['items'] if item['kind'] in ('plate','bought')}
    root = design.rootComponent
    owners = {o.fullPathName: o for o in root.allOccurrences if o.component.name in by_name}
    if len(owners) != 56:
        raise ValueError('Expected all 56 independent purchasing wrappers')
    rows = {}
    for occurrence in root.allOccurrences:
        if occurrence.component.bRepBodies.count == 0:
            continue
        ancestor = [wrapper for name,wrapper in owners.items()
                    if occurrence.fullPathName == name or occurrence.fullPathName.startswith(name+'+')]
        if not ancestor:
            continue
        owner, = ancestor
        name = owner.component.name
        if name in rows or occurrence.component.bRepBodies.count != 1:
            raise ValueError('Ambiguous physical article '+name)
        body = occurrence.component.bRepBodies.item(0)
        if not body.isSolid:
            raise ValueError('Supplier article is a surface '+name)
        proxy_error,points = inventory.placement_witness(body,occurrence,core.Point2D)
        expected = matrix(by_name[name]['matrixMm'])
        actual = occurrence.transform2
        pairs = []
        for point in points:
            base = core.Point3D.create(*point)
            target = base.copy()
            observed = base.copy()
            target.transformBy(expected)
            observed.transformBy(actual)
            pairs.append(dict(localMm=[10*x for x in point],observedWorldMm=[10*x for x in observed.asArray()],
                              expectedWorldMm=[10*x for x in target.asArray()],differenceMm=10*target.distanceTo(observed)))
        if len(pairs) != 3 or proxy_error > .001:
            raise ValueError('Independent root-space landmark check failed '+name)
        rows[name] = dict(path=occurrence.fullPathName,proxyWorldPointErrorMm=proxy_error,
                          points=pairs,maximumSourcePlacementErrorMm=max(p['differenceMm'] for p in pairs),
                          material=body.material.name,bodyRevision=body.revisionId,
                          sourceSha256=by_name[name]['articleSha256'])
    if set(rows) != set(by_name):
        raise ValueError('Missing purchased occurrence')
    return rows


def repair(app, design, project, plan, before):
    """Reimport each displaced article into its own registered editable wrapper."""
    root = design.rootComponent
    by_name = {o.component.name:o.component for o in root.allOccurrences
               if o.component.attributes.itemByName('Issue roster','article')}
    expected = [item for item in plan['items'] if item['id'] in before and
                before[item['id']]['maximumSourcePlacementErrorMm'] > .001]
    if len(expected) != 52:
        raise ValueError('Baseline no longer reproduces the 52 displaced copies')
    for item in expected:
        name = item['id']
        owner = by_name[name]
        source = project / item['article']
        if sha(source) != item['articleSha256'] or owner.occurrences.count != 1:
            raise ValueError('Unrecognized supplier fixture '+name)
        displaced = owner.occurrences.item(0)
        if 'manufacturer source' not in displaced.component.name:
            raise ValueError('Unrecognized immediate occurrence '+name)
        if not displaced.deleteMe():
            raise ValueError('Could not remove this incorrect instance '+name)
        imported = owner.occurrences.addNewComponent(core.Matrix3D.create())
        imported.component.name = name+' manufacturer issue'
        if not app.importManager.importToTarget(app.importManager.createSTEPImportOptions(str(source)), imported.component):
            raise ValueError('Exact original article failed to import '+name)
        imported.isGroundToParent = True
        imported.component.attributes.add('Supplier source','sha256',item['articleSha256'])
        for child in imported.component.allOccurrences:
            child.isGroundToParent = True
    for occurrence in root.allOccurrences:
        matches = [item for item in expected if occurrence.fullPathName.startswith(
                   by_name[item['id']].name+':') or ('+'+item['id']+':1+') in occurrence.fullPathName]
        if not matches or not occurrence.component.bRepBodies.count:
            continue
        item, = matches
        if item.get('grade'):
            grade = design.materials.itemByName('2025 '+item['grade'])
            if not grade or not grade.isValid:
                raise ValueError('Native material is missing '+item['id'])
            for body in occurrence.component.bRepBodies:
                body.material = grade
    return len(expected)


def run(context):
    app = core.Application.get()
    old = app.activeDocument
    baseline = [d.creationId for d in app.documents]
    project = Path(CONFIG['project'])
    folder = project / 'artifacts/red2-integrated'
    plugin = Path(CONFIG['plugin'])
    original = folder / 'red2-integrated-foundation-r1.f3d'
    successor = folder / 'red2-integrated-foundation-r2.f3d'
    derivative = successor.with_suffix('.step')
    planfile = folder / 'native-plan.json'
    data = dict(before=baseline)
    doc = None
    def write():
        (folder / (CONFIG['requestId']+'.json')).write_text(json.dumps(data,indent=2),encoding='utf8')
    def open_design(source):
        created = app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(source)))
        return created, fusion.Design.cast(created.products.itemByProductType('DesignProductType'))
    try:
        preflight = importlib.util.spec_from_file_location('foundation_authority',plugin/'fusion_authority.py')
        authority = importlib.util.module_from_spec(preflight); preflight.loader.exec_module(authority)
        data['authority'] = authority.verify(project,project/'cad/fusion-authority.json',project/'artifacts/native-master-review/master-issue-03/lineage.json')
        plan = json.loads(planfile.read_bytes())
        if sha(original) != CONFIG['archiveSha256'] or sha(planfile) != CONFIG['planSha256'] or plan['authority']['sha256'] != data['authority']['archiveSha256']:
            raise ValueError('Changed registered engineering source or foundation')
        if successor.exists() or derivative.exists():
            raise ValueError('Private issue already exists')
        data['beforeArchiveSha256'] = sha(original)
        doc,design = open_design(original)
        data['coldSourceVertices'] = supplier_witness(design, plan, plugin)
        write()
        if CONFIG['stage'] == 'inspect':
            failed = sum(r['maximumSourcePlacementErrorMm'] > .001 for r in data['coldSourceVertices'].values())
            if failed != 52:
                raise ValueError('The actual loaded R1 did not reproduce all 52 misplaced supplier solids')
            data['incorrectSupplierCount'] = failed
            data['status'] = 'failed-physical-registration-confirmed'
            return
        data['replacedSupplierInstances'] = repair(app,design,project,plan,data['coldSourceVertices'])
        if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(successor))):
            raise ValueError('Incomplete native successor export')
        data['archiveSha256'] = sha(successor)
        doc.close(False); doc = None
        doc,design = open_design(successor)
        data['coldCorrectedVertices'] = supplier_witness(design,plan,plugin)
        data['sourceIssuePlanSha256'] = sha(planfile)
        if not design.exportManager.execute(design.exportManager.createSTEPExportOptions(str(derivative),design.rootComponent)):
            raise ValueError('Could not export cold checked foundation')
        data['stepSha256'] = sha(derivative)
        if any(r['maximumSourcePlacementErrorMm']>.001 for r in data['coldCorrectedVertices'].values()):
            raise ValueError('Installed supplier solids remain displaced')
        for item in plan['items']:
            grade = item.get('grade')
            if grade and item['id'] in data['coldCorrectedVertices'] and data['coldCorrectedVertices'][item['id']]['material']!='2025 '+grade:
                raise ValueError('Cold supplier material changed '+item['id'])
        data['status'] = 'pass'
    except Exception:
        data['status'] = 'failed'
        data['error'] = traceback.format_exc()
    finally:
        if doc and doc.creationId not in baseline:
            doc.close(False)
        if old and old.isValid:
            old.activate()
        data['restored'] = baseline == [d.creationId for d in app.documents] and (not old or old.creationId == app.activeDocument.creationId)
        write()
        print(json.dumps({k:v for k,v in data.items() if k in ('status','restored','error','archiveSha256','stepSha256')}))
