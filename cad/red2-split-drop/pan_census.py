"""Cold-page the complete installed pan and chassis at a measured joint coordinate."""
import hashlib
import importlib.util
import json
from pathlib import Path

import adsk.core


def run(context):
    project = Path(CONFIG['project'])
    plugin=Path(CONFIG['plugin'])
    for name,expected_digest in CONFIG['pluginFiles'].items():
        if hashlib.sha256((plugin/name).read_bytes()).hexdigest()!=expected_digest:
            raise ValueError('Immutable private runtime changed '+name)
    issuer=importlib.util.spec_from_file_location('pan_registered_source',plugin/'fusion_authority.py')
    authority=importlib.util.module_from_spec(issuer)
    issuer.loader.exec_module(authority)
    current_source=authority.verify(project,project/'cad/fusion-authority.json',project/'artifacts/native-master-review/master-issue-03/lineage.json')
    station = project / 'artifacts/red2-integrated/pan-native'
    archive = station / 'integrated-340mm-pan-r2.f3d'
    installation = json.loads((station / CONFIG['installation']).read_bytes())
    if installation['authority']['archiveSha256']!=current_source['archiveSha256']:
        raise ValueError('Installed training issue does not descend from the registered source')
    if installation.get('status') != 'pass' or not installation.get('restored'):
        raise ValueError('A finished cold installation receipt is required')
    if hashlib.sha256(archive.read_bytes()).hexdigest() != installation[archive.stem]:
        raise ValueError('Archive changed after cold installation')
    poses = {row['pose']:row for row in installation['cold']['poses']}
    if set(poses) != {'stowed','mid','discharge','stowed-reset'}:
        raise ValueError('Incomplete measured pose roster')
    pose = poses[CONFIG['pose']]
    if pose['errorMm'] > .02:
        raise ValueError('Unqualified driven coordinate')
    census = station / ('r2-census-' + pose['pose'] + '.json')
    script = Path(CONFIG['plugin']) / 'fusion_issue_inventory.py'
    if hashlib.sha256(script.read_bytes()).hexdigest() != CONFIG['collectorSha256']:
        raise ValueError('Collector changed')
    spec = importlib.util.spec_from_file_location('pan_census',script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    app = adsk.core.Application.get()
    before = [doc.creationId for doc in app.documents]
    joint = dict(type='asBuilt',occurrence='@root',name='Pan shaft pitch',
                 coordinate='rotationValue',nativeValue=pose['jointRadians'])
    result = module.capture(app,archive,census,batch=CONFIG['batch'],
                            pose=dict(id=pose['pose'],joints=[joint]))
    if result['complete']:
        receipt = json.loads(census.read_bytes())
        expected = (installation['cold']['foundationBodies'] + len(installation['allShopBodies'])
                    + sum(len(rows) for rows in installation['cold']['purchases'].values()))
        if len(receipt['bodies']) != expected or receipt['otherGeometry']:
            raise ValueError('Incomplete native body coverage')
        result.update(installedBodies=expected,allPossiblePairs=expected*(expected-1)//2)
    result['restored'] = before == [doc.creationId for doc in app.documents]
    if not result['restored']:
        raise ValueError('Native document list changed')
    print(json.dumps(result))
