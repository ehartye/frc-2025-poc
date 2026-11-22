"""Run one unresolved vendor-body comparison from an explicitly bounded subprocess."""
import json
import subprocess
import sys
from functools import lru_cache
from pathlib import Path

from output_mount import ROOT, digest, read


def compare(plugin, id, folder):
    sys.path.insert(0,str(plugin/'scripts'))
    import component_shape
    p=ROOT/'artifacts/red2-integrated'
    previous=read(p/'pan-native/first-party-purchases-r2-70-retained.json')
    registered=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert previous['authority']==registered and digest(ROOT/registered['file'])==registered['sha256']
    current=read(p/'pan-thrust/fixed-races/pan-r3-fixed-races-01.json')
    issue=p/'pan-thrust/fixed-races/checked-297.step'
    assert digest(issue)==current['stepSha256']
    target=previous['roster'][id]
    catalog=read(p/'pan-assembly-fasteners.json')['preparedForNextNativeIssue']
    article,=[a for a in catalog if a['id']==target['part']]
    source=ROOT/article['source']
    assert digest(source)==target['sourceSha256']
    record=dict(placedBody=id,sourceArticle=target,sourceSha256=digest(source),sourceFile=article['source'],
                archiveSha256=current['archiveSha256'],stepSha256=digest(issue),authority=registered,
                verifierSha256=digest(plugin/'scripts/component_shape.py'),status='matching current occurrence')
    path=folder/'bounded-purchase-71-progress.json'
    def save():path.write_text(json.dumps(record,indent=2)+'\n')
    save()
    original=component_shape.load_solids
    labels={}
    placed,other=original(issue,labels)
    if len(placed)!=297 or other:raise ValueError('Lost cold issue scope')
    matches=[name for name,row in labels.items() if row['occurrencePath']==target['path']]
    if len(matches)!=1:raise ValueError(('Need unique occurrence in new derivative',matches))
    real,=matches
    source_cache=lru_cache(None)(original)
    def loaded(file):
        return (placed,other) if file==issue else source_cache(file)
    component_shape.load_solids=loaded
    record.update(currentPlacedBody=real,status='running unmodified first-party directed comparison');save()
    result=component_shape.compare_components(issue,[(real,source,target['supplierSolid'],target['transform'])])
    record.update(status='completed',result=result);save()


if __name__=='__main__':
    folder=ROOT/'artifacts/red2-integrated/pan-native'
    plugin=Path(sys.argv[1])
    if len(sys.argv)>2:
        compare(plugin,'M3X40',folder)
    else:
        work=[sys.executable,str(Path(__file__).resolve()),str(plugin),'--worker']
        record=dict(worker=Path(__file__).name,limitSeconds=150)
        try:
            run=subprocess.run(work,capture_output=True,text=True,timeout=150)
            record.update(exitCode=run.returncode,stderr=run.stderr[-1800:])
        except subprocess.TimeoutExpired:
            record.update(status='bounded comparison unresolved; isolated worker terminated')
        evidence=folder/'bounded-purchase-71-progress.json'
        if evidence.is_file():record['lastCompletedStage']=read(evidence)
        (folder/'bounded-purchase-71.json').write_text(json.dumps(record,indent=2)+'\n')
        print({k:v for k,v in record.items() if k!='lastCompletedStage'})
