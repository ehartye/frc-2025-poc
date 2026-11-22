"""Index method, native source, observed defects and limited successor controls."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
def read(path):return json.loads(path.read_bytes())
def ref(path):return {'file':path.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
def main():
    reg=ROOT/'cad/fusion-authority.json'
    spec=read(reg)
    if ref(ROOT/spec['source']['archive']['file'])!=spec['source']['archive']:raise ValueError('Authority drift')
    p=ROOT/'artifacts/complete-census'
    out=ROOT/'artifacts/split-drop-native'
    q=read(p/'parked-native.json')
    if not q['contacts']['complete'] or q['archiveSha256']!=spec['source']['archive']['sha256']:raise ValueError('Incomplete or stale source')
    bodies=q['bodies']
    pairs=q['contacts']['pairs']
    pos=[row for row in pairs if max(row['temporaryCommonMm3'],row['interferenceMm3'])>0]
    def describe(row):
        return dict(pair=[bodies[i]['path']+'/'+bodies[i]['name'] for i in row['pair']],
                    temporaryCommonMm3=row['temporaryCommonMm3'],nativeInterferenceMm3=row['interferenceMm3'])
    census=dict(source=ref(p/'parked-native.json'),bodies=len(bodies),candidatePairs=len(pairs),
                positivePairs=len(pos),over100Mm3=sum(max(x['temporaryCommonMm3'],x['interferenceMm3'])>100 for x in pos),
                largest=[describe(x) for x in sorted(pos,key=lambda x:-x['temporaryCommonMm3'])[:10]],
                aboveMethodDelta001=[describe(x) for x in pos if abs(x['temporaryCommonMm3']-x['interferenceMm3'])>.01],
                checkpointReceipts=[ref(p/f'batch-{i:02}.json') for i in range(1,7)])
    revisions=[]
    for rev in ['r1','r2']:
        n=read(out/(rev+'-load-02.json'))
        result=read(out/(rev+'-transfer-result-v2.json'))
        if not n['restored'] or not n['authority']['archiveSha256']==q['archiveSha256']:raise ValueError('Loaded run unbound')
        r=dict(archive=ref(out/('red2-split-drop-fixture-'+rev+'.f3d')),step=ref(out/(rev+'-export.step')),
               matchedPiece=ref(out/'r1-load-02-coral-A.step'),loaded=ref(out/(rev+'-load-02.json')),
               solver=ref(out/(rev+'-transfer-result-v2.json')),study=ref(out/(rev+'-transfer-study.json')),
               geometryMap=ref(out/(rev+'-derivative-fixture-map.json')),
               admittedGapMm=result['cases'][0]['admittedGapMm'],execution=result['cases'][0]['executionReason'],
               positiveLoadedPairs=[b for b in n['loadedWitness']['pairs'] if max(b['volumeMm3'],b['temporaryCommonMm3'])>0],
               rejectedMeasurement=ref(out/(rev+'-load-01.json')),
               rejectionBasis='Creating the piece after commanding the sliders reset the actual fixture pose. Only its parked export remains usable.',
               scope='Private upper fixture. No field score or loaded reset.')
        r['nativePoses']=[ref(out/((rev+'-' if rev=='r2' else '')+pose+'-native.json')) for pose in ['home','present','clear','jam-left']]
        revisions.append(r)
    record=dict(authority=ref(reg),lineage=ref(ROOT/'artifacts/native-master-review/master-issue-03/lineage.json'),parkedCanonical=census,
                matchedFixtureControls=revisions,signedFieldCorners=ref(out/'r2-signed-field-screen.json'))
    (HERE/'private-evidence.json').write_text(json.dumps(record,indent=2)+'\n')
if __name__=='__main__':main()
