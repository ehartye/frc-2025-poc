"""Recheck the four landed shaft articles on the 301-solid STEP minted from the cold native issue."""
import json
import sys
from pathlib import Path
from output_mount import ROOT,digest,read
from pan_race_patches import patches
from pan_thrust_layout import translate


def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from component_shape import compare_components
    from mechanical_motion import common,distance
    from showcase_contacts import load_solids
    p=ROOT/'artifacts/red2-integrated/pan-thrust'
    out=p/'moving-stack-issued';step=out/'checked-301.step';a=out/'checked-301.f3d'
    result=read(out/'r3-moving-proxy-short-01.json');plan=read(p/'moving-stack-install-plan.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/authority['file'])==result['authority']['archiveSha256'] and digest(step)==result['stepSha256'] and digest(a)==result['archiveSha256']
    solids,extra=load_solids(step); assert len(solids)==301 and not extra
    jobs=[];cases={}
    for name in plan['order']:
        row=plan['placed'][name]
        file=ROOT/row['source']['file']; assert digest(file)==row['source']['sha256']
        labels=load_solids(file)[0];key,=labels.keys()
        matrix=[x for r in row['actualTargetWorldMatrixMm'] for x in r]+[0,0,0,1]
        jobs.append((name,file,key,matrix))
        piece=solids[name];mate=solids[row['mate']]
        probes={}
        for dy in (0.,.05,-.05):
            moved=translate(piece,[0,dy,0])
            faces=patches(moved,mate)
            probes[str(dy)]=dict(distanceMm=distance(moved,mate),volumeMm3=common(moved,mate),areaMm2=sum(v['areaMm2'] for v in faces),faces=faces)
        if probes['0.0']['areaMm2']<1 or probes['0.0']['volumeMm3']>.001 or probes['0.0']['distanceMm']>.001:
            raise ValueError('Expected planar contact lost '+name)
        cases[name]=dict(mate=row['mate'],sourceFile=row['source'],probes=probes)
    diff=compare_components(step,jobs)
    output={'authority':authority,'privateCandidate':result['archiveSha256'],'stepSha256':digest(step),'nativeReceipt':digest(out/'r3-moving-proxy-short-01.json'),
            'installedArticleGeometry':diff,'trueMatingFacesAndPerturbations':cases}
    (out/'four-seats-and-shapes.json').write_text(json.dumps(output,indent=2)+'\n')
    print([(n,pro['probes']['0.0']['areaMm2'],diff[n]) for n,pro in cases.items()],flush=True)


if __name__=='__main__':run(Path(sys.argv[1]))
