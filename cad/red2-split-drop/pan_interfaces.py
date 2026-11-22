"""Check each purchased pan solid against newly fabricated finite stock in its planned pose."""
import sys
import json
from pathlib import Path
import numpy as np
from pan_mechanics import stock,rotation
from output_mount import ROOT,read,place,digest
D=ROOT/'artifacts/red2-integrated'
def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,common
    plan,drive=[read(D/n) for n in ('pan-shop-plan.json','pan-drive-registration.json')]
    assert digest(ROOT/plan['authority']['file'])==plan['authority']['sha256']
    assert digest(D/'red2-integrated-foundation-r2.f3d')==plan['foundationSha256']
    shift=np.array([[1,0,0,205],[0,0,1,216],[0,-1,0,580],[0,0,0,1]])
    fabricated={p['id']:(stock(p),p['moving']) for p in plan['items']}
    purchase={};cache={}
    for row in drive['components']:
        file=ROOT/row['source']
        if digest(file)!=row['sourceSha256']:raise ValueError(row['id'])
        if file not in cache:cache[file]=load_solids(file)[0]
        pose=np.eye(4);pose[:3]=row['frameMm']
        for rid,shape in cache[file].items():
            mobile=row['id']=='REV-41-1620' or row['id'].startswith('M3x8 adapter') or row.get('moving',False)
            purchase[(row['id'],rid)]=(place(shape,(shift@pose)[:3]),mobile)
    poses={};processed=set()
    for phase,angle in [('zero',0),('stow',70),('transit',34),('dump',-2)]:
        parts={name:place(sh,rotation(angle)) if move else sh for name,(sh,move) in fabricated.items()}
        boxes={name:bounds(s) for name,s in parts.items()};seen=[];near=0
        for (sku,rid),(raw,mobile) in purchase.items():
            s=place(raw,rotation(angle)) if mobile else raw;box=bounds(s)
            for name,other in parts.items():
                if ((sku,rid,name) in processed and mobile==fabricated[name][1]) or box_gap(box,boxes[name])>.001:continue
                near+=1;v=common(s,other)
                if v>.01:seen.append(dict(purchase=sku,solid=rid,stock=name,positiveVolumeMm3=v))
                processed.add((sku,rid,name))
        poses[phase]=dict(checkedCandidatePairs=near,positive=seen)
        (D/'pan-purchase-stock.json').write_text(json.dumps(dict(authority=plan['authority'],foundation=plan['foundationSha256'],stockPlan=digest(D/'pan-shop-plan.json'),drivePlan=digest(D/'pan-drive-registration.json'),purchaseSolids=len(purchase),shopSolids=len(fabricated),poses=poses),indent=2)+'\n')
        print(phase,near,seen,flush=True)
if __name__=='__main__':run(Path(sys.argv[1]))
