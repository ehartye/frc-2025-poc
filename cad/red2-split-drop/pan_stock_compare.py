"""Compare every cold shop body to its independent, full, dimensioned cut stock."""
import json,sys
from pathlib import Path
import numpy as np
from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from pan_mechanics import stock,rotation
from output_mount import ROOT,digest,read
D=ROOT/'artifacts/red2-integrated'
def vol(shape):
    prop=GProp_GProps();BRepGProp.VolumeProperties_s(shape,prop);return prop.Mass()
def run(plugin,step,report):
    sys.path.insert(0,str(plugin/'scripts'))
    from component_shape import compare_components
    from showcase_contacts import load_solids
    from mechanical_motion import common
    plan=read(D/'pan-shop-plan.json');assert digest(ROOT/plan['authority']['file'])==plan['authority']['sha256']
    ident={};items=load_solids(step,ident)[0];parts={v['id']:v for v in plan['items']}
    indexed={name:[] for name in parts}
    for k in items:
        path=ident[k]['occurrencePath']
        name=[n for n in parts if n in path or n+':1' in path]
        if len(name)>1:raise ValueError('Ambiguous stock path '+str(path))
        if name:indexed[name[0]].append(k)
    src=D/'pan-native/independent-stock';src.mkdir(exist_ok=True)
    jobs=[];rows={}
    for j,(n,part) in enumerate(parts.items(),1):
        official=stock(part);f=src/(str(j)+'.step')
        if not f.exists():
            w=STEPControl_Writer();w.Transfer(official,STEPControl_AsIs);assert w.Write(str(f))==IFSelect_RetDone
        collection=load_solids(f)[0]
        if len(collection)!=1:raise ValueError('Independent stock is not one solid '+n)
        label,genuine=next(iter(collection.items()))
        if abs(vol(genuine)-vol(official))>.002:raise ValueError('Independent geometry changed '+n)
        matrix=np.eye(4)
        if part['moving']:matrix[:3]=rotation(70)
        row=rows[n]=dict(solidNames=indexed[n],independentSourceSha256=digest(f),sourceVolumeMm3=vol(official),expectedRigidTransformMm=matrix.ravel().tolist())
        for k in indexed[n]:jobs.append((k,f,label,matrix.ravel().tolist()))
    differences=compare_components(step,jobs)
    for n,row in rows.items():
        names=row['solidNames']
        row['nativePieceCount']=len(names)
        row['perPieceDifferenceMm3']={k:differences[k] for k in names}
        row['sumActualVolumeMm3']=sum(vol(items[k]) for k in names)
        row['overlapBetweenPiecesMm3']=sum(common(items[k],items[h]) for i,k in enumerate(names) for h in names[i+1:])
        row['absentFromPhysicalUnionMm3']=row['sourceVolumeMm3']-row['sumActualVolumeMm3']+sum(differences[k]['extraMm3'] for k in names)+row['overlapBetweenPiecesMm3']
        row['pass']=len(names)==1 and all(q<.025 for q in differences[names[0]].values())
    rec=dict(authority=plan['authority'],foundation=plan['foundationSha256'],stepSha256=digest(step),pluginFileSha256=digest(plugin/'scripts/component_shape.py'),pluginFunction='component_shape.compare_components',
            shopPlanSha256=digest(D/'pan-shop-plan.json'),allStock=rows,failed=[k for k,v in rows.items() if not v['pass']])
    report.write_text(json.dumps(rec,indent=2)+'\n')
    print('Checked',len(rows),'expected stocks; failed:',[(k,rows[k]['nativePieceCount'],rows[k]['absentFromPhysicalUnionMm3']) for k in rec['failed']],flush=True)
if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]),Path(sys.argv[3]))
