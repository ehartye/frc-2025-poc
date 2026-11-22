"""Find conflicts for the finite shear deck, removable end bars and actual gusset paths on the four-corner issue."""
import json
import sys
from pathlib import Path
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Vec
from crossmember_envelope import box
from four_corner_packaging import polygon

HERE=Path(__file__).parent
ROOT=HERE.parents[3]


def shape_plan():
    lower=177.8; t=3.175; arm=115.; X=344.6;Y=339.6
    pieces={}
    web=[(arm,Y),(-arm,Y),(-arm,arm),(-X,arm),(-X,-arm),(-arm,-arm),(-arm,-Y),(arm,-Y),(arm,-arm),(X,-arm),(X,arm),(arm,arm)]
    pieces['formed web envelope']=BRepPrimAPI_MakePrism(polygon([(a,b,lower) for a,b in web]),gp_Vec(0,0,t)).Shape()
    pieces['removable central throughmember']=box((-25.4,-(Y-25.4),101.6),(25.4,Y-25.4,lower))
    for axis,extent in [(0,X),(1,Y)]:
        for sign in (-1,1):
            def xyz(inboard,tangent,z):
                radial=sign*(extent-inboard)
                return (radial,tangent,z) if axis==0 else (tangent,radial,z)
            def env(key,low,hi):
                a=xyz(*low);b=xyz(*hi)
                pieces[f'{axis}/{sign}/{key}']=box(tuple(min(x,y) for x,y in zip(a,b)),tuple(max(x,y) for x,y in zip(a,b)))
            env('torque box',(0,-arm,101.6),(25.4,arm,lower))
            for end in (-1,1):
                env(f'edge fold {end}',(25.4,end*arm,lower+t),(extent-arm,end*(arm-t),lower+t+25))
            for station in (-100.,0.,100.):
                spread=29 if axis==1 and station==0 else 9
                for flank in (-1,1):
                    tang=station+flank*spread
                    face=polygon([xyz(25.4,tang-3,114.3), xyz(25.4,tang-3,lower), xyz(25.4+80,tang-3,lower)])
                    direction=gp_Vec(0,6,0) if axis==0 else gp_Vec(6,0,0)
                    pieces[f'{axis}/{sign}/{station}/{flank}/gusset']=BRepPrimAPI_MakePrism(face,direction).Shape()
    return pieces


def run(package):
    sys.path.insert(0,str(package/'scripts'))
    from fusion_authority import load,register_path,verify,resolve
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    reg=register_path(ROOT);issue=reg.parent/'issue-003/lineage.json'
    authority=verify(ROOT,reg,issue);parts,extra=load_solids(resolve(ROOT,load(issue)['step']))
    if len(parts)!=488 or extra:raise ValueError('Expected all four installed corners and four rails')
    bb={k:bounds(v) for k,v in parts.items()};result={}
    proposals=shape_plan()
    for name,stock in proposals.items():
        measured=[];a=bounds(stock);pairs=sorted((box_gap(a,b),k) for k,b in bb.items())
        for lower,k in pairs:
            if lower>3:break
            dist=distance(stock,parts[k]);v=common(stock,parts[k]) if dist<.0001 else 0
            measured.append(dict(id=k,gapMm=dist,overlapMm3=v))
        result[name]=dict(tested=measured,remainingLowerBoundMm=next((v for v,k in pairs if v>3),None))
    return dict(authority=authority,shapes=len(proposals),originals=len(parts),testedPairs=len(proposals)*len(parts),sections=result,
        scope='Nominal parked geometry only. Tube and torque-box volumes are full occupied envelopes, sheet folds and 24 finite gussets use X/Y/Z extents in the active frame. Their own joint compatibility, weld/bend allowances, panel and module service, angular wheel motion and functional upper clearance need separate checks.')

if __name__=='__main__':
    result=run(Path(sys.argv[1]).resolve())
    (HERE/'folded-frame-clearance.json').write_text(json.dumps(result,indent=2)+'\n')
    hits=[(k,v['id'],round(v['overlapMm3'],1)) for k,r in result['sections'].items() for v in r['tested'] if v['overlapMm3']>.01]
    print(result['shapes'],result['testedPairs'],hits)
