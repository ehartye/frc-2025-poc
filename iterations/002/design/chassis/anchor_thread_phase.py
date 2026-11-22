"""Rotate genuine male solids about their seated axes and measure both thread profiles on the cold 504."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Ax1,gp_Dir,gp_Lin,gp_Pnt,gp_Trsf
from OCP.TopLoc import TopLoc_Location

HERE=Path(__file__).parent
ROOT=HERE.parents[3]


def vertical_ray(s,cls,origin,normal,p,q):
    axis=gp_Lin(gp_Pnt(*origin),gp_Dir(*normal))
    s.Perform(axis,p-.0002,q+.0002)
    if not s.IsDone():raise ValueError('Ray failed')
    hits=sorted(set(round(float(s.WParameter(i)),9) for i in range(1,s.NbPnt()+1)))
    limits=[p]+[a for a in hits if p+.000001<a<q-.000001]+[q]
    filled=[]
    for l,r in zip(limits,limits[1:]):
        if r-l<1e-7:continue
        cls.Perform(gp_Pnt(*(origin+(l+r)/2*normal)),1.e-7)
        if cls.State()==TopAbs_IN:filled.append((l,r))
    return filled


def overlaps(a,b):
    return [(max(l,x),min(r,y)) for l,r in a for x,y in b if min(r,y)>max(l,x)]


def trial(components,receiver,male,origin,axis,low,high,p,q):
    from mechanical_motion import common
    from interface_consistency import inscribed_balls
    origin=np.array(origin);normal=np.array(axis)
    u=np.array([1.,0,0]);v=np.cross(normal,u)
    radii=np.linspace(low,high,7);dr=radii[1]-radii[0]
    count=36;A=[];B=[]
    engines=[]
    for solid in (components[male],components[receiver]):
        engine=IntCurvesFace_ShapeIntersector();engine.Load(solid,1.e-7)
        engines.append((engine,BRepClass3d_SolidClassifier(solid)))
    for r in radii:
        records=[[],[]]
        for k in range(count):
            theta=math.tau*k/count
            at=origin+r*(math.cos(theta)*u+math.sin(theta)*v)
            for (engine,cls),dst in zip(engines,records):
                dst.append(vertical_ray(engine,cls,at,normal,p,q))
        A.append(records[0]);B.append(records[1])
    volumes=[]
    for offset in range(count):
        total=0.
        for j,r in enumerate(radii):
            for k in range(count):
                cov=overlaps(A[j][(k-offset)%count],B[j][k])
                total+=sum(b-a for a,b in cov)*r*math.tau/count*dr
        volumes.append(total)
    low_index=int(np.argmin(volumes));high_index=int(np.argmax(volumes))
    exact=[]
    for offset in sorted(set([0,low_index,high_index])):
        theta=offset*math.tau/count
        t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(*origin),gp_Dir(*normal)),theta);shape=components[male].Moved(TopLoc_Location(t))
        probes=[]
        for j,r in enumerate(radii):
            for k in range(count):
                angle=k*math.tau/count
                at=origin+r*(math.cos(angle)*u+math.sin(angle)*v)
                for lo,hi in overlaps(A[j][(k-offset)%count],B[j][k]):
                    if hi-lo>.14:probes.append((hi-lo,(at+normal*(lo+hi)/2).tolist()))
        probes=sorted(probes,reverse=True)[::max(1,len(probes)//18)][:18]
        balls,interior=inscribed_balls(shape,components[receiver],[b for _,b in probes],.00001) if probes else ([],0)
        exact.append(dict(clockDeg=offset*360/count,approxOverlapMm3=volumes[offset],
                          directCommonMm3=common(shape,components[receiver]),interiorWitnessMm3=interior,balls=balls))
    return dict(male=male,receiver=receiver,headOriginMm=origin.tolist(),direction=axis,measuredThreadSegmentMm=[p,q],
        radiiMm=radii.tolist(),rotationStepDeg=360/count,angularQuadratureVolumesMm3=volumes,extrema=[low_index*360/count,high_index*360/count],exact=exact,
        sampledAxialBandsAtMiddleRadius={k:{'male':A[3][i],'female':B[3][i]} for i,k in ((0,'0deg'),(9,'90deg'),(18,'180deg'),(27,'270deg'))})


def run(pkg,folder):
    sys.path.insert(0,str(pkg/'scripts'))
    from fusion_authority import register_path,verify,digest
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    reg=register_path(ROOT);auth=verify(ROOT,reg,reg.parent/'issue-003/lineage.json')
    cold=json.loads((folder/'report.json').read_bytes())
    step=folder/'port-504-negative.step'
    if cold['authority']!=auth or digest(step)!=cold['diagnosticStepSha256']:raise ValueError('Unregistered diagnostic')
    solids,sheets=load_solids(step)
    if len(solids)!=504 or sheets:raise ValueError('Incomplete issued shape')
    bounds_by={k:np.array(bounds(v)) for k,v in solids.items()}
    def match(r):
        rows=sorted((float(np.max(abs(v-r['boundsMm']))),k) for k,v in bounds_by.items())
        if rows[0][0]>.1 or rows[1][0]<.5:raise ValueError('No unique placed material: '+str(rows[:2]))
        return rows[0][1]
    rr=cold['anchor'];recv=match(next(r for r in rr if 'H1150 captive' in r['path']))
    a=match(next(r for r in rr if '91251A542' in r['path'] and r['boundsMm'][0]>55))
    b=match(next(r for r in rr if '97763A232' in r['path'] and r['boundsMm'][0]>40))
    out=dict(authority=auth,diagnosticArchiveSha256=cold['archiveSha256'],stepSha256=digest(step),coldManifestSha256=digest(folder/'report.json'),tests={})
    for name,args in [('quarter', (a,(65.,328.9375,107.95),[0.,0,-1.],2.56,3.16,3.90,25.40)),
                      ('keeper',(b,(50.,318.275,94.63336287672979-3.175-7.1),[0.,1.,0.],2.06,2.40,3.175,12.70))]:
        d=trial(solids,recv,*args);out['tests'][name]=d
        (HERE/'anchor-thread-phase.json').write_text(json.dumps(out,indent=2)+'\n')
        print(name,d['extrema'],[(r['clockDeg'],r['directCommonMm3'],r['interiorWitnessMm3']) for r in d['exact']],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('pkg',type=Path);p.add_argument('issue',type=Path)
    a=p.parse_args();run(a.pkg.resolve(),a.issue.resolve())
