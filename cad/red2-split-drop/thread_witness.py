"""Check a helical M3 screw's Boolean using a partition and interior witnesses."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.TopAbs import TopAbs_IN,TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from output_mount import OUT,ROOT,place,matrix,read,digest

def shell(shape):
    shells=TopExp_Explorer(shape,TopAbs_SHELL)
    result=shells.Current()
    shells.Next()
    if shells.More():raise ValueError('Multiple shells require all-boundary distance')
    return result

def main(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import common,distance
    from showcase_contacts import load_solids
    fit=read(OUT/'output-mount-fit.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert fit['authority']==authority and digest(ROOT/authority['file'])==authority['sha256']
    source=OUT/'supplier/2800-0003-0008.STEP'
    output=OUT/'supplier/REV-41-1615.STEP'
    assert digest(source)==fit['sources']['2800-0003-0008'] and digest(output)==fit['sources']['REV-41-1615']
    screw=next(iter(load_solids(source)[0].values()))
    block=place(load_solids(output)[0]['REV-41-1612'],np.array(fit['outputPose']))
    T=np.array(fit['faceScrews']['face screw 60']['transform'])
    mounted=place(screw,T)
    cs=BRepClass3d_SolidClassifier(mounted);cb=BRepClass3d_SolidClassifier(block)
    a,b=shell(mounted),shell(block)
    witnesses=[]
    # Keep the chosen axial centers half a pitch apart. Once a positive ball
    # is found for a given pitch, no other witness at this pitch contributes.
    for z in np.arange(-8.78,-4.7,.5):
        best=None
        for deg in range(0,360,30):
            angle=math.radians(deg);r=1.335
            local=[r*math.cos(angle),r*math.sin(angle),z,1.]
            world=T@local
            point=gp_Pnt(*world)
            cs.Perform(point,1e-7);cb.Perform(point,1e-7)
            if cs.State()!=TopAbs_IN or cb.State()!=TopAbs_IN:continue
            vertex=BRepBuilderAPI_MakeVertex(point).Vertex()
            ball=min(distance(vertex,a),distance(vertex,b))-.001
            if ball<.005:continue
            if best is None or ball>best['radiusMm']:
                best=dict(screwLocalMm=local[:3],worldMm=world.tolist(),radiusMm=ball)
        if best:witnesses.append(best)
    for i,v in enumerate(witnesses):
        assert all(math.dist(v['worldMm'],o['worldMm'])>v['radiusMm']+o['radiusMm'] for o in witnesses[:i])
    partition=[]
    for z,h in [(-9.,5.),(-9.02,4.91)]:
        axis=gp_Ax2(gp_Pnt(0,0,z),gp_Dir(0,0,1))
        def cyl(r):return BRepPrimAPI_MakeCylinder(axis,r,h).Shape()
        outer,inner=cyl(1.6),cyl(1.25)
        ring=BRepAlgoAPI_Cut(outer,inner).Shape()
        partition.append(dict(z=z,h=h,whole=common(screw,outer),core=common(screw,inner),ring=common(screw,ring)))
    lower=sum(4/3*math.pi*p['radiusMm']**3 for p in witnesses)
    result=dict(issue=hashlib.sha256((OUT/'output-mount-fit.json').read_bytes()).hexdigest(),
                valid=[BRepCheck_Analyzer(s).IsValid() for s in (mounted,block)],
                directCommonMm3=common(mounted,block),partitionMm3=partition,
                disjointInscribedBalls=witnesses,lowerBoundForPositiveVolumeMm3=lower,
                disposition='A direct zero violates the independently classified interior overlap. This issue requires the native reviewer and an independent geometric method.')
    (OUT/'output-thread-witness.json').write_text(json.dumps(result,indent=2)+'\n')
    print('direct',result['directCommonMm3'],'disjoint witnesses',len(witnesses),'lower bound',lower,'partitions',partition)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path);main(ap.parse_args().plugin)
