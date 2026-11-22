"""Compare exact vendor rack and pinion flanks through one repeating engagement."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(file):return json.loads(file.read_bytes())
def sha(file):return hashlib.sha256(file.read_bytes()).hexdigest()

def main(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import common,distance,transform
    spec=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if sha(ROOT/spec['file'])!=spec['sha256']:raise ValueError('Changed canonical source')
    if sha(OUT/'red2-split-drop-fixture-r2.f3d')!=read(OUT/'r2-load-02.json')['sha256']:raise ValueError('Changed upper fixture')
    parts={}
    for name,h in [('WCP-0291','2519d52681cb423a48a7f50a7b2af96a6194ce930f621d26c98c61a2558ae3d7'),('WCP-0290','27edad00569d465247adfdada71fa5600a4325953abe2456027aade27af5d06d')]:
        file=OUT/'supplier'/(name+'.step')
        if sha(file)!=h:raise ValueError('Supplier version changed')
        solids,surfaces=load_solids(file)
        if set(solids)!={name} or surfaces:raise ValueError('Incomplete supplier stock')
        parts[name]=solids[name]
    # Material is clipped only beyond the pinion's outside diameter and worst motion.
    window=BRepPrimAPI_MakeBox(gp_Pnt(-21,-10,-3),gp_Pnt(21,16,3)).Shape()
    cut=BRepAlgoAPI_Common(parts['WCP-0291'],window)
    if not cut.IsDone():raise ValueError('Cannot isolate the checked tooth group')
    rack=cut.Shape()
    def tr(shape,x,y,z):return transform(shape,[dict(kind='translation',travelMm=[x,y,z])],1)
    def rot(shape,degrees):return transform(shape,[dict(kind='rotation',travelDeg=degrees,originMm=[0,0,0],axis=[0,0,1])],1)
    r=10.16
    pitch=math.pi*25.4/20
    delta=.10
    def pair(phase,travel,extra,offset=0):
        gear=tr(rot(parts['WCP-0290'],phase+math.degrees(travel/r)),0,13.6525+r+extra,0)
        moved=tr(rack,travel+offset,0,0)
        g=distance(gear,moved)
        return dict(gapMm=g,volumeMm3=common(gear,moved) if g<1e-7 else 0)
    phases=[]
    for i in range(9):
        phase=22.5*i/8
        fit=pair(phase,0,delta)
        phases.append(dict(deg=phase,**fit))
    best=min(phases,key=lambda s:(s['volumeMm3'],-s['gapMm']))
    traces=[]
    for extra in [0,delta,.25]:
        for i in range(17):
            travel=pitch*i/16
            traces.append(dict(centerExtraMm=extra,travelMm=travel,**pair(best['deg'],travel,extra)))
    # With the pinion immobilized by one rack, push the other rack independently.
    jam=[]
    for i in range(9):
        travel=pitch*i/8
        jam.append(dict(unilateralTravelMm=travel,**pair(best['deg'],0,delta,travel)))
    report=dict(authority=spec['sha256'],fixture=read(OUT/'r2-load-02.json')['sha256'],
                rackSha256=sha(OUT/'supplier/WCP-0291.step'),pinionSha256=sha(OUT/'supplier/WCP-0290.step'),
                pitchMm=pitch,pitchRadiusMm=r,rackPitchLineLocalYmm=13.6525,phaseSamples=phases,
                centerExtraMm=delta,chosenPhaseDeg=best['deg'],meshSamples=traces,oneSideJammed=jam,
                scope='Exact supplier BReps on a mesh fixture. Finite samples and ideal tooth synchronization. Deflection, tooth strength, bracket and mounting compliance, wear, continuous native closure and loaded robot remain open.')
    (OUT/'rack-pinion-fits.json').write_text(json.dumps(report,indent=2))
    print('phase',best,'mesh maximum',max(x['volumeMm3'] for x in traces),'one-side jam',max(x['volumeMm3'] for x in jam))

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('plugin',type=Path)
    main(parser.parse_args().plugin)
