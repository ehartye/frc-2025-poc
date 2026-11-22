"""Register the actual five-hole output block, adapter and three face screws."""
import argparse
import hashlib
import json
import math
import sys
from functools import lru_cache
from pathlib import Path

import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'artifacts/split-drop-native'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def read(path):
    return json.loads(path.read_bytes())

def matrix(angle, xyz):
    a = math.radians(angle)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,s,0,xyz[0]], [s,-c,0,xyz[1]], [0,0,-1,xyz[2]]])

def place(shape, values):
    transform = gp_Trsf()
    transform.SetValues(*values.flatten())
    return BRepBuilderAPI_Transform(shape,transform,False).Shape()

def main(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids, box_gap
    from mechanical_motion import bounds, common, distance

    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    native=read(OUT/'r4-load-01.json')
    assert digest(ROOT/authority['file'])==authority['sha256']==native['authority']['archiveSha256']
    assert digest(OUT/'red2-split-drop-fixture-r4.f3d')==native['sha256']
    assert digest(OUT/'r4-export.step')==native['step']['sha256']
    files={sku:OUT/'supplier'/(sku+'.STEP') for sku in ['REV-41-1615','REV-41-1620','2800-0003-0008']}
    catalog={sku:load_solids(path)[0] for sku,path in files.items()}
    identities={}
    fixture,_=load_solids(OUT/'r4-export.step',identities)
    # The adapter's flange on the issued fixture is fixed at z=521. The
    # REV-41-1612 block in the output stage has its mating face at native z=-6.
    output_pose=matrix(-18.75,[-147.,0.,515.])
    body={sku:place(shape,output_pose) for sku,shape in catalog['REV-41-1615'].items()}
    cached_bounds=lru_cache(maxsize=None)(bounds)
    def rows(shape,near):
        result=[]
        for key,other in near.items():
            if box_gap(cached_bounds(shape),cached_bounds(other))>2:continue
            gap=distance(shape,other)
            result.append(dict(component=key,gapMm=gap,overlapMm3=common(shape,other) if gap<.00001 else 0.))
        return result
    adapter=next(shape for key,shape in fixture.items() if 'REV-41-1620' in identities[key]['occurrencePath'])
    gearbox_fit={key:rows(shape,fixture) for key,shape in body.items()}
    # The block has five complete tapped 2.5 mm minor bores on a 16 mm PCD.
    # The sixth is absent at the lateral through-screw access. The 60/180/300
    # positions form an even three-screw pattern on complete tapped faces.
    screw0=catalog['2800-0003-0008']['2800-0003-0008']
    screws={};access={}
    for angle in (60,180,300):
        a=math.radians(angle)
        xy=output_pose@np.array([8*math.cos(a),8*math.sin(a),-6,1])
        # Source head bearing plane z=-12; installed +Z is outward.
        position=matrix(0,[*xy[:2],512.])
        fastener=place(screw0,position)
        key=f'face screw {angle}'
        screws[key]=dict(outputAngleDeg=angle,transform=position.tolist(),
            mates=rows(fastener,{**body,'already mounted adapter':adapter}),fixture=rows(fastener,fixture))
        # Circumscribed 2.5 mm hex-driver shaft over the full 65 mm entry.
        # The candidate tool is a cylindrical clearance gauge only.
        axis=gp_Ax2(gp_Pnt(*xy[:2],525.2),gp_Dir(0,0,1))
        gauge=BRepPrimAPI_MakeCylinder(axis,2.5/math.sqrt(3),65).Shape()
        access[key]=rows(gauge,{**fixture,**body})
    result=dict(authority=authority,archive=native['sha256'],derivative=native['step']['sha256'],
        sources={sku:digest(path) for sku,path in files.items()},outputPose=output_pose.tolist(),
        outputStageFixtureNeighbors=gearbox_fit,faceScrews=screws,axialToolGauge=access,
        limits='Stationary home-pose supplier fit only. Native integration, torque, output bearing side load, bracket, cable and gear retention, and the actual service tool remain open.')
    (OUT/'output-mount-fit.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,group in [('output',gearbox_fit),('screw',{k:v['mates'] for k,v in screws.items()}),('key',access)]:
        print(name,[(k,[(r['component'],round(r['overlapMm3'],3)) for r in rs if r['overlapMm3']>.01]) for k,rs in group.items() if any(r['overlapMm3']>.01 for r in rs)])

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('plugin',type=Path)
    main(parser.parse_args().plugin)
