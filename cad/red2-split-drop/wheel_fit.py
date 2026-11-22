"""Register the authentic aluminum supplier wheel against the as-supplied 2022 module."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/split-drop-native'
def read(p): return json.loads(p.read_bytes())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def bbox(s):
    b=Bnd_Box();BRepBndLib.Add_s(s,b,False)
    return [*b.CornerMin().Coord(),*b.CornerMax().Coord()]
def cylindrical_faces(shape):
    faces=[]
    ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        face=BRepAdaptor_Surface(TopoDS.Face(ex.Current()))
        if face.GetType()==GeomAbs_Cylinder:
            cyl=face.Cylinder(); direction=cyl.Axis().Direction().Coord();xyz=cyl.Location().Coord()
            if abs(direction[0])<.01 and 1.7<cyl.Radius()<3.2:
                faces.append(dict(radius=cyl.Radius(),axis=direction,point=xyz))
        ex.Next()
    return faces

def main(assets,plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds,common,distance,transform
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if sha(ROOT/authority['file'])!=authority['sha256']:raise ValueError('Current native root changed')
    donor=assets/'swerve-package/REV-21-3005.STEP'
    wheel=assets/'2025-red2-tip-tray/REV-21-3002.STEP'
    if sha(donor)!='11f8381810846a2e52a23dbbe07f8271b314967624eee53f56e59a3ece05c75e':raise ValueError('Unknown donor')
    if sha(wheel)!='ab93934e5b5c966c9b411c8d046ff8d345fcf7ab44696903409b378dbc15c6c2':raise ValueError('Unknown wheel')
    hub=load_solids(wheel)[0]['REV-21-3002']
    identities={}
    entire,surfaces=load_solids(donor,identities)
    if len(entire)!=273 or [s['name'] for s in surfaces]!=['REV-20-3003']:raise ValueError('Unidentified donor geometry')
    deleted=[k for k in entire if 'REV-21-3004' in str(identities[k]['occurrencePath'])]
    if deleted!=['REV-21-3004-P01']:raise ValueError('Incomplete molded wheel branches')
    screws=[k for k,v in identities.items() if v['instance'] in {f'1.{i}.1' for i in range(49,55)}]
    if len(screws)!=6 or not all('Button Head Cap Screws_1.000' in k for k in screws):raise ValueError('The six wheel drive screws changed')
    phases=[]
    for angle in (0,5,10,15):
        trial=transform(hub,[dict(kind='rotation',travelDeg=angle,originMm=[0,0,0],axis=[1,0,0])],1)
        fits=[]
        for key in screws:
            gap=distance(trial,entire[key])
            fits.append(dict(screw=key,gapMm=gap,volumeMm3=common(trial,entire[key]) if gap<1e-6 else 0))
        phases.append(dict(axialRotationDeg=angle,splineOverlapMm3=common(trial,entire['REV-21-3005-P15']),screwFits=fits))
    best=min(phases,key=lambda v:(sum(x['volumeMm3'] for x in v['screwFits']),-min(x['gapMm'] for x in v['screwFits'])))['axialRotationDeg']
    trial=transform(hub,[dict(kind='rotation',travelDeg=best,originMm=[0,0,0],axis=[1,0,0])],1)
    box=bounds(trial);neighbors=[]
    for name,shape in entire.items():
        if name in deleted or box_gap(box,bbox(shape))>1.0:continue
        gap=distance(trial,shape)
        neighbors.append(dict(name=name,instance=identities[name]['instance'],gapMm=gap,
                              occupiedMm3=common(trial,shape) if gap<1e-6 else 0))
    result=dict(authority=authority,sourceWheelSha256=sha(wheel),sourceModuleSha256=sha(donor),
                sourceModuleSolids=len(entire),excludedOriginalWheel=[*deleted,*surfaces],sourceAxleUnit=[1,0,0],
                removedCoreReason='Supplier assembly retained as the 2022 donor historical source. The replaced wheel includes one open shell.',
                phaseCandidates=phases,testedPhaseDeg=best,untranslatedHubBoundsMm=box,allBroadphaseNeighbors=neighbors,
                radialHoles=cylindrical_faces(hub),rootToCorner=read(ROOT/'experiments/swerve-drive/packaging/module.json')['modules'],
                limits='Narrow-phase checks use the manufactured hub on the unchanged donor at the wheel datum. Native cold reopen, complete remapped equipment, shop-cut tread, screw installation, actual torque transmission and loaded rolling radius remain open.')
    (OUT/'aluminum-hub-fit.json').write_text(json.dumps(result,indent=2)+'\n')
    print('phases',[(p['axialRotationDeg'],sum(x['volumeMm3'] for x in p['screwFits'])) for p in phases],
          'neighbors',len(neighbors),'positive',[(n['name'],n['occupiedMm3']) for n in neighbors if n['occupiedMm3']>.01],flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assets',type=Path);parser.add_argument('plugin',type=Path)
    args=parser.parse_args();main(args.assets,args.plugin)
