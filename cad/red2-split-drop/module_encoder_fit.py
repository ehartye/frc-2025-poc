"""Resolve the original through-bore sensor's actual source solids against the four-peg bridge."""
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepTools import BRepTools
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape

from output_mount import ROOT, digest, place, read


def study(plugin, donor, p26_file):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common, distance
    from showcase_contacts import box_gap, load_solids
    folder=ROOT/'artifacts/red2-integrated/module-option'
    path=folder/'measured-v11-top-stack.json'
    previous=read(path)
    if digest(ROOT/previous['authority']['file'])!=previous['authority']['sha256']:
        raise ValueError('Changed assembly authority')
    if digest(p26_file)!=previous['sources']['P26']:
        raise ValueError('Changed four-peg bridge')
    (bridge,)=load_solids(p26_file)[0].values()
    bridge=place(bridge,np.array(previous['placements']['P26'])[:3])
    def read_brep(key):
        f=donor/(key+'.brep')
        assert digest(f)==previous['sources'][key]
        geom=TopoDS_Shape()
        assert BRepTools.Read_s(geom,str(f),BRep_Builder())
        return geom
    original=read_brep('28-REV-21-3005-P13')
    encoder=read_brep('24-REV-11-1271')
    bb=bounds(bridge)
    def measure(target,body):
        if box_gap(bounds(target),bounds(body))>.00001:
            return 0.
        return common(target,body)
    def axial_flats(body):
        t=TopExp_Explorer(body,TopAbs_FACE);rows=[]
        while t.More():
            face=TopoDS.Face(t.Current());t.Next()
            surf=BRepAdaptor_Surface(face)
            if surf.GetType()!=GeomAbs_Plane:continue
            p=surf.Plane();n=np.array(p.Axis().Direction().Coord())
            if abs(n[1])>.0001:continue
            offset=n@p.Location().Coord()
            if 5.9 < abs(offset) < 6.8:
                rows.append(dict(radiusMm=abs(offset),normalAngleDeg=math.degrees(math.atan2(n[0], n[2])),axialExtentMm=[bounds(face)[1],bounds(face)[4]]))
        return rows
    e=TopExp_Explorer(encoder,TopAbs_SOLID);rows=[]
    targets={}
    installed={}
    while e.More():
        body=TopoDS.Solid(e.Current());e.Next();num=len(rows)+1
        installed[num]=body
        old=measure(original,body);new=measure(bridge,body)
        rows.append(dict(sourceOrder=num,boundsMm=bounds(body),oldP13CommonMm3=old,newP26CommonMm3=new))
        if max(new,old)>.001:
            targets[num]=body
            rows[-1]['axialFlats']=axial_flats(body)
    result=dict(sourceSha256=previous['sources']['24-REV-11-1271'],topStackSha256=digest(path),bridgeSha256=digest(p26_file),
        bridgeBoundsMm=bb,bridgeAxialFlats=axial_flats(bridge),constituents=rows)
    if len(targets)!=1:
        raise ValueError('Need unique interfering constituent before rotating any part')
    (index,held),=targets.items()
    rotor_flats=rows[index-1]['axialFlats']
    angles=lambda flats: sorted(set(round(f['normalAngleDeg']%60,8) for f in flats))
    old_flat,=angles(rotor_flats)
    drive_flat,=angles(result['bridgeAxialFlats'])
    phase=((drive_flat-old_flat+30)%60)-30
    def rotated(deg):
        c=math.cos(math.radians(deg));s=math.sin(math.radians(deg))
        return place(held,np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0]]))
    result['registeredRotor']=dict(sourceSolid=index,drivenPhaseDeg=phase,expectedFlatGapMm=rotor_flats[0]['radiusMm']-result['bridgeAxialFlats'][0]['radiusMm'],controls={})
    trial=result['registeredRotor']
    for name,angle in [('source',0),('aligned',phase),('minus-2deg',phase-2),('plus-2deg',phase+2)]:
        rotor=rotated(angle)
        trial['controls'][name]=dict(angleDeg=angle,P26CommonMm3=common(rotor,bridge),P26gapMm=distance(rotor,bridge),fixedOtherEncoderSolids=len(installed)-1)
        if name in ('aligned','source'):
            trial['controls'][name]['otherEncoderCommonMm3']={n:measure(rotor,v) for n,v in installed.items() if n!=index}
    def regions(a,b):
        cut=BRepAlgoAPI_Common(a,b)
        cut.Build()
        if not cut.IsDone():raise ValueError('Unable to resolve the contact region')
        items=[]
        it=TopExp_Explorer(cut.Shape(),TopAbs_SOLID)
        while it.More():
            body=TopoDS.Solid(it.Current());it.Next()
            props=GProp_GProps();BRepGProp.VolumeProperties_s(body,props)
            items.append(dict(volumeMm3=props.Mass(),centerMm=props.CentreOfMass().Coord(),boundsMm=bounds(body)))
        return items
    trial['alignedP26ContactRegions']=regions(rotated(phase),bridge)
    trial['alignedOriginalP13ContactRegions']=regions(rotated(phase),original)
    print('contact regions',trial['alignedP26ContactRegions'],flush=True)
    trial['description']='The 12 native planar bore faces in this single constituent form six radial normals. Rotate only the rotor relative to the 28 original stationary encoder constituents. Capture an installed wheel index after assembly.'
    print('rotor',index,phase,{k:(v['P26CommonMm3'],v['P26gapMm']) for k,v in trial['controls'].items()},flush=True)
    (folder/'original-1271-bridge-constituents.json').write_text(json.dumps(result,indent=2)+'\n')
    print('changed',[(r['sourceOrder'],r['newP26CommonMm3'],r['oldP13CommonMm3'],r.get('axialFlats')) for r in rows if max(r['newP26CommonMm3'],r['oldP13CommonMm3'])>.001])
    print('bridge flats',result['bridgeAxialFlats'])

if __name__=='__main__':
    study(*[Path(s) for s in sys.argv[1:]])
