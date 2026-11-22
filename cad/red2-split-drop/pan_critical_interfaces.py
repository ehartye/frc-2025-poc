"""Challenge R2's actual bearings, loose idler and supplier output spline in the issued STEP."""
import json
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Vec

from output_mount import ROOT, digest, place, read
from pan_mechanics import rotation


def study(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common, distance
    from showcase_contacts import box_gap, load_solids
    directory=ROOT/'artifacts/red2-integrated'
    station=directory/'pan-native'
    r=read(station/'pan-r2-foundation-install-02.json');archive=station/'integrated-340mm-pan-r2.f3d';step=archive.with_suffix('.step')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if digest(ROOT/authority['file'])!=r['authority']['archiveSha256'] or digest(archive)!=r[archive.stem] or digest(step)!=r['derivativeSha256']:
        raise ValueError('Changed recorded source or derivative')
    metadata={};bodies,other=load_solids(step,metadata)
    if len(bodies)!=294 or other:raise ValueError('Whole current scope required')
    boxes={k:bounds(v) for k,v in bodies.items()}
    def roster(label):
        return [k for k in bodies if label in metadata[k]['occurrencePath'] or label+':1' in metadata[k]['occurrencePath'] or k==label]
    def one(label):
        matches=roster(label);assert len(matches)==1,(label,matches);return matches[0]
    def vol(s):
        props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props);return props.Mass()
    def pair(a,b):
        return dict(gapMm=distance(a,b),commonMm3=common(a,b))
    proof=dict(authority=authority,archiveSha256=digest(archive),stepSha256=digest(step),installedBodies=len(bodies))
    sides={}
    stub=one('Idler headed stub hex shank')
    for name,word,pin in [('drive','Port',one('REV-41-1620 pan purchase')),('idler','Starboard',stub)]:
        bearing=one('REV-21-1915 '+name+' pan purchase');tower=one(word+' bearing tower')
        sides[name]={}
        for state in r['cold']['poses']:
            diff=state['angleDeg']-70
            moved=lambda k:place(bodies[k],rotation(diff)) if diff else bodies[k]
            sides[name][state['pose']]=dict(bearingPath=metadata[bearing]['occurrencePath'],shaftPath=metadata[pin]['occurrencePath'],
                towerPath=metadata[tower]['occurrencePath'],shaft=pair(moved(bearing),moved(pin)),tower=pair(moved(bearing),bodies[tower]))
    proof['bearingAtMeasuredPoses']=sides
    # Construct the exact union swept by this straight constant hexagonal blank
    # in both axial directions. The verified independent stock is a single prism.
    sb=boxes[stub];span=sb[4]-sb[1];it=TopExp_Explorer(bodies[stub],TopAbs_FACE);end=[]
    while it.More():
        face=(getattr(TopoDS,'Face_s',None) or TopoDS.Face)(it.Current());it.Next()
        fs=BRepAdaptor_Surface(face);b=bounds(face)
        if fs.GetType()==GeomAbs_Plane and abs(b[1]-sb[1])<.001 and abs(b[4]-sb[1])<.001:end.append(face)
    (origin,)=end
    travel=3.
    frame=np.eye(4);frame[1,3]=-travel
    sweep=BRepPrimAPI_MakePrism(place(origin,frame[:3]),gp_Vec(0,span+2*travel,0)).Shape()
    expected=vol(bodies[stub])*(span+2*travel)/span
    if abs(vol(sweep)-expected)>.002:raise ValueError('Axial envelope volume mismatch')
    b=bounds(sweep);roster_axial={}
    for name,s in bodies.items():
        if name==stub or box_gap(b,boxes[name])>1e-6:continue
        roster_axial[name]=dict(path=metadata[name]['occurrencePath'],**pair(sweep,s))
    proof['idlerAxialFreedom']=dict(sweepMm=[-travel,travel],sourceLengthMm=span,sweptVolumeMm3=vol(sweep),
        fullNeighborRoster=len(bodies)-1,broadPhase=roster_axial,maximumCommonMm3=max(x['commonMm3'] for x in roster_axial.values()))
    record=station/'critical-interfaces-r2.json'
    record.write_text(json.dumps(proof,indent=2))
    # Use the source constituent's actual axial interval and volume to identify
    # the unlabelled neutral carrier in the cold assembly, independent of group order.
    drive=read(directory/'pan-drive-registration.json')
    male_spec=next(row for row in drive['components'] if row['id']=='REV-41-1601')
    male_src=ROOT/male_spec['source'];assert digest(male_src)==male_spec['sourceSha256']
    ns=load_solids(male_src)[0]
    male,=[(k,placed) for k,s in ns.items() if abs(bounds(placed:=place(s,np.array(male_spec['frameMm'])))[2]-4.375)<.005]
    female_src=ROOT/'artifacts/split-drop-native/supplier/REV-41-1615.STEP'
    female=('REV-41-1611_Through Hole',load_solids(female_src)[0]['REV-41-1611_Through Hole'])
    def identify(article,source):
        zb=bounds(source)
        pool=roster(article+' pan purchase')
        hits=[k for k in pool if abs(boxes[k][1]-216-zb[2])<.015 and abs(boxes[k][4]-216-zb[5])<.015
                and abs(vol(bodies[k])-vol(source))<max(.04,vol(source)*.001)]
        if len(hits)!=1:raise ValueError('Cannot identify purchased spline from actual dimensions '+str((article,hits,zb,[(k,boxes[k][1],boxes[k][4],vol(bodies[k])) for k in pool])))
        return hits[0]
    carrier=identify('REV-41-1601',male[1]);hub=identify('REV-41-1615 rotating',female[1])
    poses={}
    for state in r['cold']['poses']:
        h=place(bodies[hub],rotation(state['angleDeg']-70))
        poses[state['pose']]=pair(bodies[carrier],h)
    neutral=place(bodies[hub],rotation(-70));poses['flat-source-control']=pair(bodies[carrier],neutral)
    proof['spline']=dict(male=metadata[carrier]['occurrencePath'],female=metadata[hub]['occurrencePath'],poses=poses,
                         basis='As-issued cartridge housing and male carrier are locked together in the neutral assembly; gear-train torque motion is unresolved. These are frozen CAD poses.')
    record.write_text(json.dumps(proof,indent=2))
    print(proof['idlerAxialFreedom'],poses)


if __name__=='__main__':study(Path(sys.argv[1]))
