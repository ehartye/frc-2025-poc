"""Measure the keyed NEO V1.1 spacer, pinion, short key and bridge in the supplier assembly frame."""
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Shape

from output_mount import ROOT, digest, place, read


def study(plugin,shared,donor):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common, distance
    from showcase_contacts import load_solids
    folder=ROOT/'artifacts/red2-integrated/module-option'
    source=read(folder/'source-datums.json')
    assert digest(ROOT/source['authority']['file'])==source['authority']['sha256']
    assert digest(shared/'manifest.json')==source['manifestSha256']
    parts={};hashes={}
    def load(name,file):
        if file.suffix=='.brep':
            shape=TopoDS_Shape()
            assert BRepTools.Read_s(shape,str(file),BRep_Builder())
        else:
            (shape,)=load_solids(file)[0].values()
        parts[name]=shape
        hashes[name]=digest(file)
        if name in source['parts']:assert hashes[name]==source['parts'][name]['sourceSha256']
    for name in ['0-REV-21-1650','21-REV-21-3005-P09','22-REV-21-3005-P10','24-REV-11-1271','28-REV-21-3005-P13','12-REV-21-3005-P06','17-REV-21-3005-P04','18-REV-21-3005-P05','19-REV-21-3005-P08','27-REV-21-3005-P12']:
        load(name,donor/'vendor-native'/(name+'.brep'))
    for name in ['P24','P25','P26']:
        load(name,shared/'v1-1-detail'/(name+'.step'))
    def shift(x,y,z):
        t=np.eye(4);t[:3,3]=[x,y,z];return t
    def around_y(deg):
        a=math.radians(deg);c=math.cos(a);s=math.sin(a)
        return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])
    def get(name,m):
        return place(parts[name],m[:3])
    def pair(a,b):
        return dict(gapMm=distance(a,b),commonMm3=common(a,b))
    motor=parts['0-REV-21-1650'];pin=parts['21-REV-21-3005-P09']
    encoder=parts['24-REV-11-1271'];bridge=parts['28-REV-21-3005-P13']
    motorfaces=source['parts']['0-REV-21-1650']['faces']
    plane=next(f for f in motorfaces if f['kind']=='plane' and 47.13<f['areaMm2']<47.15)
    direction=plane['direction'];assert abs(direction[1])<1e-8
    key_clock=math.degrees(math.atan2(direction[0],direction[2]))
    bottom=float(np.dot(plane['centerMm'],direction))
    shaft=max(f['boundsMm'][4] for f in motorfaces if f['kind']=='cylinder' and abs(f['radiusMm']-4)<1e-5 and abs(f['originMm'][0])<1e-5)
    span=bounds(parts['P24']);spacer_len=span[5]-span[2];tip=shaft-spacer_len
    bridge_plane=source['parts']['28-REV-21-3005-P13']['faces'][0]['centerMm'][1]
    align=np.array([[-1,0,0,0],[0,0,1,0],[0,1,0,0],[0,0,0,1.]])
    matrices=dict(P24=shift(0,(tip+shaft)/2,0)@align, P26=shift(0,bridge_plane,0)@align,
        P25=around_y(key_clock)@shift(0,tip-3.5,bottom+1),P09=around_y(key_clock))
    placed={n:get(n,m) for n,m in matrices.items() if n!='P09'}
    placed['P09']=get('21-REV-21-3005-P09',matrices['P09'])
    pairs={}
    def measure(name,a,b):
        pairs[name]=pair(a,b)
    for a,b in [('P24','P26'),('P24','P25'),('P24','P09'),('P25','P09'),('P26','P09')]:
        measure(a+' to '+b,placed[a],placed[b])
    for n in ['P24','P25','P26','P09']:
        measure(n+' to motor',placed[n],motor)
    measure('P26 to encoder',placed['P26'],encoder)
    for label,part in parts.items():
        if label.startswith(('12-','17-','18-','19-','27-')):
            measure('P26 to '+label,placed['P26'],part)
            if label.startswith(('12-','18-','19-')):
                measure('clocked P09 to '+label,placed['P09'],part)
                measure('old P09 to '+label,pin,part)
    controls={}
    controls['P25 legacy slot vs motor']=pair(get('P25',shift(0,tip-3.5,bottom+1)),motor)
    controls['P25 real motor slot vs unclocked pinion']=pair(placed['P25'],pin)
    controls['P24 extra 0.5 mm into motor shoulder']=pair(get('P24',shift(0,.5,0)@matrices['P24']),motor)
    controls['P25 pushed 0.75 mm under spacer']=pair(get('P25',shift(0,.75,0)@matrices['P25']),placed['P24'])
    controls['original P10 to original motor']=pair(parts['22-REV-21-3005-P10'],motor)
    # Bind the four feet of the new and old bridges separately rather than trusting their boxes.
    tips=lambda name,axis,pos:[np.array(f['centerMm']) for f in source['parts'][name]['faces'] if f['kind']=='plane'
        and abs(f['boundsMm'][axis]-pos)<1e-5 and abs(f['boundsMm'][axis+3]-pos)<1e-5]
    old=tips('28-REV-21-3005-P13',1,bridge_plane-5)
    new=[(matrices['P26']@np.r_[p,1.])[:3] for p in tips('P26',2,-5)]
    if len(old)!=4 or len(new)!=4:raise ValueError('Four native peg tip faces must register separately '+repr((len(old),len(new))))
    pegerrors=[min(np.linalg.norm(p-q) for q in old) for p in new]
    result=dict(authority=source['authority'],supplierManifestSha256=source['manifestSha256'],sources=hashes,
        method='Current REV V1.1 keyed NEO top-plate procedure; the spacer seating against the actual motor shoulder is an installation hypothesis. Grounded datum checks use actual registered rigid source geometry.',
        revision='Raw supplier solids only, provisional parent. Full cold installed assembly and loaded module still open.',
        instructions='https://docs.revrobotics.com/ion-build/build-guides/maxswerve-module-assembly',
        keyedNEOShoulderMm=shaft,spacerLengthMm=spacer_len,shaftSideSpacerEndMm=tip,keywayBaseRadiusMm=bottom,
        observedMotorKeyClockDeg=key_clock,bridgePegErrorsMm=pegerrors,placements={k:v.tolist() for k,v in matrices.items()},
        pairs=pairs,negativeControls=controls)
    (folder/'measured-v11-top-stack.json').write_text(json.dumps(result,indent=2))
    print(shaft,spacer_len,tip,key_clock,pegerrors)
    print(pairs)
    print(controls)


if __name__=='__main__':study(*[Path(arg) for arg in sys.argv[1:]])
