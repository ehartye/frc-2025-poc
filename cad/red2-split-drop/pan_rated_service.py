"""Check a dimensionally bounded short-series 7/64 inch L-key against the actual cold robot."""
import json
import math
import sys
from pathlib import Path
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism,BRepPrimAPI_MakeCylinder
from OCP.GeomAbs import GeomAbs_Plane
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Vec
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from output_mount import ROOT,digest,read,place
from pan_thrust_layout import cylinder,subtract,transform


def radial_sector(x,y,z,start,angle,outer,height,inner=0):
    a=math.radians(start)
    frame=gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0.,0.,1.),gp_Dir(math.cos(a),math.sin(a),0.))
    shape=BRepPrimAPI_MakeCylinder(frame,outer,height,math.radians(angle)).Shape()
    if inner:shape=subtract(shape,cylinder((x,y,z-.1),(0,0,1),inner,height+.2))
    return shape


def check(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    d=ROOT/'artifacts/red2-integrated/pan-thrust'
    fit=read(d/'rated-barrel-fit.json')
    field=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/field['file'])==fit['authority']['sha256']
    step=d/'fixed-races/checked-297.step'
    assert digest(step)==fit['parentStep']
    sh=d/'fastener-option'/fit['source']['file']
    assert digest(sh)==fit['source']['sha256']
    screw,=load_solids(sh)[0].values()
    print('Loaded exact purchase',flush=True)
    source_flip=transform(np.diag([1.,-1.,-1.]),[0,0,489.5-38.1])
    seated=place(screw,source_flip)
    # Read all six actual socket flats around the center axis. Exclude shank,
    # chamfer and outer end surfaces by plane, normal, radius and axial limits.
    faces=[];top=bounds(seated)[5];ex=TopExp_Explorer(seated,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face(ex.Current());ex.Next();s=BRepAdaptor_Surface(f)
        if s.GetType()!=GeomAbs_Plane:continue
        p=s.Plane();n=p.Axis().Direction()
        if abs(n.Z())>.001:continue
        dist=abs(p.Distance(gp_Pnt(0.,0.,top)));bb=bounds(f)
        if 1.30<dist<1.50 and bb[2]>top-3:
            faces.append(dict(angle=math.degrees(math.atan2(n.Y(),n.X()))%360,r=dist,zMin=bb[2],zMax=bb[5]))
    if len(faces)!=6:raise ValueError(('Lost the six source socket flats',faces))
    af=25.4*7/64
    angle=math.radians(faces[0]['angle']+30)
    ring=BRepBuilderAPI_MakePolygon()
    for k in range(6):
        theta=angle+math.pi*k/3
        ring.Add(gp_Pnt(af/math.sqrt(3)*math.cos(theta),af/math.sqrt(3)*math.sin(theta),top-1.8))
    ring.Close()
    key=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(ring.Wire()).Face(),gp_Vec(0,0,6)).Shape()
    print('Testing actual hex socket',flush=True)
    keyed_volume=common(key,seated)
    if keyed_volume>.001:raise ValueError(('Exact source socket does not accept stated key',faces,keyed_volume))
    print('Reading 297 parked bodies',flush=True)
    paths={};world,extras=load_solids(step,paths)
    if len(world)!=297 or extras:raise ValueError('Incomplete cold robot')
    for label,row in fit['joints'].items():
        loc=row['plan']
        name='Proposed 632120 '+label
        # Every grip/handle sweep lies above z489.5. Enclose the complete real head
        # with its maximum circumscribed cylinder; the threaded shank is below it.
        world[name]=cylinder((loc['x'],loc['y'],489.5),(0,0,1),2.778,3.506)
        paths[name]={'occurrencePath':[name]}
    print('Indexing installed neighbors',flush=True)
    boxes={k:bounds(v) for k,v in world.items()}
    results={}
    rate=read(d/'rated-barrel-strength.json')
    for site,station in fit['joints'].items():
        x,y=(station['plan'][k] for k in ['x','y'])
        # For an arbitrary screw clock, 120 degrees guarantees at least one 60-degree
        # remove/reindex stroke. The distal arm/body regions include the full width,
        # a 12 mm radius access allowance near the hand, and the vertical reseating stroke.
        coll={}
        print('checking',site,flush=True)
        def hits(s):
            bb=bounds(s)
            return {'/'.join(paths[k]['occurrencePath']):v for k in world if box_gap(bb,boxes[k])<.001 and (v:=common(s,world[k]))>.001}
        stem_hits=hits(cylinder((x,y,top+.001),(0,0,1),1.65,24))
        elbows=hits(cylinder((x,y,top+.001),(0,0,1),7.,24))
        for center in range(0,360,15):
            # Within radius 7 mm the phase-independent bend circle covers every clock.
            # Beyond it 15 deg > asin(1.61/7) bounds the swept hex bar plus its corners.
            arm=radial_sector(x,y,top+10.,center-75,150,57.,14.,6.5)
            # For the last 16 mm of usable lever, +/-20 deg covers a 12 mm hand
            # allowance down to radius 35 mm. Excludes any claim for a full human hand.
            grip=radial_sector(x,y,top-1.,center-81,162,69.,36.,24.)
            missed={'arm':hits(arm),'grip':hits(grip)}
            coll[center]=missed
            if not stem_hits and not elbows and not any(missed.values()):break
        clear=[c for c,m in coll.items() if not any(m.values())]
        print('swing',site,clear[:1],flush=True)
        results[site]=dict(clear=bool(clear and not stem_hits and not elbows),selectedCenterDeg=next(iter(clear),None),stemHits=stem_hits,elbowHits=elbows,angularCases=coll)
    report=dict(authority=field,parentStep=digest(step),frozenScrew=fit['source'],shortKeySpec='Unbrako short-series 7/64 to ANSI B18.3, inch engineering guide p32 dimensions/mechanical properties',
        source='https://www.unbrako.com.au/images/downloads/engg_guide.pdf',sourceSocketFlats=faces,fullKeyAFmm=af,keyVerticesVsActualSocketCommonMm3=keyed_volume,actualFullDepthMm=min(f['zMax']-f['zMin'] for f in faces),keyInsertMm=1.8,
        tool=dict(shortLegIn=[.609,.797],longLegIn=[2.031,2.219],torsionalYieldInLb=60.,minimumTorsionalYieldNm=6.7791,nominalLeverAtAppliedLoadMm=45.,inclusiveSweepDeg=120),
        service='Stationary stow; insert short end and index long arm by 60 degrees. Bounds cover initial engagement and up to 3 mm reseating lift. The torque/preload relation is an acceptance on the actual rated lot; a force read on a calibrated tangential lever during service provides a repeatable torque, with lubrication held fixed.',
        torque=dict(threadNmm=max(r['fullThreadTorqueNmm'] for r in rate['cases'].values()),underHeadFrictionMax=.12,
        totalAppliedUpperNm=(max(r['fullThreadTorqueNmm'] for r in rate['cases'].values())+2200*.12*2.778)/1000,
        leverForceUpperN=(max(r['fullThreadTorqueNmm'] for r in rate['cases'].values())+2200*.12*2.778)/45),
        cases=results,limits='Installed predecessor and accessible proposed bracing at stationary maintenance stow. Later additions require rerunning. The real wrench, torque/settling calibration and separate lower grip fixture are shop traveler entries.')
    (d/'rated-barrel-short-key.json').write_text(json.dumps(report,indent=2)+'\n')
    print([(k,v['clear'],v['selectedCenterDeg']) for k,v in results.items()],flush=True)
    print('source flats',faces,flush=True)


if __name__=='__main__':check(Path(sys.argv[1]))
