"""Machine and place two removable tab saddles per corner with supplier button screws."""
import json
import sys
from pathlib import Path
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt
from chassis_layout import ROOT,OUT,digest,read
from output_mount import place


def cylinder(origin,axis,radius,length):
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*origin),gp_Dir(*axis)),radius,length).Shape()


def oriented(zaxis,seat,bearing):
    z=np.array(zaxis,dtype=float);x=np.array([0.,0.,1.])
    y=np.cross(z,x)
    return np.column_stack((x,y,z,np.array(seat)-z*bearing))


def roster(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    from chassis_interfaces import blank
    from chassis_layout import layout
    base=layout();dr=read(ROOT/'experiments/swerve-drive/packaging/module.json')
    specs={}
    for sku,length,pin in [
        ('REV-29-2943',12.7,'4c253e1b8a23613182cf6040a59f547948c3e6d3b0844ac81d0b3f365b8904cb'),
        ('REV-29-2951',25.4,'89e2b6ad909e8bdcb66fb86a7948298c159ffb0fd8c6b00bac022a7e54550ea2')]:
        file=ROOT/'artifacts/split-drop-native/supplier'/(sku+'.STEP')
        assert digest(file)==pin
        s,extra=load_solids(file);assert len(s)==1 and not extra
        sh=next(iter(s.values()))
        specs[sku]=(sh,bounds(sh)[2]+length,file,pin)
    # A physical section through either original tab gives the two actual
    # mounting faces at 31.750 and 82.550. Machine a 50.70 mm spacer, seat
    # it on the lower tab and fit the measured remaining stack with shim.
    def box(lo,hi):return BRepPrimAPI_MakeBox(gp_Pnt(*lo),gp_Pnt(*hi)).Shape()
    L=[-48.,31.75,52.];H=[-23.5,82.45,87.7]
    saddle=box(L,H);shim=box([-43.5,82.45,53.],[ -24.5,82.55,86.7])
    ops=[]
    for z in (57.15,82.55):
        # 4.0894 mm is the #20 pilot. Keep the minor bore and tapped depth
        # separate from tooth flank, runout and assembly torque validation.
        for y,dy in [(31.75,1),(82.45,-1)]:
            tool=cylinder([-31.75,y,z],[0,dy,0],2.0447,12.)
            saddle=BRepAlgoAPI_Cut(saddle,tool).Shape()
            ops.append(dict(startMm=[-31.75,y,z],axis=[0,dy,0],radiusMm=2.0447,depthMm=12.,process='Drill and tap #10-32 UNF, full thread length 10 mm.'))
        shim=BRepAlgoAPI_Cut(shim,cylinder([-31.75,82.45,z],[0,1,0],2.65,.1)).Shape()
    for y in (46.,67.):
        for radius,depth in [(2.65,24.5),(5.05,9.)]:
            saddle=BRepAlgoAPI_Cut(saddle,cylinder([-23.5,y,69.85],[-1,0,0],radius,depth)).Shape()
            ops.append(dict(startMm=[-23.5,y,69.85],axis=[-1,0,0],radiusMm=radius,depthMm=depth,
                            process='Through clearance followed by a flat counterbore for a real REV 1 inch screw.'))
    # Each 14 mm long tapped insert is flush with the inner rail face.
    # Its far end is closed; the seated horizontal screw reaches 9.9 mm.
    boss=cylinder([-48.,0.,0.],[-1,0,0],6.5,14.)
    boss=BRepAlgoAPI_Cut(boss,cylinder([-48.,0.,0.],[-1,0,0],2.0447,12.)).Shape()
    rail={r['name']:blank(r) for r in base['frame']}
    allparts={};items={};railcuts={k:[] for k in rail}
    def put(name,shape,transform,data):
        assert abs(np.linalg.det(transform[:3,:3])-1)<1e-9
        allparts[name]=place(shape,transform[:3,:]);items[name]=dict(matrixMm=transform[:3,:].tolist(),**data)
    def mat(m):
        result=np.eye(4);result[:3,:]=m;return result
    for corner,m in dr['modules'].items():
        world=mat(np.column_stack((m['sourceXyzToRobot'],[m['xM']*1000,m['yM']*1000,38.1])))
        for side,swap in [('A',np.eye(4)),('B',np.array([[0,0,-1,139.7],[0,1,0,0],[1,0,0,0],[0,0,0,1]]))]:
            transform=world@swap
            label=corner+' '+side+' '
            put(label+'saddle',saddle,transform,dict(kind='saddle',stockBoundsMm=[L,H],drilling=ops,material='6061-T6 aluminum billet'))
            put(label+'seat shim',shim,transform,dict(kind='shim',stockBoundsMm=[[-43.5,82.45,53.],[-24.5,82.55,86.7]],
                    material='0.100 mm stainless shim stock, select after measuring actual tab opening'))
            for y in (46.,67.):
                offset=np.eye(4);offset[:3,3]=[0,y,69.85]
                key=label+'rail boss '+str(y)
                put(key,boss,transform@offset,dict(kind='boss',material='6061-T6 round stock; derate circumferential weld heat-affected zone'))
                c=(transform@np.array([-48.,y,69.85,1]))[:3];axis=transform[:3,0]*-1
                name=('left' if c[1]>0 else 'right') if abs(axis[1])>.9 else ('front' if c[0]>0 else 'rear')
                name+=' frame tube'
                rail[name]=BRepAlgoAPI_Cut(rail[name],cylinder(c,axis,6.55,14.)).Shape()
                railcuts[name].append(dict(centerMm=c.tolist(),axis=axis.tolist(),radiusMm=6.55,depthMm=14.,boss=key))
            for role,sku,d,seat,engage in [
                *[(f'lower tab {z}','REV-29-2943',[0,-1,0],[-31.75,27.46375,z],8.41375) for z in (57.15,82.55)],
                *[(f'upper tab {z}','REV-29-2943',[0,1,0],[-31.75,86.2125,z],8.9375) for z in (57.15,82.55)],
                *[(f'frame {y}','REV-29-2951',[1,0,0],[-32.5,y,69.85],9.9) for y in (46.,67.)]]:
                s,bearing,fn,sha=specs[sku]
                put(label+role+' '+sku,s,transform@mat(oriented(d,seat,bearing)),dict(kind='bought',sku=sku,
                    article=fn.relative_to(ROOT).as_posix(),sha256=sha,engagementMm=engage,
                    material='Manufacturer alloy steel, black oxide; grade unspecified',sharing='Private fit; no public redistribution clearance.'))
    assert len(allparts)==80 and sum(map(len,railcuts.values()))==16
    return base,rail,allparts,dict(installed=items,frameBores=railcuts)


if __name__=='__main__':
    import argparse
    from chassis_interfaces import blank
    ap=argparse.ArgumentParser();ap.add_argument('plugin',type=Path)
    plugin=ap.parse_args().plugin
    base,rail,parts,data=roster(plugin)
    from mechanical_motion import bounds,common,distance
    from showcase_contacts import load_solids,box_gap
    registered=read(OUT/'registered-interfaces.json')
    plates={}
    for k,r in registered['installedPurchasedPlates'].items():
        path=ROOT/r['article'];assert digest(path)==r['articleSha256']
        plates[k]=place(next(iter(load_solids(path)[0].values())),np.array(r['matrixMm']))
    stock=rail|plates|parts;boxes={k:bounds(s) for k,s in stock.items()}
    failures=[];contacts=[]
    for i,(a,s) in enumerate(stock.items()):
        for b in list(stock)[i+1:]:
            if a not in parts and b not in parts:continue
            if box_gap(boxes[a],boxes[b])>.12:continue
            gap=distance(s,stock[b]);vol=common(s,stock[b]) if gap<1e-7 else 0.
            if gap>.12:continue
            r=dict(a=a,b=b,gapMm=gap,volumeMm3=vol);contacts.append(r)
            same=a[:4]==b[:4]
            threaded=same and (('saddle' in a and 'tab ' in b and 'REV-29' in b) or ('rail boss' in a and 'frame ' in b and 'REV-29' in b))
            # Compare named screw seats below; each frame screw must only
            # engage its actual boss. A second eligible item would fail counts.
            if vol>.001 and not threaded:failures.append(r)
    threads=[r for r in contacts if r['volumeMm3']>.001 and r not in failures]
    assert len(threads)==48,('Missing or duplicate named threaded volume',len(threads))
    data.update(authority=base['authority'],registeredPlates=registered['installedPurchasedPlates'],
                threadModel='Smooth supplier major diameter vs drilled tapped pilot. CAD pair classification does not resolve flank fit, stripping, torque or fabrication tolerance.',
                parkedNearby=contacts,threadPairs=len(threads),unexpected=failures)
    (OUT/'module-mounts.json').write_text(json.dumps(data,indent=2)+'\n')
    print(len(parts),'hardware/stock',len(threads),'threaded contacts',len(failures),'unexpected',failures[:4])
