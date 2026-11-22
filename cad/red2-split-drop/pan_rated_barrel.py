"""Test an eight-station positive brace joint using the rated Actobotics 632120 screw."""
import json
import math
import sys
from pathlib import Path

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from output_mount import ROOT, digest, read, place
from pan_counterbrace import box
from pan_race_patches import transverse_faces
from pan_thrust_layout import cylinder, subtract, fuse, transform, translate


def horizontal_seat(one, two, z):
    def face_list(s):
        from OCP.BRepAdaptor import BRepAdaptor_Surface
        from OCP.GeomAbs import GeomAbs_Plane
        from OCP.TopAbs import TopAbs_FACE
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopoDS import TopoDS
        enum = TopExp_Explorer(s, TopAbs_FACE)
        while enum.More():
            f = TopoDS.Face(enum.Current())
            enum.Next()
            surf = BRepAdaptor_Surface(f)
            if surf.GetType() == GeomAbs_Plane:
                p = surf.Plane()
                if abs(p.Axis().Direction().Z()) > 1-1e-8 and abs(p.Location().Z()-z) < .0001:
                    yield f
    total = 0.
    for left in face_list(one):
        for right in face_list(two):
            q = BRepAlgoAPI_Common(left,right)
            if not q.IsDone() or q.Shape().IsNull():
                raise ValueError("Unresolved coplanar face pair")
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(q.Shape(),props)
            total += props.Mass()
    return total


def check(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common
    from showcase_contacts import load_solids, box_gap
    p = ROOT/'artifacts/red2-integrated/pan-thrust'
    issued = p/'fixed-races'
    duty = read(p/'r3-brace-section-strength.json')
    register = read(ROOT/'cad/fusion-authority.json')['source']['archive']
    current = read(issued/'pan-r3-fixed-races-01.json')
    step = issued/'checked-297.step'
    if (digest(ROOT/register['file']) != current['authority']['archiveSha256']
            or digest(step) != current['stepSha256'] or duty['authority'] != register):
        raise ValueError("Changed authority, candidate or load history")
    manifest = read(p/'fastener-option/632120-source.json')
    article = next(v for v in manifest['sources'] if v['catalog']=='637174')
    source = p/'fastener-option'/article['file']
    if digest(source) != article['sha256']:
        raise ValueError('Changed 2018 purchased article')
    screw, = load_solids(source)[0].values()
    names = {}
    robot, extras = load_solids(step,names)
    if extras or len(robot)!=297:
        raise ValueError('Missing installed robot solids')
    def find(label):
        hits=[key for key,row in names.items() if label in row['occurrencePath']]
        if len(hits)!=1:raise ValueError((label,hits))
        return hits[0]
    def cyl(x,y,z,r,h):
        return cylinder((x,y,z),(0,0,1),r,h)
    beams={}
    for x in (186,224):
        beams[f'{x} upper']=box((x-12.5,-233,477.5),(x+12.5,233,480))
        beams[f'{x} lower']=box((x-12.5,-233,455),(x+12.5,233,457.5))
        for n,lx in enumerate((x-12.5,x+10)):
            beams[f'{x} web {n}']=box((lx,-233,457.5),(lx+2.5,233,477.5))
    joints={}
    hardware={}
    for side,y_sign in [('port',1),('starboard',-1)]:
        foot=find(side.title()+' bearing tower foot')
        rail=find(('left' if y_sign==1 else 'right')+' upper carriage sill')
        for x in (186,224):
            for station,abs_y,bottom in [('cross',205,455),('sill',242,450)]:
                key=f'{side}/{x}/{station}'
                y=abs_y*y_sign
                host=f'{x} lower' if station=='cross' else rail
                pocket=cyl(x,y,448,5.55,33)
                robot[foot]=subtract(robot[foot],cyl(x,y,479,2.65,9))
                if station=='cross':
                    beams[host]=subtract(beams[host],pocket)
                    beams[f'{x} upper']=subtract(beams[f'{x} upper'],pocket)
                else:robot[host]=subtract(robot[host],pocket)
                barrel=fuse(cyl(x,y,bottom-3,8,3),cyl(x,y,bottom,5.53,479.99-bottom))
                barrel=fuse(barrel,cyl(x,y,479.99,2.63,4.01))
                # Nominal internal thread major diameter; mating flank strength
                # and interrupted length are evaluated separately from envelope fit.
                barrel=subtract(barrel,cyl(x,y,466.,3.5052/2,19))
                for offset in [-6.15,6.15]:
                    barrel=subtract(barrel,cyl(x+offset,y,bottom-3.1,1.25,2.1))
                spread=subtract(cyl(x,y,487,5.8,2.5),cyl(x,y,486.9,1.875,2.7))
                bolt=place(screw,transform(np.diag([1.,-1.,-1.]),[x,y,489.5-38.1]))
                articles={'bolt':bolt,'washer':spread,'barrel':barrel}
                hardware.update({key+'/'+k:s for k,s in articles.items()})
                joints[key]=dict(foot=foot,host=host,bottom=bottom,x=x,y=y)
    placed=robot|beams|hardware
    boxes={k:bounds(s) for k,s in placed.items()}
    results={}
    for key,station in joints.items():
        x,y,bottom=(station[k] for k in ('x','y','bottom'))
        bolt,washer,barrel=(hardware[key+'/'+k] for k in ['bolt','washer','barrel'])
        foot,host=placed[station['foot']],placed[station['host']]
        signed={q:common(translate(barrel,[0.,0.,q]),host) for q in [.05,.25]}
        patch=horizontal_seat(bolt,washer,489.5)
        bare=horizontal_seat(translate(bolt,[0,0,-2.5]),foot,487)
        foot_patch=horizontal_seat(washer,foot,487)
        flange_patch=horizontal_seat(barrel,host,bottom)
        if min(patch,foot_patch,flange_patch)<1. or abs(signed[.05]/.05-flange_patch)>.01:
            raise ValueError((key,patch,foot_patch,flange_patch,signed))
        controls={}
        for kind,gauge,excluded in [
                ('parked screw',bolt,{'bolt'}),('parked washer',washer,{'washer'}),
                ('parked barrel',barrel,{'barrel'}),
                ('driver approach',cyl(x,y,490.98,2.7,90),{'bolt'}),
                ('screw extraction envelope',cyl(x,y,489.51,2.871,44),{'bolt'}),
                ('spanner approach envelope',cyl(x,y,bottom-42,8.,39),{'barrel'}),
                ('barrel extraction after screw',cyl(x,y,bottom-40,5.53,40),{'barrel','bolt'})]:
            hit={}
            bb=bounds(gauge)
            for k,s in placed.items():
                if k in {key+'/'+item for item in excluded} or box_gap(bb,boxes[k])>.0001:continue
                vol=common(gauge,s)
                if vol>.001:hit[k]=vol
            controls[kind]=hit
        forces=duty['stations'][key]['factoredN']
        results[key]=dict(plan=station,headToWasherMm2=patch,washerToFootMm2=foot_patch,
                          bareHeadToFootMm2=bare,flangeToHostMm2=flange_patch,
                          upliftPerturbationCommonMm3=signed,geometry=controls,factoredN=forces)
    target=p/'rated-barrel-fit.json'
    target.write_text(json.dumps(dict(authority=register,parentArchive=current['archiveSha256'],parentStep=digest(step),
                  source=article,dutySha256=digest(p/'r3-brace-section-strength.json'),joints=results,
                  scope='Offline installation on actual cold stow. Half-open withdrawal cylinder and driver/spanner envelopes are partial access controls. Thread strength, actual tools, separate assembly stages, complete motion, welds and clamp calibration require independent qualification.'),indent=2)+'\n')
    print([(k,round(r['headToWasherMm2'],3),round(r['bareHeadToFootMm2'],3),round(r['washerToFootMm2'],3),r['geometry']) for k,r in results.items()],flush=True)


if __name__=='__main__':
    check(Path(sys.argv[1]))
