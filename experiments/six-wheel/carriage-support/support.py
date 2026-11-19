"""Create the machined fixed-height bridge supports in the registered robot frame, millimetres."""
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"mast-attachment"))
from mast import j,prism,hexagon,compound
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt

# The high and low rows straddle the existing mast attachments.
BOLTS=[(-53,350),(-35,350),(-53,399),(-35,399)]
FRONT=167.
REAR=-87.
Y_REAR=196.
Y_FRONT=(182.,197.)
# Single stiffened X/Z plate, with positive fore and aft pickup into different members.
OUTLINE=[(-101,398),(-60,340),(-28,340),(174,417),(174,433),
         (151,433),(-18,402),(-24,408),(-69,408),(-80,412),(-101,412)]
OPEN=[(-29,355),(140,416),(-24,390)]
REAR_WINDOW=[(-64,355),(-64,385),(-87,399)]
def cylinder(x,y,z,axis,r,length):
    return j.cad.brep(["cyl",[x,y,z,*axis,r,length]])

def zcyl(x,y,z,r,h):
    return cylinder(x,y,z,[0,0,1],r,h)

def xcyl(x,y,z,r,h):
    return cylinder(x,y,z,[1,0,0],r,h)

def cone(x,y,z,r1,r2,h):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,1)),r1,r2,h).Shape()

def fuse_all(parts):
    it=iter(parts)
    result=next(it)
    for p in it:result=j.fuse(result,p)
    return result

def side_box(s,x,X,y,Y,z,Z):return j.box(x,X,s*y,s*Y,z,Z)

def blank(s):
    web=prism(s,OUTLINE,201,205)
    for hole in (OPEN,REAR_WINDOW):web=j.cut(web,prism(s,hole,200,206))
    # Continuous thickened diagonal limits out-of-plane strut buckling.
    rib=prism(s,[(-31,341),(167,417),(159,420),(-34,347)],205,210)
    root=[side_box(s,-63,-25,205,210,z-8.,z+8.) for z in (350.,399.)]
    # Two attachment lands meet both crossbar Z412/outer X-97 and forward bridge Z433.
    rear_floor=side_box(s,-100,-77,187,205,405,412)
    front_floor=side_box(s,154,174,174,205,426,433)
    tip=prism(s,[(144,417),(174,417),(174,426),(154,426)],190,205)
    body=fuse_all([web,rib,*root,rear_floor,front_floor,tip])
    for x,z in BOLTS:
        body=j.cut(body,j.rod(x,s,193,201,z,7.1))
        body=j.cut(body,j.rod(x,s,199,212,z,3.25))
    body=j.cut(body,zcyl(REAR,s*Y_REAR,365,7.2,40))
    body=j.cut(body,zcyl(REAR,s*Y_REAR,402,3.25,12))
    for y in Y_FRONT:
        body=j.cut(body,zcyl(FRONT,y*s,412,2.75,25))
        body=j.cut(body,zcyl(FRONT,y*s,395,6.5,31))
    return body

def nut_disk(x,y,z,axis,r,length,hole):
    return j.cut(cylinder(x,y,z,axis,r,length),cylinder(x-axis[0],y-axis[1],z-axis[2],axis,hole,length+2))

def source():
    def read(folder,meta,file,ids_key,digest_key):
        row=j.read(ROOT/folder/meta)
        step=ROOT/folder/file
        if j.digest(step)!=row[digest_key]:raise ValueError('Different '+file)
        return j.cad.read_named(step,[{'id':k} for k in row[ids_key]])
    splices=read('mast-attachment','fit.json','mast-joints.step','volumeMm3','stepSha256')
    shelf=read('shelf-structure','installed.json','stow-receiver.step','volumeMm3','manufacturingSha256')
    low=j.read(ROOT/'design/fixed-height/native.json')['low-red-receive']
    ids=list(low['editedVolumesMm3'])+list(low['addedVolumesMm3'])
    locks=j.cad.read_named(ROOT/'design/fixed-height/low-locks.step',[{'id':k} for k in ids])
    return {'crossbar':locks['lift-crossbar-drilled'],'bridge':shelf['slide-base-bridge'],
            **{'mast-'+s:splices['mast-base-'+s+'-mast'] for s in ('L','R')}}

def make():
    orig=source()
    machined=dict(orig)
    new={}
    for side,s in (('L',1),('R',-1)):
        pre='support-'+side+'-'
        new[pre+'ribbed-bracket']=blank(s)
        for i,(x,z) in enumerate(BOLTS):
            name=pre+'mast-'+str(i)+'-'
            machined['mast-'+side]=j.cut(machined['mast-'+side],j.rod(x,s,209,236,z,5.05))
            new[name+'sleeve']=j.cut(j.rod(x,s,210,235,z,5),j.rod(x,s,209,236,z,3.10))
            # M6 x 45 stock, head inside, verified finished grip and engagement required.
            new[name+'screw']=j.fuse(hexagon(x,s,197,z,10,4),j.rod(x,s,201,244,z,3))
            new[name+'washer']=j.cut(j.rod(x,s,235,236.6,z,6),j.rod(x,s,234,238,z,3.2))
            new[name+'locknut']=j.cut(hexagon(x,s,236.6,z,10,6),j.rod(x,s,236,244,z,3.01))
        y=Y_REAR*s
        # M6 tapped into the aluminum crossbar. The major diameter represents the
        # union of the manufactured thread reservations; acceptance uses the tap minor.
        machined['crossbar']=j.cut(machined['crossbar'],zcyl(REAR,y,411.9,3.01,19))
        new[pre+'rear-screw']=j.fuse(zcyl(REAR,y,399.4,5.78,4),zcyl(REAR,y,403.4,3,25))
        new[pre+'rear-washer']=nut_disk(REAR,y,403.4,(0,0,1),6,1.6,3.2)
        for i,y in enumerate(Y_FRONT):
            y*=s
            name=pre+'front-'+str(i)+'-'
            bore=fuse_all([zcyl(FRONT,y,428,2.75,11),cone(FRONT,y,435.25,2.75,5.5,2.75)])
            machined['bridge']=j.cut(machined['bridge'],bore)
            new[name+'flush-screw']=j.fuse(zcyl(FRONT,y,415,2.5,20),cone(FRONT,y,435,2.5,5.5,3))
            new[name+'washer']=nut_disk(FRONT,y,425,(0,0,1),5,1,2.65)
            new[name+'locknut']=nut_disk(FRONT,y,420,(0,0,1),4.62,5,2.51)
    return orig,machined,new
