"""Original SP1 adapter and cable clevis. Supplier stock remains a dimensioned reservation."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "selected-match"))
from clearance import cad, native_box
from fit import subtract, unite
from rail_profile import locknut

Y, Z = -127., 415.
GUIDE, PIN = 26., 75.
FACE = Z + 12.2
MOUNT = [(-13.5 - 17.8, Y + 19.9), (-13.5 + 17.8, Y + 19.9)]
BRIDGE = [(x, y) for x in (-48, 34) for y in (-117, -92)]
REMOVED = {"slide-position-saddle", "slide-position-target", "slide-position-line", "slide-position-SP1-25"}


def box(x0,y0,z0,x1,y1,z1):
    return native_box((x0,y0,z0,x1,y1,z1))


def rod(x,y,z,axis,r,length):
    return cad.brep(["cyl", [x,y,z,*axis,r,length]])


def zrod(x,y,z,r,h):
    return rod(x,y,z,(0,0,1),r,h)


def yrod(x,y,z,r,l):
    return rod(x,y,z,(0,1,0),r,l)


def xrod(x,y,z,r,l):
    return rod(x,y,z,(1,0,0),r,l)


def fixed_adapter():
    # Four original pass-throughs keep their 2 mm mounting lands. Lower center
    # pad supports the supplied L-foot. Its two tapped mounting holes stop below bridge.
    pad=box(-53,-125,431,39,-84,433)
    pad=unite(pad,box(-38,-122,FACE,12,-86,431))
    for x,y in BRIDGE:
        pad=subtract(pad,zrod(x,y,425,1.65,10))
    for x,y in MOUNT:
        pad=subtract(pad,zrod(x,y,FACE-1,1.5,6.8))
    return pad


def moving_fork(stroke):
    # Continuous piece of plate stock; two cheeks have a common reamed bore.
    x=PIN+stroke
    slab=box(x,-133,481,x+44,-121,485)
    post=box(x,-130.5,420,x+6,-123.5,481)
    bridge=box(x-6,Y-5.25,422,x+6,Y+5.25,426)
    part=unite(unite(slab,post),bridge)
    for y in (Y-5.25,Y+2.75):
        lug=box(x-3.25,y,410,x+6,y+2.5,423)
        part=unite(part,subtract(lug,yrod(x,y-1,Z,2.05,4.5)))
    for a in (96,112):
        part=subtract(part,zrod(a+stroke,Y,480,1.65,7))
    return part


def shoulder(stroke):
    x=PIN+stroke
    head=yrod(x,Y-7.25,Z,3.75,2)
    bearing=yrod(x,Y-5.25,Z,2,10.8)
    thread=yrod(x,Y+5.55,Z,1.5,5.7)
    head=subtract(head,box(x-4,Y-7.35,Z-.55,x+4,Y-6.35,Z+.55))
    return unite(unite(head,bearing),thread)


def nut(stroke):
    # Turn original M3 locknut geometry so its axis follows the pin.
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Ax1,gp_Dir,gp_Pnt,gp_Trsf
    tr=gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,0,0)),-math.pi/2)
    raw=BRepBuilderAPI_Transform(locknut(0,0,4),tr,True).Shape()
    return cad.translated(raw,(PIN+stroke,Y+6.05,Z))


def manufactured(stroke):
    parts={"sp1-step-adapter":fixed_adapter(),"sp1-shelf-clevis":moving_fork(stroke),
           "sp1-eye-shoulder-pin":shoulder(stroke),"sp1-eye-locknut":nut(stroke),
           "sp1-eye-washer":subtract(yrod(PIN+stroke,Y+5.55,Z,3.5,.5),yrod(PIN+stroke,Y+5.05,Z,1.6,1.5))}
    for i,(x,y) in enumerate(MOUNT):
        # Standard service screw envelope: length 6 under head, minimum 4 mm engagement.
        parts[f"sp1-foot-M3-{i}"]=unite(zrod(x,y,FACE-5,2.8,3),zrod(x,y,FACE-2,1.5,6))
    return parts


def supplied(stroke):
    # Mount inverted, cable right in the manual p2 horizontal low illustration.
    # Gross housing/terminal stock is enclosed; contour, terminal boots and bracket
    # thickness are fit assumptions. Supplier geometry is not included in the STEP.
    foot=box(-37.65,Y+9.0,FACE-2,10.65,Y+29.6,FACE)
    # Maintain the documented two clear passages at the foot and an accessible
    # Reserve the central electrical terminals separately from the two mounting screws.
    main=box(-39.6,Y-17.7,375.4,11.7,Y+10.2,FACE-2)
    tails=box(-23,Y+10.2,390, -4,Y+29.6,408)
    for x,y in MOUNT:
        foot=subtract(foot,zrod(x,y,FACE-3,1.85,4))
    guide=xrod(10,Y,Z,6.65,GUIDE-10)
    housing=unite(unite(main,guide),unite(foot,tails))
    eye_x=PIN+stroke
    # Source max OD/thickness and minimum of its conflicting bore callouts.
    ring=subtract(yrod(eye_x,Y-2.16,Z,4.825,4.32),yrod(eye_x,Y-3,Z,2.095,6))
    neck=xrod(eye_x-23.9,Y,Z,3.9,19.6)
    return {"sp1-supplier-reservation":housing,"sp1-eye-reservation":unite(ring,neck),
            "sp1-live-cable":xrod(GUIDE,Y,Z,.019*25.4/2,eye_x-23.9-GUIDE)}
