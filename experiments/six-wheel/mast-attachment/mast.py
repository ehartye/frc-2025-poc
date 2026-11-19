"""Create the machined ribbed side splices and hand tooling for two reversible mast feet."""
from pathlib import Path
import sys
import importlib.util
import math

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"deck-joints"))
spec=importlib.util.spec_from_file_location("deck_fit",ROOT/"deck-joints/fit.py")
deck=importlib.util.module_from_spec(spec)
spec.loader.exec_module(deck)
from kit import j
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound

RAIL=[(-116,128),(-116,141.5),(42,128),(42,141.5)]
MAST=[(-51,165),(-33,165),(-51,217),(-33,217)]
PROFILE=[(-132,121),(67,121),(67,146),
         (-20,267),(-20,275),(-70,275),(-70,267),(-132,146)]
SLOT=(-135.,68.,235.,238.6)


def prism(s,polygon,start,stop):
    return j.extrude([(x,s*start,z) for x,z in polygon],(0,s*(stop-start),0))


def clip_below(poly,height):
    clipped=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        if a[1]<=height:clipped.append(a)
        if (a[1]<height)!=(b[1]<height):
            clipped.append((a[0]+(b[0]-a[0])*(height-a[1])/(b[1]-a[1]),height))
    return clipped


def profile(s,full=False):
    perimeter=PROFILE if s>0 else PROFILE[:-1]+[(-81,194),(-87,168),(-132,146)]
    rear_rib=([(-125,158),(-117,158),(-63,249),(-70,252)] if s>0 else
              [(-80,192),(-73,192),(-63,249),(-70,252)])
    ribs=[rear_rib,[(28,158),(36,158),(-20,252),(-27,249)]]
    return (perimeter,ribs) if full else (clip_below(perimeter,229.),[clip_below(p,225.) for p in ribs])


def plate(s,full=False):
    perimeter,ribs=profile(s,full)
    part=prism(s,perimeter,235,238)
    # Large asymmetric rounded tooling can accommodate these triangular empty panels.
    # Preserve the full lower chord and a 22 mm wide load path to each mast edge.
    rear_open=[(-108,150),(-71,216),(-71,171),(-61,150)]
    for opening in ([rear_open] if s>0 else [])+[[(19,150),(-19,215),(-19,171),(-29,150)]]:
        # Leave the full splice web between the upper rail bolts and lower mast
        # bolts. A deep lightening window here weakened the section crossing the deck.
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
        window=BRepAlgoAPI_Common(prism(s,opening,234,239),j.box(-140,70,s*233,s*240,194,248)).Shape()
        part=j.cut(part,window)
    # Integral external stiffeners, machined from the same 8 mm stock above the sheet.
    for rib in ribs:
        part=j.fuse(part,prism(s,rib,238,243))
    old=[(-51,194),(-33,194),(-51,263),(-33,263)]
    for x,z in RAIL+(old if full else MAST):
        part=j.cut(part,j.rod(x,s,234,245,z,3.25))
    return part


def upward_removal(s):
    # Exact swept outer polygons including the 0.25 mm outboard seating translation.
    # Treat lightening openings as solid when disassembling the bracket.
    perimeter,ribs=profile(s)
    allparts=[]
    for poly,start,end in [(perimeter,235,238.25)]+[(r,238,243.25) for r in ribs]:
        allparts.append(prism(s,poly,start,end))
        dz=35.
        for p,q in zip(poly,poly[1:]+poly[:1]):
            if abs(q[0]-p[0])>1e-8:
                allparts.append(prism(s,[p,q,(q[0],q[1]+dz),(p[0],p[1]+dz)],start,end))
    return compound(allparts)


def lateral_jog(s,dx):
    perimeter,ribs=profile(s)
    allparts=[]
    for polygon,start,end in [(perimeter,235.25,238.25)]+[(r,238.25,243.25) for r in ribs]:
        poly=[(x,z+35) for x,z in polygon]
        allparts.append(prism(s,poly,start,end))
        for p,q in zip(poly,poly[1:]+poly[:1]):
            if abs(q[1]-p[1])>1e-8:
                allparts.append(prism(s,[p,q,(q[0]+dx,q[1]),(p[0]+dx,p[1])],start,end))
    return compound(allparts)


def outward_removal(s,dx=0,full=False):
    perimeter,ribs=profile(s,full)
    groups=[(perimeter,235.,238.25)]+[(r,238.,243.25) for r in ribs]
    return compound([prism(s,[(x+dx,z+35) for x,z in p],y,y2+220.) for p,y,y2 in groups])


def hexagon(x,s,y,z,af,length):
    rad=af/(2*math.cos(math.pi/6))
    vertices=[(x+rad*math.cos(n*math.pi/3),z+rad*math.sin(n*math.pi/3)) for n in range(6)]
    return prism(s,vertices,y,y+length)


def fasteners(x,z,s,upper):
    inner=210 if upper else 229
    outer=238.
    # Finished underhead lengths on the real stack, 37.5 or 18.5 mm;
    # thread major/root and helix are conservative reference envelopes.
    head=hexagon(x,s,inner-4,z,10,4)
    screw=j.fuse(head,j.rod(x,s,inner,outer+9.5,z,3))
    washer=j.cut(j.rod(x,s,outer,outer+1.6,z,6),j.rod(x,s,outer-.1,outer+2,z,3.2))
    nut=j.cut(hexagon(x,s,outer+1.6,z,10,6),j.rod(x,s,outer+1,outer+8,z,3))
    result={"screw":screw,"washer":washer,"nut":nut}
    if upper:
        result["sleeve"]=j.cut(j.rod(x,s,210,235,z,5),j.rod(x,s,209,236,z,3.1))
    return result


def slot(stock):
    for s in (-1,1):
        stock=j.cut(stock,j.box(SLOT[0],SLOT[1],s*SLOT[2],s*SLOT[3],145,158))
    return stock


def overlay(source,deck_name):
    parts={"mast-slotted-deck":slot(source[deck_name])}
    for side,s in (("L",1),("R",-1)):
        mast=source["backup-"+side+"-drilled-mast"]
        rail=source["drive-rail-inner-"+side]
        prefix="mast-base-"+side+"-"
        parts[prefix+"splice"]=plate(s)
        for index,(x,z) in enumerate(RAIL+MAST):
            upper=index>=len(RAIL)
            if upper: mast=j.cut(mast,j.rod(x,s,209,236,z,5.05))
            else: rail=j.cut(rail,j.rod(x,s,228,236,z,3.25))
            for k,v in fasteners(x,z,s,upper).items():
                parts[prefix+str(index)+"-"+k]=v
        parts[prefix+"mast"]=mast
        parts[prefix+"rail"]=rail
    return parts


def compound(items):
    b=BRep_Builder()
    shape=TopoDS_Compound()
    b.MakeCompound(shape)
    for p in items: b.Add(shape,p)
    return shape
