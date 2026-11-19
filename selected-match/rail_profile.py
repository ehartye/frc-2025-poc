"""Compare a raised inner-channel floor and a rearward draw-wire mounting station."""
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism, BRepPrimAPI_MakeCone
from OCP.gp import gp_Pnt, gp_Vec, gp_Ax2, gp_Dir
import math
from clearance import cad
from fit import subtract, unite

TARGET_SHIFT = -42.
WIRE_Y = -127.
TARGET_SIDE = WIRE_Y + 208.
CHANNEL_Z = 446.
NOSE_RISE = 19.
RAMP = (58.,98.)
TARGET_NAMES = ("slide-position-SP1-25","slide-position-saddle","slide-position-target","slide-position-line")


def profile():
    nose = []
    for i in range(17):
        t = i/16
        x = RAMP[0] + t*(RAMP[1]-RAMP[0])
        rise = NOSE_RISE*(3*t*t-2*t*t*t)
        nose.append((x,CHANNEL_Z+rise))
    return nose + [(160.,CHANNEL_Z+NOSE_RISE)]


def prism_y(points, y, width):
    border = BRepBuilderAPI_MakePolygon()
    for x,z in points:
        border.Add(gp_Pnt(x,y,z))
    border.Close()
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(border.Wire()).Face(),gp_Vec(0,width,0)).Shape()


def channel(original, side, stroke):
    # The replacement is one continuous U milled from stock. Preserve the eight
    # existing wall holes while relocating 3 mm of floor up into the open profile.
    center = 180 if side == "L" else -180
    path = [(x+stroke,z) for x,z in profile()]
    cutter = prism_y([*path,(160+stroke,440),(RAMP[0]+stroke,440)],center-11,22)
    raised = prism_y([*path,*[(x,z+3) for x,z in reversed(path)]],center-10,20)
    result = unite(subtract(original,cutter),raised)
    if cad.solid_count(result) != 1:
        raise ValueError("New rail must remain one machinable solid")
    return result


def cyl(x,y,z,radius,height):
    return cad.brep(["cyl",[x,y,z,0,0,1,radius,height]])


def countersink(x,y,z,head,tip):
    return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(x,y,z),gp_Dir(0,0,-1)),head,tip,head-tip).Shape()


def locknut(x,y,top):
    polygon = BRepBuilderAPI_MakePolygon()
    for i in range(6):
        angle = i*math.pi/3
        radius = 5.5/math.sqrt(3)
        polygon.Add(gp_Pnt(x+radius*math.cos(angle), y+radius*math.sin(angle),top-4.0))
    polygon.Close()
    nut = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(0,0,4)).Shape()
    return subtract(nut,cyl(x,y,top-5,1.5,6))


def fasten(kit,prefix,stations,upper,lower,top,nut_top,length):
    for i,(x,y) in enumerate(stations):
        passage=cyl(x,y,nut_top-1,1.65,top-nut_top+2)
        kit[upper]=subtract(kit[upper],unite(passage,countersink(x,y,top,3.4,1.65)))
        kit[lower]=subtract(kit[lower],passage)
        screw=unite(cyl(x,y,top-length,1.5,length-1.86),countersink(x,y,top,3.36,1.5))
        kit[f"{prefix}-M3-{i}"]=screw
        kit[f"{prefix}-locknut-{i}"]=locknut(x,y,nut_top)


def revised(bodies,stroke):
    kit = {}
    for side in ("L","R"):
        name=f"drilled-slide-{side}-inner"
        kit[name]=channel(bodies[name],side,stroke)
    # The fixed bridge had crossed the relocated position flag at stow. A front-open
    # slot leaves both rail bearing strips and the motor/saddle interfaces intact.
    slot = unite(cad.brep(["box",[55,WIRE_Y-6,430,124,12,12]]),
                 cad.brep(["cyl",[55,WIRE_Y,430,0,0,1,6,12]]))
    kit["slide-base-bridge"] = subtract(bodies["slide-base-bridge"],slot)
    for key in TARGET_NAMES:
        kit[key]=cad.translated(bodies[key],(TARGET_SHIFT,TARGET_SIDE,0))
    # Flush heads keep the piece-facing shelf free of protrusions. The thin saddle
    # extends in X so its nut pairs clear the original supplier housing envelope.
    target="slide-position-target"
    shelf="receiver-R6-drilled-for-camera"
    kit[shelf]=bodies[shelf]
    flange=cad.brep(["box",[75+stroke,WIRE_Y-6,481,44,12,4]])
    kit[target]=unite(kit[target],flange)
    fasten(kit,"flag",[(96+stroke,WIRE_Y),(112+stroke,WIRE_Y)],shelf,target,500,481,25)
    saddle="slide-position-saddle"
    kit[saddle]=unite(kit[saddle],cad.brep(["box",[-53,WIRE_Y+3,431,92,39,2]]))
    fasten(kit,"sensor-saddle",[(x,y) for x in (-48,34) for y in (WIRE_Y+10,WIRE_Y+35)],
           "slide-base-bridge",saddle,438,431,12)
    return kit


def channel_sections(side):
    center = 180 if side == "L" else -180
    line = [(-160.,CHANNEL_Z),*profile()]
    return [[a[0],center-10,min(a[1],b[1]),b[0],center+10,479] for a,b in zip(line,line[1:])]
