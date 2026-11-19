"""Removable lower and upper guide arms on both fixed masts, dimensions in millimetres."""
import importlib.util
import math
import sys
from pathlib import Path

from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "carriage-support"))
spec = importlib.util.spec_from_file_location("carriage_fit", ROOT / "carriage-support/fit.py")
carriage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carriage)
from support import xcyl, fuse_all, j

TOP = 813
STATIONS = (687, 787)
SIDES = (("L", 1), ("R", -1))
MAST_BOLTS = ((215.5, -5), (229.5, 5))


def center_x(z):
    return 165 + .46 * (z - 625)


def surface_y(z):
    return 150.5125 + .038 * max(0, z - 770)


def side_box(sign, x0, x1, y0, y1, z0, z1):
    return j.box(x0, x1, sign*y0, sign*y1, z0, z1)


def strip(sign, lo, hi, start, end, y0, y1):
    """A tapered planar sheet segment following the original guide contact surface."""
    return j.extrude([(center_x(z)+start, sign*(surface_y(z)+y), z)
                      for z, y in ((lo,y0), (hi,y0), (hi,y1), (lo,y1))],
                     (end-start,0,0))


def face(sign, liner):
    segments = []
    for lo, hi in ((625,770), (770,TOP)):
        if liner:
            segments.append(strip(sign,lo,hi,-20,20,0,3))
            continue
        lip = .8 * math.hypot(1,.46)
        segments.extend((strip(sign,lo,hi,-20,20,3,3.8),
                         strip(sign,lo,hi,-20,-20+lip,3.8,7.8),
                         strip(sign,lo,hi,20-lip,20,3.8,7.8)))
    return fuse_all(segments)


def hex_x(x, y, z, flats, h):
    radius=flats/math.sqrt(3)
    return j.extrude([(x,y+radius*math.cos(i*math.pi/3),z+radius*math.sin(i*math.pi/3))
                      for i in range(6)], (h,0,0))


def ycone(x, sign, y, z, big, small, length):
    axis=gp_Ax2(gp_Pnt(x, sign*y, z),gp_Dir(0,sign,0))
    return BRepPrimAPI_MakeCone(axis,big,small,length).Shape()


def make():
    folder=ROOT / "carriage-support"
    old=carriage.selected(folder,"carriage-support.step","fit.json","stepSha256","volumesMm3")
    parts={}
    for side,sign in SIDES:
        prefix="guide-"+side+"-"
        mast=old["support-machined-mast-"+side]
        backing=face(sign,False)
        parts[prefix+"liner"]=face(sign,True)
        for station in STATIONS:
            key=prefix+str(station)+"-"
            x=center_x(station)
            y=surface_y(station)
            inner=surface_y(station+6)+8.15
            end=x+17
            outer=side_box(sign,-14,end,inner,inner+30,station-6,station+6)
            cavity=side_box(sign,-15,end+1,inner+.8,inner+29.2,station-5.2,station+5.2)
            arm=j.cut(outer,cavity)
            root=side_box(sign,-20,-14,inner,235,station-11,station+11)
            for dz in (-10,8):
                root=j.fuse(root,side_box(sign,-42,-20,inner+2,209.5,station+dz,station+dz+2))
            # Collinear bores capture both mast walls with a continuous crush sleeve.
            for index,(by,dz) in enumerate(MAST_BOLTS):
                name=key+"root-"+str(index)+"-"
                bz=station+dz
                mast=j.cut(mast,xcyl(-71,sign*by,bz,3.3,52))
                root=j.cut(root,xcyl(-21,sign*by,bz,2.2,8))
                parts[name+"sleeve"]=j.cut(xcyl(-70,sign*by,bz,3.25,50),xcyl(-71,sign*by,bz,2.1,52))
                parts[name+"screw"]=j.fuse(hex_x(-14,sign*by,bz,7,2.8),xcyl(-79,sign*by,bz,2,65))
                parts[name+"washer"]=j.cut(xcyl(-70.8,sign*by,bz,4.5,.8),xcyl(-72,sign*by,bz,2.15,4))
                parts[name+"locknut"]=j.cut(hex_x(-75.8,sign*by,bz,7,5),xcyl(-77,sign*by,bz,2.01,8))
            # A removable tapped bar slides into the open tube nose. The wedge matches the backing slope.
            bar=side_box(sign,x-14,x+14,inner+.8,inner+9.8,station-5,station+5)
            wedge=j.extrude([(x-12,sign*fy,z) for fy,z in
                    ((surface_y(station-4)+3.8,station-4),(surface_y(station+4)+3.8,station+4),
                     (inner,station+4),(inner,station-4))], (24,0,0))
            for index,dx in enumerate((-8,8)):
                at=x+dx
                seat=y+3+.18
                cone=ycone(at,sign,seat,station,3,1.5,1.5)
                counter=j.fuse(j.rod(at,sign,seat-1,seat,station,3),cone)
                blank=j.rod(at,sign,seat+1.5,seat+16,station,1.5)
                parts[key+"face-screw-"+str(index)]=j.fuse(cone,blank)
                through=j.rod(at,sign,y+2,inner+10.5,station,1.65)
                backing=j.cut(j.cut(backing,counter),through)
                wedge=j.cut(j.cut(wedge,counter),through)
                arm=j.cut(arm,through)
                bar=j.cut(bar,j.rod(at,sign,inner,inner+11,station,1.51))
            parts[key+"mount-arm"]=j.fuse(root,arm)
            parts[key+"tapped-bar"]=bar
            parts[key+"wedge"]=wedge
        parts[prefix+"backing"]=backing
        parts[prefix+"machined-mast"]=mast
    return parts
