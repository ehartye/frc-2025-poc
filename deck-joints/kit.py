"""Machine the six full-height deck ties from the registered unpocketed deck and end rails."""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT/"drive-side-attachment"))
import joint as j

AXES = [(s*339.,y) for s in (-1,1) for y in (-225.,-s*72.,225.)]
CHANGED = ("deck-main","cross-rear","cross-front")
REMOVAL = 124.


def cylinder(x,y,z,r,height):
    return j.cad.brep(["cyl",[x,y,z,0,0,1,r,height]])


def ring(x,y,z,outer,inner,height):
    return j.cut(cylinder(x,y,z,outer,height),cylinder(x,y,z-1,inner,height+2))


def hexagon(x,y,z,r,length):
    pts=[(x+r*math.cos(i*math.pi/3),y+r*math.sin(i*math.pi/3),z) for i in range(6)]
    return j.extrude(pts,(0,0,length))


def bodies(x,y):
    center=math.copysign(340,x)
    foot=j.cut(j.box(center-9.75,center+9.75,y-11,y+11,46,50),cylinder(x,y,45,2.2,6))
    sleeve=j.fuse(foot,ring(x,y,50,4,2.20,97.95))
    # Finished screw to the major diameter. The hex recess remains open for the holding key.
    cap=j.cut(cylinder(x,y,42,3.5,4),hexagon(x,y,41.9,1.5/math.cos(math.pi/6),2.3))
    tie=j.fuse(cap,cylinder(x,y,46,2.,117.5))
    washer=ring(x,y,154,4.5,2.15,.8)
    nut=j.cut(hexagon(x,y,154.8,3.5/math.cos(math.pi/6),5),cylinder(x,y,154,2,7))
    return {"sleeve":sleeve,"screw":tie,"washer":washer,"locknut":nut}


def name(x,y):
    return ("front" if x>0 else "rear")+"-"+str(int(y))


def integral(x,y):
    center=math.copysign(340,x)
    sign=math.copysign(1,x)
    foot=j.box(center-9.75,center+9.75,y-11,y+11,46,50)
    foot=j.fuse(foot,j.box(sign*328,sign*331,y-11,y+11,46,50))
    # The full-length inboard lip keys against either end of the same tube side face.
    # Outboard material would intrude on the bumper's bearing face.
    foot=j.fuse(foot,j.box(sign*328,sign*329.8,y-11,y+11,49,53))
    neck=cylinder(x,y,50,4,97.95)
    stud=cylinder(x,y,147.95,2.5,16.05)
    tie=j.fuse(j.fuse(foot,neck),stud)
    # A flat bottom represents the maximum material removed by the shop's drill point.
    tie=j.cut(tie,cylinder(x,y,45,2.70,91))
    washer=ring(x,y,154,5.,2.65,1.)
    nut=j.cut(hexagon(x,y,155,4/math.cos(math.pi/6),6.3),cylinder(x,y,154,2.5,9))
    return {"anchor":tie,"washer":washer,"locknut":nut}


def overlay(source,steel=False):
    changed={k:source[k] for k in CHANGED}
    added={}
    bore=2.25 if steel else 2.75
    for x,y in AXES:
        rail="cross-front" if x>0 else "cross-rear"
        changed["deck-main"]=j.cut(changed["deck-main"],cylinder(x,y,149,bore,6))
        changed[rail]=j.cut(changed[rail],cylinder(x,y,147,bore,4))
        changed[rail]=j.cut(changed[rail],cylinder(x,y,49,4.10,4))
        hardware=bodies(x,y) if steel else integral(x,y)
        added.update({"deck-tie-"+name(x,y)+"-"+k:v for k,v in hardware.items()})
    return {**{"tie-machined-"+k:v for k,v in changed.items()},**added}
