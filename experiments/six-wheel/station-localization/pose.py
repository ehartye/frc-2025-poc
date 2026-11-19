"""Station observation model. All distances mm, rotations degrees, image coordinates pixels."""
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PREVIOUS=json.loads((ROOT/"station-supply/compensated-dock.json").read_bytes())
LANDING=PREVIOUS["free_flight"]["landing_station_x_mm"]
CENTER=434.525+40
RX=CENTER-LANDING
WIDTH,HEIGHT=1456,1088
FOCAL=6/.00345
EYE=(-43,-222.5,1088)
SECOND_EYE=(30,222.5,1088)
TARGET_X,TARGET_Z=11.0744,1485.9
POINTS=tuple((TARGET_X,y,TARGET_Z+z) for z in (-117.475,117.475) for y in (-117.475,117.475))


def dot(a,b):
    return sum(x*y for x,y in zip(a,b))


def unit(a):
    n=math.sqrt(dot(a,a))
    return tuple(x/n for x in a)


def cross(a,b):
    x,y,z=a; p,q,r=b
    return (y*r-z*q,z*p-x*r,x*q-y*p)


def local(point,normal=0,side=0,yaw=0):
    x,y,z=point
    a=math.radians(yaw)
    c,s=math.cos(a),math.sin(a)
    x,y=x-LANDING-normal,y-side
    return RX-c*x-s*y,s*x-c*y,z


def axes(eye):
    target=local((TARGET_X,0,TARGET_Z))
    forward=unit(tuple(t-v for t,v in zip(target,eye)))
    right=unit(cross(forward,(0,0,1)))
    return right,cross(forward,right),forward


def image(point,pose,eye=EYE):
    v=tuple(p-e for p,e in zip(local(point,*pose),eye))
    right,down,forward=axes(eye)
    depth=dot(v,forward)
    if depth<=0: raise ValueError("Landmark behind camera")
    return WIDTH/2+FOCAL*dot(v,right)/depth,HEIGHT/2+FOCAL*dot(v,down)/depth
