"""Rectified views of the four outer holes in the lower station panel."""
import itertools
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"station-approach"))
from station_trial import project,FOCAL_PX,PITCH

# GE-25008 is 6.35 mm thick. The paired circular openings form a 717.55 x 520.70 mm rectangle.
EYES=(37.5,-37.5)
APERTURES=json.loads(Path(__file__).with_name("native.json").read_bytes())["frontWireCenterYzAndSizeMm"]
outer_y=max(abs(row[0]) for row in APERTURES)
outer_circles=sorted(((y,z) for y,z,w,h in APERTURES if abs(y)==outer_y and abs(w-h)<.001),
                     key=lambda p:(-p[1],p[0]))
if len(outer_circles)!=4:raise ValueError("Four outer circular panel wires required")
CORNERS=np.array([[0,y,z] for y,z in outer_circles])
CENTERS=CORNERS.copy()
CENTERS[:,0]=-3.175
RADIUS=7.9375
ANGLES=np.arange(32)*2*math.pi/32
RIM=np.c_[np.zeros(len(ANGLES)),RADIUS*np.cos(ANGLES),RADIUS*np.sin(ANGLES)]


def components(img):
    _,_,boxes,centers=cv2.connectedComponentsWithStats(np.uint8(img<60),8)
    return [p for (x,y,w,h,area),p in zip(boxes[1:],centers[1:]) if
            x>1 and y>1 and x+w<639 and y+h<479 and 2<=w<=20 and 2<=h<=18 and
            area>=4 and .35<w/h<3.5 and area>=.45*w*h]


def track(images,prior,bias):
    expected=pixels(prior)+bias
    marks=np.full_like(expected,np.nan)
    for eye,image in enumerate(images):
        found=np.array(components(image)).reshape(-1,2)
        for j,p in enumerate(expected[eye]):
            near=found[np.linalg.norm(found-p,axis=1)<9]
            if len(near)==1:marks[eye,j]=near[0]
    valid=np.isfinite(marks[:,:,0])
    spread=valid.any(axis=0)
    if spread.sum()<2 or not spread[:2].any() or not spread[2:].any() or min(valid.sum(axis=1))<1 or not valid.all(axis=0).any():
        return None
    return marks


def acquire(images):
    marks=np.full((2,4,2),np.nan)
    left,right=map(components,images)
    for l in left:
        matches=[]
        for r in right:
            disparity=l[0]-r[0]
            if abs(l[1]-r[1])>1.5 or disparity<11 or disparity>66:continue
            depth=FOCAL_PX*75/disparity
            z=970+depth*(-math.sin(PITCH)+(240-l[1])/FOCAL_PX*math.cos(PITCH))
            # Under the declared staging prior both outer columns remain on opposite
            # sides of image center. Reject uncertain column or shared matches.
            index=0 if abs(z-CORNERS[0,2])<38 else 2 if abs(z-CORNERS[-1,2])<38 else None
            if index is None or abs((l[0]+r[0])/2-320)<12:continue
            matches.append((index+int((l[0]+r[0])/2>320),r))
        if len(matches)>1:return None
        if matches:
            j,r=matches[0]
            if np.isfinite(marks[0,j,0]):return None
            marks[:,j]=[l,r]
    if np.isfinite(marks[0,:,0]).sum()<3:return None
    return marks


def observations(images):
    """Associate two broad rows of holes. Multiple candidate rectangles suppress the measurement."""
    def side(img):
        usable=components(img)
        rows=[(a,b) for a,b in itertools.permutations(usable,2) if b[0]-a[0]>80 and
              abs(b[1]-a[1])<.15*(b[0]-a[0])]
        rectangles=[]
        for u,l in itertools.permutations(rows,2):
            top_width=u[1][0]-u[0][0]
            bottom_width=l[1][0]-l[0][0]
            height=np.mean([l[i][1]-u[i][1] for i in (0,1)])
            if (.40*top_width<height<.92*top_width and .57*top_width<bottom_width<1.03*top_width
                    and max(abs(l[i][0]-u[i][0]) for i in (0,1))<.30*top_width):
                rectangles.append(np.array([*u,*l]))
        return rectangles
    a,b=map(side,images)
    pairs=[(l,r) for l,r in itertools.product(a,b) if np.max(abs(l[:,1]-r[:,1]))<2 and
           np.all((l[:,0]-r[:,0]>5)&(l[:,0]-r[:,0]<70))]
    return np.array(pairs[0]) if len(pairs)==1 else None


def pixels(q):
    return np.array([project(CENTERS,(444.525+q[0],q[1],q[2]),eye) for eye in EYES])


def fit(marks,bias,prior):
    q=np.asarray(prior,dtype=float).copy()
    valid=np.isfinite(marks.ravel())
    for _ in range(10):
        estimate=pixels(q)
        residual=(estimate+bias-marks).ravel()
        steps=np.diag([.01,.01,.00001])
        jac=np.stack([((pixels(q+d)-estimate)/d[i]).ravel() for i,d in enumerate(steps)],axis=1)
        matrix=jac[valid]
        values=residual[valid]
        update=np.linalg.lstsq(matrix,values,rcond=None)[0]
        q-=np.clip(update,[-400,-190,-.2],[400,190,.2])
        if np.max(abs(update))<.000001:break
    return q,float(np.nanmax(abs(pixels(q)+bias-marks)))


def render(q,depth_images,contrast=True):
    """Two finite aperture faces, sampled at 4x; the held native body can cover either eye."""
    result=[]
    for i,eye in enumerate(EYES):
        img=np.full((480,640),104,np.uint8)
        for center in CORNERS:
            front=project(center+RIM,(444.525+q[0],q[1],q[2]),eye)
            back=project(center+RIM+[-6.35,0,0],(444.525+q[0],q[1],q[2]),eye)
            if not np.isfinite(front).all() or not np.isfinite(back).all():
                continue
            area,border=cv2.intersectConvexConvex(front.astype(np.float32),back.astype(np.float32))
            if not area or border is None:
                continue
            border=border[:,0,:]
            x,y=np.maximum(0,np.floor(border.min(axis=0)).astype(int)-2)
            x1,y1=np.minimum([640,480],np.ceil(border.max(axis=0)).astype(int)+2)
            if x1<=x or y1<=y:continue
            supersample=np.full(((y1-y)*4,(x1-x)*4),104,np.uint8)
            cv2.fillConvexPoly(supersample,np.rint((border-[x,y])*4).astype(np.int32),31 if contrast else 104)
            img[y:y1,x:x1]=cv2.resize(supersample,(x1-x,y1-y),interpolation=cv2.INTER_AREA)
        # Compare against the same fixed robot raster at every new camera pose.
        v,u=np.ogrid[:480,:640]
        xf=math.cos(PITCH)+(240-v)/FOCAL_PX*math.sin(PITCH)
        yl=-(u-320)/FOCAL_PX
        normal_ray=xf*math.cos(q[2])-yl*math.sin(q[2])
        origin=444.525+q[0]+315*math.cos(q[2])+eye*math.sin(q[2])
        plane=(origin+3.175)/np.maximum(.01,normal_ray)
        img[depth_images[i]<plane]=104
        result.append(img)
    return result


def referenced(path):
    a=np.load(path/"first-fixed.npz")
    b=np.load(path/"return-fixed.npz")
    images=lambda scene:[scene[k] for k in ("left","right")]
    nominal=observations(images(a))
    second=observations(images(b))
    if nominal is None or second is None:raise ValueError("Native image did not yield a unique rectangle")
    bias=nominal-pixels([0,0,0])
    body=np.load(path.parent/"station-overlay"/"stowed-robot-fixed.npz")
    masks=[body[k+"_z"] for k in ("left","right")]
    model=observations(render([0,0,0],masks))
    if model is None:raise ValueError("Native body masks hide the synthetic datum")
    return masks,bias,model-pixels([0,0,0]),second
