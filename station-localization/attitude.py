"""Bound a tilted and vertically displaced receiver using synchronized views of the four field datums."""
import math
import numpy as np
from pose import EYE,SECOND_EYE,HEIGHT,WIDTH,FOCAL,LANDING,RX,POINTS,axes

PIVOT_Z=937.15
PIVOT=np.array([RX,0.,PIVOT_Z])
PRIOR=np.array([15.,35.,3.,*np.radians([3.,.35,.35])])
# Coordinates of receiver datum in station X/Y/Z, then body heading, pitch and roll.


def rot(q):
    yaw,pitch,roll=q[3:]
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return (np.array([[1,0,0],[0,cr,sr],[0,-sr,cr]])@
            np.array([[cp,0,-sp],[0,1,0],[sp,0,cp]])@
            np.array([[cy,sy,0],[-sy,cy,0],[0,0,1]]))


def point_vector(p):
    return np.array([LANDING-p[0],-p[1],p[2]-PIVOT_Z])


def projected(q,eye):
    result=[]
    right,down,forward=map(np.asarray,axes(eye))
    change=rot(q)
    for p in POINTS:
        ray=PIVOT+change@(point_vector(p)+np.array([q[0],q[1],-q[2]]))-eye
        depth=forward@ray
        if depth<=0: raise ValueError("Behind image plane")
        result.append([WIDTH/2+FOCAL*(right@ray)/depth,HEIGHT/2+FOCAL*(down@ray)/depth])
    return result


def outer(marks,eyes,error_px,initial=PRIOR):
    """Interval enclosure about nominal pose; uses a bounded rotation remainder and a left inverse."""
    if len(eyes)!=len(marks) or not math.isfinite(error_px) or error_px<0:
        raise ValueError("Invalid camera set or uncertainty")
    if len(eyes)==2:
        baseline=np.subtract(eyes[1],eyes[0])
        if len(marks[0])!=4 or len(marks[1])!=4:
            raise ValueError("Four associated faces required")
        delta=math.sqrt(2)*error_px/FOCAL
        for uv_pair in zip(*marks,strict=True):
            directions=[]
            for e,(u,v) in zip(eyes,uv_pair,strict=True):
                r,d,f=map(np.asarray,axes(e))
                directions.append(f+(u-WIDTH/2)/FOCAL*r+(v-HEIGHT/2)/FOCAL*d)
            a,b=directions
            # Coplanarity of two sightlines to the same landmark is independent of robot attitude.
            residual=abs(baseline@np.cross(a,b))
            allowed=np.linalg.norm(baseline)*(delta*(np.linalg.norm(a)+np.linalg.norm(b))+delta**2)
            if residual>allowed+1e-8:return None
    rows=[];observations=[];geometry=[]
    for eye,pixels in zip(eyes,marks,strict=True):
        if len(pixels)!=4: raise ValueError("Four associated faces required")
        right,down,forward=map(np.asarray,axes(eye))
        for p,(u,v) in zip(POINTS,pixels,strict=True):
            if not (error_px<=u<WIDTH-error_px and error_px<=v<HEIGHT-error_px):
                raise ValueError("Feature cropped")
            w=point_vector(p)
            v0=PIVOT+w-eye
            for axis,a in ((right,(u-WIDTH/2)/FOCAL),(down,(v-HEIGHT/2)/FOCAL)):
                n=axis-a*forward
                # dR/dyaw, dR/dpitch, dR/droll for inverse rotations at zero.
                cols=([1,0,0],[0,1,0],[0,0,-1],-np.cross([0,0,1],w),-np.cross([0,1,0],w),-np.cross([1,0,0],w))
                rows.append([n@col for col in cols])
                observations.append(-n@v0)
                geometry.append((np.linalg.norm(w),np.linalg.norm(n),forward@v0))
    A=np.array(rows); W=np.linalg.pinv(A)
    if np.max(np.abs(W@A-np.eye(6)))>1e-8:raise ValueError("Rank-deficient view")
    center=W@observations
    initial=np.array(initial)
    extent=initial.copy()
    for _ in range(60):
        angle=sum(extent[3:])
        shift=np.linalg.norm(extent[:3])
        # Products of rotations are norm-preserving. Taylor's integral remainder for
        # the rotation product has norm <= angle^2/2. Rotation of uncertain translation
        # contributes <= angle*shift. Depth uses the whole current uncertainty box.
        b=np.array([error_px/FOCAL*(z+r*angle+shift)+nn*(.5*r*angle**2+shift*angle)
                    for r,nn,z in geometry])
        lo=np.maximum(-initial,center-np.abs(W)@b-1e-8)
        hi=np.minimum(initial,center+np.abs(W)@b+1e-8)
        if np.any(lo>hi):return None
        updated=np.minimum(extent,np.maximum(abs(hi),abs(lo)))
        if np.max(extent-updated)<1e-9:break
        extent=updated
    # Test for inconsistency orthogonal to the column space as well.
    obs=np.array(observations)
    if np.any(np.abs(obs-A@center)>np.abs(np.eye(len(obs))-A@W)@b+1e-7):
        return None
    conv=np.array([1.,1.,1.,180/math.pi,180/math.pi,180/math.pi])
    return (np.array([lo,hi]).T*conv[:,None]).tolist()


if __name__=="__main__":
    q=np.zeros(6)
    for es in ((EYE,),(EYE,SECOND_EYE)):
        marks=[projected(q,eye) for eye in es]
        for e in (.05,.1,.2,.4):
            r=outer(marks,es,e)
            print(len(es),e, [round(b,4) for _,b in r])
