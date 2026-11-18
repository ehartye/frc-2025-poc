"""Compare in-plane bearing-land compliance with a constant-strain plane-stress mesh."""
from pathlib import Path
import csv
import json
import numpy as np

HERE=Path(__file__).resolve().parent


def windows():
    with (HERE/"windows.csv").open(encoding="utf-8",newline="") as f:
        return [(float(r["minU"])-147,float(r["minV"])-205,float(r["maxU"])-147,
                 float(r["maxV"])-205,float(r["internalRadius"])) for r in csv.DictReader(f)]


def inside_window(x,y):
    inside=np.zeros(x.shape,dtype=bool)
    for x0,y0,x1,y1,r in windows():
        cx=np.clip(x,x0+r,x1-r)
        cy=np.clip(y,y0+r,y1-r)
        inside|=(x-cx)**2+(y-cy)**2<=r*r
    return inside


def compare(spacing,light):
    x=np.linspace(-147,175,int(np.ceil(322/spacing))+1)
    y=np.linspace(-205,205,int(np.ceil(410/spacing))+1)
    xx,yy=np.meshgrid(x,y)
    coords=np.column_stack((xx.ravel(),yy.ravel()))
    n=len(coords)
    nx=len(x)
    cells=np.arange(n).reshape(len(y),nx)[:-1,:-1].ravel()
    dx,dy=x[1]-x[0],y[1]-y[0]
    area=dx*dy/2
    E,nu,t=68300,.33,5
    D=E/(1-nu**2)*np.array([[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]])
    families=[]
    diagonal=np.zeros(n*2)
    for local in ((0,1,nx+1),(0,nx+1,nx)):
        tri=cells[:,None]+np.array(local)
        cent=coords[tri].mean(axis=1)
        if light:tri=tri[~inside_window(cent[:,0],cent[:,1])]
        vertex=coords[tri[0]]
        grads=np.linalg.inv(np.column_stack((np.ones(3),vertex)))[1:,:]
        B=np.zeros((3,6))
        B[0,0::2],B[1,1::2]=grads
        B[2,0::2],B[2,1::2]=grads[1],grads[0]
        ke=t*area*B.T@D@B
        dof=(tri[:,:,None]*2+np.array([0,1])).reshape(-1,6)
        diagonal+=np.bincount(dof.ravel(),np.broadcast_to(ke.diagonal(),dof.shape).ravel(),minlength=n*2)
        families.append((dof,ke))
    fixed=np.repeat((coords[:,0]>=-97)&(coords[:,0]<=-77),2)|(diagonal<1e-9)
    # A force at one forward rail region loads both plate bending in its own plane
    # and shear across the pocketed intermediate webs. Out-of-plane bending is separate.
    pad=(coords[:,0]>=136)&(coords[:,0]<=160)&(coords[:,1]>=162)&(coords[:,1]<=198)
    if not pad.any():raise ValueError("No nodes in load pad")
    def multiply(a):
        result=np.zeros_like(a)
        for dof,ke in families:
            result+=np.bincount(dof.ravel(),(a[dof]@ke.T).ravel(),minlength=len(a))
        result[fixed]=0
        return result
    result={"spacingMm":spacing,"analyzedAreaMm2":area*sum(len(dof) for dof,ke in families)}
    for index,direction in enumerate(("driveX","sideY")):
        load=np.zeros(2*n)
        load[index::2][pad]=1000/pad.sum()
        inv=np.zeros_like(load)
        inv[~fixed]=1/diagonal[~fixed]
        u=np.zeros_like(load)
        residual=load.copy()
        z=inv*residual
        vec=z.copy()
        rz=residual@z
        tolerance=np.linalg.norm(load)*1e-8
        for i in range(3500):
            kp=multiply(vec)
            alpha=rz/(vec@kp)
            u+=alpha*vec
            residual-=alpha*kp
            if np.linalg.norm(residual)<tolerance:break
            z=inv*residual
            next_rz=residual@z
            vec=z+(next_rz/rz)*vec
            rz=next_rz
        else:raise ValueError("Unconverged plane-stress solve")
        result[direction]={"loadN":1000,"meanPadDisplacementAlongLoadMm":float(np.mean(u[index::2][pad])),
                           "maxDisplacementMm":float(np.max(np.hypot(u[0::2],u[1::2]))),
                           "relativeForceResidual":float(np.linalg.norm(residual)/np.linalg.norm(load))}
    return result


if __name__=="__main__":
    result={key:[compare(spacing,light) for spacing in (6.,4.,3.)]
            for key,light in (("blank",False),("windowed",True))}
    (HERE/"panel.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8",newline="\n")
