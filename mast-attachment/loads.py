"""Resolve simultaneous frame wrenches on the actual bolt groups and deck-height splice section."""
import argparse
import hashlib
from itertools import product
from pathlib import Path

import numpy as np
from mast import HERE,MAST,RAIL,ROOT,j,plate

REFERENCE=np.array([-45.,0.,154.])
BOLT_AREA=20.1  # M6 coarse tensile stress area; threads may lie in a shear plane.
PRELOAD=1500.    # N, design exercise only, not a tightening instruction.


def transform(force,moment,origin,destination):
    return moment+np.cross((origin-destination)/1000,force)


def group(force,moment,origin,bolts,side):
    coords=np.array(bolts,dtype=float)
    xy=coords-coords.mean(axis=0)
    x,z=xy.T
    center=np.array([coords[:,0].mean(),side*235.,coords[:,1].mean()])
    mx,my,mz=(1000*transform(force,moment,origin,center)).T
    fx,fy,fz=force.T
    n=len(coords)
    normal=np.vstack([fy,mx,mz]).T@np.linalg.pinv(np.stack([np.ones(n),-z,x])).T
    inertia=np.sum(x*x+z*z)
    sx=fx[:,None]/n+my[:,None]*z/inertia
    sz=fz[:,None]/n-my[:,None]*x/inertia
    shear=np.hypot(sx,sz)
    equivalent=np.hypot((PRELOAD+abs(normal))/BOLT_AREA,np.sqrt(3)*shear/BOLT_AREA)
    return {"axialN":float(np.max(abs(normal))),"shearN":float(np.max(shear)),
            "threadVonMisesMPa":float(np.max(equivalent)),
            "spliceBoreBearingMPa":float(np.max(shear)/(6*3)),
            "single2mmMastWallOnSleeveMPa":float(np.max(shear)/(2*10))}


def cross_section(side,z_mm):
    # Determine its material intervals by a 0.20 mm grid conservatively stripping
    # edge cells, then use the section's resulting area and principal moments.
    s=plate(side)
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_IN
    from OCP.gp import gp_Pnt
    classifier=BRepClass3d_SolidClassifier(s)
    xs=np.arange(-135.,68.,.2)
    def inside(x):
        classifier.Perform(gp_Pnt(x,side*236.5,z_mm),1e-7)
        return classifier.State()==TopAbs_IN
    included=np.array([inside(x-.1) and inside(x+.1) for x in xs])
    cells=xs[included]
    width=len(cells)*.2
    mean=float(cells.mean())
    return {"zMm":z_mm,"widthMm":width,"centroidXMm":mean,"areaMm2":width*3,
            "weakZmm3":width*3**2/6,
            "strongZmm3":float(np.sum((cells-mean)**2*.2*3+.2**3*3/12)/max(abs(xs.min()-mean),abs(xs.max()-mean)))}


def report(t,force,moment,origin,side,section):
    g={}
    for kind,bolts in (("mast",MAST),("rail",RAIL)):
        g[kind]={}
        for name,missing in [("intact",None)]+[("omit"+str(i),i) for i in range(4)]:
            g[kind][name]=group(force,moment,origin,[p for i,p in enumerate(bolts) if i!=missing],side)
    governing={}
    span=[]
    for cut in section:
        at=np.array([cut["centroidXMm"],side*236.5,cut["zMm"]])
        m=transform(force,moment,origin,at)
        span.append(abs(m))
        stress=abs(force[:,2])/cut["areaMm2"]+abs(m[:,0])*1000/cut["weakZmm3"]+abs(m[:,1])*1000/cut["strongZmm3"]
        idx=int(stress.argmax())
        if stress[idx]>governing.get("sectionMPa",-1):
            governing={"sectionMPa":float(stress[idx]),"sectionZmm":cut["zMm"],"witnessTimeS":float(t[idx]),
                        "coincidentForceN":force[idx].tolist(),"coincidentSectionNm":m[idx].tolist()}
    envelope=np.max(span,axis=0)
    # The outer straight plate edges narrow monotonically between the low and
    # high fastener bands. Include the kink at Z168 and conservatively pair the
    # greatest station moments with the weakest section at the same load clock.
    bounded=abs(force[:,2])/min(c["areaMm2"] for c in section)
    bounded+=envelope[:,0]*1000/min(c["weakZmm3"] for c in section)
    bounded+=envelope[:,1]*1000/min(c["strongZmm3"] for c in section)
    return {"group":g,"spanGrossUpperMPa":float(bounded.max()),**governing}


def states(trace):
    t=trace[:,0]
    force=trace[:,3:6]
    moment=trace[:,9:12]
    # Transfer each actually calculated signed vertical rail reaction at fixed-height
    # mast spacing. Share the residual global couple and in-plane load equally.
    split=[force*.5,force*.5]
    for i in (0,1):split[i][:,2]=-trace[:,i+1]
    if not np.allclose(split[0]+split[1],force,atol=1e-4):raise ValueError("Vertical force does not close")
    p=[np.array([-45,222.5,154]),np.array([-45,-222.5,154])]
    transported=sum(np.cross((location-REFERENCE)/1000,f) for location,f in zip(p,split))
    for i,s in enumerate((1,-1)):
        yield "paired"+("Left" if s==1 else "Right"),s,t,split[i],(moment-transported)/2,p[i]
        yield "single"+("Left" if s==1 else "Right"),s,t,force,moment,REFERENCE


def run(csv_dir,registration):
    data=j.read(registration)
    sections={s:[cross_section(s,z) for z in (145,146,149,152,155,158,159.5)] for s in (1,-1)}
    rows={}
    for case in data["cases"]:
        path=csv_dir/case["file"]
        if hashlib.sha256(path.read_bytes()).hexdigest()!=case["sha256"]:raise ValueError("Different force sample trace")
        trace=np.loadtxt(path,skiprows=1,delimiter=",")
        rows[case["case"]]={k:report(t,f,m,p,s,sections[s]) for k,s,t,f,m,p in states(trace)}
    bound=np.array(list(product((-300.,300.),(-250.,250.),(-1900.,1900.),(-50.,50.),(-431.,431.),(-50.,50.))))
    adverse={str(s):report(np.arange(len(bound)),bound[:,:3],bound[:,3:],REFERENCE,s,sections[s]) for s in (1,-1)}
    j.write(HERE/"load.json",{"transferSha256":j.digest(registration),"sourceCsvSha256":{r["case"]:r["sha256"] for r in data["cases"]},
             "sections":sections,"cases":rows,"independentAdverseSingleSide":adverse,"assumedPreloadN":PRELOAD,
             "certifiedYieldAndFastenerProofMPa":[400.,580.],"measuredAllowable":None})


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv_dir",type=Path)
    p.add_argument("registration",type=Path)
    a=p.parse_args()
    run(a.csv_dir,a.registration)
