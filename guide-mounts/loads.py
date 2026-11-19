"""Screen the spring-supported backing under both issued attained pipe histories."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-guide"))
import guide

STATIONS=(687.,787.)
TOP=813.
E=69000.  # design assumption for aluminum parts in N/mm^2


def arm(z, pivot_span=14., ribbed=True):
    length=165+.46*(z-625)+14
    inner=150.5125+.038*max(z+6-770,0)+8.15
    tube_i=(12*30**3-10.4*28.4**3)/12
    flat_i=22*6**3/12
    # Section normal to Y. Opposed 22 x 2 returns share the 22 mm-deep, 6 mm-thick root plate.
    area=22*6 + 2*22*2
    neutral=(22*6*(-17)+2*22*2*(-31))/area
    rib_i=flat_i+22*6*(-17-neutral)**2+2*(2*22**3/12+22*2*(-31-neutral)**2)
    rib_length=209.5-(inner+15)
    flat_length=215.5-209.5
    rotation=(rib_length/(rib_i if ribbed else flat_i)+flat_length/flat_i)/E
    grip=56.
    bolt_k=200000*8.78/grip
    angular_bolt_k=bolt_k*pivot_span**2/2
    terms={"tubeBending":length**3/(3*E*tube_i),
           "rootBending":length**2*rotation,
           "rootTension":(rib_length/area+flat_length/(22*6))/E,
           "boltCouple":length**2/angular_bolt_k}
    return {"lengthMm":length,"termsMmPerN":terms,"totalMmPerN":sum(terms.values())}


def travel(z, force, lip=4., pivot_span=14., ribbed=True):
    slope=math.hypot(1,.46)
    width,thickness=40/slope,.8
    web=width*thickness
    legs=2*thickness*lip
    neutral=(web*thickness/2+legs*(thickness+lip/2))/(web+legs)
    inertia=width*thickness**3/12+web*(neutral-thickness/2)**2+2*thickness*lip**3/12+legs*(thickness+lip/2-neutral)**2
    lo,hi=STATIONS
    span=(hi-lo)*slope
    a=(z-lo)*slope
    b=span-a
    over=np.maximum(0,-np.minimum(a,b))
    spine=np.where(np.minimum(a,b)>=0,a**2*b**2/(3*E*inertia*span),
                   over**2*(span+over)/(3*E*inertia))
    supports=(b/span)**2*arm(lo,pivot_span,ribbed)["totalMmPerN"]+(a/span)**2*arm(hi,pivot_span,ribbed)["totalMmPerN"]
    return force*(spine+supports)


def contacts(trace):
    # Screen every stored pose with a conservatively large vertical cylinder bound.
    # The original solver uses 128 sectors and 4 radial cells on the open annular end.
    indices=np.where((trace[:,3]/1000+guide.feed.R+abs(trace[:,6])*guide.feed.HALF>=.625)&
                     (trace[:,3]/1000-guide.feed.R-abs(trace[:,6])*guide.feed.HALF<=.850))[0]
    rows=[]
    highest=-np.inf
    clipped=0
    for i in indices:
        row=trace[i]
        center=row[1:4]/1000
        axis=row[4:7]
        ring=guide.annular_points(axis,128,4)
        for side in (-1,1):
            arms=ring+side*guide.feed.HALF*axis
            x,y,z=(center+arms).T
            taper=np.where(z>.770,.038,0.)
            norm=np.sqrt(1+taper*taper)
            depth=(side*y-guide.feed.HALF+.0003-.038*np.maximum(z-.770,0))/norm
            active=(z>=.625)&(z<=.850)&(abs(x-guide.x_center(z)-.020)<.020)&(depth>0)
            if not active.any():
                continue
            nz=z[active]
            highest=max(highest,float(nz.max()*1000))
            clipped+=int(np.sum(nz>TOP/1000))
            normals=np.column_stack([np.zeros(len(nz)),side/norm[active],-taper[active]/norm[active]])
            normal_v=np.sum((row[7:10]+np.cross(row[10:13],arms[active]))*normals,axis=1)
            pressure=np.maximum(0,10000*depth[active]+200*normal_v)/len(ring)
            f=pressure.sum()
            if f>1e-8:
                rows.append((row[0],side,f,1000*(pressure@nz)/f))
    return np.asarray(rows),{"screenedStates":len(trace),"detailedStates":len(indices),
                            "highestPenetratingCellMm":highest,"penetratingCellsAboveCut":clipped}


def run(traces):
    source=json.loads((ROOT/"conditioned-match/elastic.json").read_bytes())
    trials={}
    for case in ("leftSmall","rightSmall"):
        r=next(r for r in source["cases"] if r["case"]==case and r["id"].endswith("machined") and "31.25" in r["id"])
        file=traces/(r["id"]+".npy")
        if hashlib.sha256(file.read_bytes()).hexdigest()!=r["trajectorySha256"]:
            raise ValueError("Unexpected pipe trajectory "+case)
        rows,report=contacts(np.load(file,mmap_mode="r"))
        report.update(sourceSha256=r["trajectorySha256"],peakForceN=float(rows[:,2].max()),layouts={})
        for name,lip,pivot,ribs in (("folded",4,14,True),("flat",0,14,True),
                                     ("halfCouple",4,7,True),("noRibs",4,14,False)):
            u=travel(rows[:,3],rows[:,2],lip,pivot,ribs)
            index=int(np.argmax(u))
            report["layouts"][name]={"peakTravelMm":float(u[index]),"atMs":float(rows[index,0]),
                "side":int(rows[index,1]),"forceN":float(rows[index,2]),"loadHeightMm":float(rows[index,3])}
        trials[case]=report
    result={"source":"issued rigid-guide machined-shelf trajectories","topMm":TOP,
            "arms":{str(z):arm(z) for z in STATIONS},"cases":trials,"measuredStiffness":None}
    (HERE/"loads.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf8",newline="\n")
    print(json.dumps(trials,indent=2))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("traces",type=Path)
    run(p.parse_args().traces)
