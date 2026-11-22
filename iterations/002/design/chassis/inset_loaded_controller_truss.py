"""Recover actual equipment poses from the contact solution and bound the adjacent truss stroke."""
from pathlib import Path
import hashlib, json, sys
import numpy as np

from inset_electrical_package import HERE, CAD, box_corners
from inset_electronics_deck import definition, plate
from inset_pan_truss import path_gap
from direct_saddle_inertia import construct


def foot(nodes,x,y):
    """Cubic transverse Euler–Bernoulli displacement, linear axial displacement."""
    sy=np.sign(y); ya,yb=115.3*sy,242.*sy
    a,b=(next(np.array(v["dX_dY_yawRad"]) for v in nodes if np.linalg.norm(np.array(v["atMm"])-[x,k])<1e-5) for k in (ya,yb))
    if x==250.:
        from inset_key_contact import member, section
        yc=231.5*sy
        k1,_=member(np.array([x,ya]),np.array([x,yc]),section(3.,False))
        k2,_=member(np.array([x,yc]),np.array([x,yb]),section(3.,True))
        mid=-np.linalg.solve(k1[3:,3:]+k2[:3,:3],k1[3:,:3]@a+k2[:3,3:]@b)
        if abs(y)<=abs(yc):
            b,yb=mid,yc
        else:
            a,ya=mid,yc
    h=yb-ya;t=(y-ya)/h
    dx=(1-3*t*t+2*t**3)*a[0]+(3*t*t-2*t**3)*b[0]-h*((t-2*t*t+t**3)*a[2]+(-t*t+t**3)*b[2])
    return np.array([dx,(1-t)*a[1]+t*b[1],0.])


def truss_posture(native,hosts,stress_limit):
    landed=[]
    for side,layout in native["layout"].items():
        nodes=np.array(layout["nodes"]);edges=layout["edges"];B=np.zeros((18,6));lengths=[]
        floor=np.stack([foot(hosts,n[0],n[1]) for n in nodes[2:]])
        for i,(a,b) in enumerate(edges):
            dv=nodes[b]-nodes[a];lengths.append(np.linalg.norm(dv));u=dv/lengths[-1]
            B[a*3:a*3+3,i]=u;B[b*3:b*3+3,i]=-u
        inv=np.linalg.inv(B[:6].T)
        top=(-inv@B[6:].T@floor.ravel()).reshape(2,3)
        # Independently signed member length changes, with no forced concurrency.
        # Applies only while the six bars obey the working axial stress bound.
        boxes=(abs(inv)@(np.array(lengths)*stress_limit/68900.)).reshape(2,3)
        uncertainty=np.r_[np.linalg.norm(boxes,axis=1),np.zeros(4)]
        deformed=nodes+np.vstack([top,floor])
        for i,j in edges:
            landed.append((deformed[i],deformed[j],uncertainty[i],uncertainty[j]))
    return landed


def run(plugin,service=False):
    sys.path.insert(0,str(plugin/"scripts"))
    from showcase_contacts import load_solids,box_gap
    from mechanical_motion import bounds
    m=construct(plugin,12.7,True)
    src=HERE/("inset-brace-access-key-contact.json" if service else "inset-truss-key-contact.json");results=json.loads(src.read_bytes())
    pack=HERE/"inset-controller-truss.json";static=json.loads(pack.read_bytes())
    frame=HERE/"inset-pan-truss.json";bars=json.loads(frame.read_bytes())
    if not m["authority"]==results["authority"]==static["authority"]==bars["authority"]:raise ValueError("Unbound issue")
    whole={}
    for role,file in (("controller","REV-11-2158.STEP"),("bracket","REV-21-2151.STEP")):
        shapes,unused=load_solids(CAD/file)
        if unused:raise ValueError("Unexpected reference surface")
        whole[role]={n:np.array(box_corners(bounds(s))) for n,s in shapes.items()}
    devices=static["records"]["178.0"]["instances"];deck_geometry="brace-access" if service else True;models=definition(True,deck_geometry)
    ledger=json.loads((HERE/"inset-stagger-inertia-ledger.json").read_bytes())
    xy={int(k):np.array(v) for k,v in ledger["fixedSupportDOFs"].items()}
    trials=[];cached={}
    for variant,study in results["outcomes"].items():
        if not study["decks"]:continue
        k,grid=map(int,variant.split()[-2:])
        if grid not in cached:
            _,cached[grid]=plate(m,xy,k,grid,thickness=1.8796,keyed=True,contact=True,truss=deck_geometry,recover=True)
        for name,c in study["cases"].items():
            u=np.zeros(len(m["base"]));hosts=c["hostPoses"]
            for node in hosts:u[node["dof"]:node["dof"]+3]=node["dX_dY_yawRad"]
            truss=truss_posture(bars,hosts,120.)
            chunks=[]
            for i,(a,b,ea,eb) in enumerate(truss):
                for j in range(8):
                    t=j/8;s=(j+1)/8
                    start=(1-t)*a+t*b; end=(1-s)*a+s*b
                    r=19.05/np.sqrt(2)+max((1-t)*ea+t*eb,(1-s)*ea+s*eb)
                    chunks.append((i,start,end,r,np.r_[np.minimum(start,end)-r,np.maximum(start,end)+r]))
            minima=[];fitted=[]
            for side,plate_map in enumerate(cached[grid]):
                sy=plate_map["sign"]
                seated=plate_map["hostMap"]@u-np.array(c["physicalKeyContact"]["hostMinusSheetLocalMm"][side])
                points=plate_map["activePoints"]*[1,sy]
                disp=(plate_map["nodalFromSupport"]@seated)*[1,sy]
                for device in devices:
                    d=device["name"];holes=[np.array([x,sy*y]) for nm,x,y,_ in models[sy]["holes"] if nm==d]
                    if not holes:continue
                    if len(holes)!=3:raise ValueError(d)
                    near=np.unique(np.concatenate([np.argsort(np.linalg.norm(points-a,axis=1))[:4] for a in holes]))
                    reference=points[near]; deformed=reference+disp[near]
                    origin=np.mean(reference,axis=0);target=np.mean(deformed,axis=0)
                    a,_,b=np.linalg.svd((reference-origin).T@(deformed-target));rot=b.T@a.T
                    if np.linalg.det(rot)<0:raise ValueError("Unexpected reflection")
                    err=np.max(np.linalg.norm((reference-origin)@rot.T+target-deformed,axis=1))
                    fitted.append(dict(name=d,maxRigidResidualMm=err,localRotationRad=np.arctan2(rot[1,0],rot[0,0]),shiftAtCentroidMm=(target-origin).tolist()))
                    for role,matrix in device["worldMatricesMm"].items():
                        tf=np.array(matrix)
                        for part,verts in whole[role].items():
                            world=verts@tf[:,:3].T+tf[:,3];world[:,:2]=(world[:,:2]-origin)@rot.T+target
                            box=np.r_[world.min(axis=0)-err,world.max(axis=0)+err]
                            best=15.;at=None
                            for i,p,q,r,bb in chunks:
                                if box_gap(bb,box)>best:continue
                                g=path_gap(p,q,box,r)
                                if g<best:best,at=g,i
                            if at is not None:minima.append(dict(name=d,role=role,body=part,bar=at,gapBoundMm=best))
            if len(fitted)!=9 or not minima:raise ValueError("Incomplete placed controller fit")
            trials.append(dict(variant=variant,case=name,worst=min(minima,key=lambda q:q["gapBoundMm"]),devices=fitted))
    return dict(authority=m["authority"],inputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (src,pack,frame)},
                trials=trials,minimum=min((r["worst"]["gapBoundMm"] for r in trials)),
                scope="Same 24 signed planar frame and sheet equilibria, including key slip and all nine controller+carrier source envelopes. Brackets follow a rigid fit to nearby sheet material around three holes. Residual is included on each envelope. Tube centerlines recover the rigid truss response to in-plane foot movements; a conservative envelope adds independent member strains up to 120 MPa to both upper joints. Does not cover out-of-plane floor roll, pan impact beyond this working limit, local hole-edge/contact pressure, unqualified wiring, tool use or service during impact. A nonpositive lower bound requires closer analysis.")

if __name__=="__main__":
    service="--brace-access" in sys.argv
    v=run(Path(sys.argv[1]),service);(HERE/("inset-service-loaded-controller-truss.json" if service else "inset-loaded-controller-truss.json")).write_text(json.dumps(v,indent=1)+"\n")
    print(v["minimum"],min(v["trials"],key=lambda q:q["worst"]["gapBoundMm"])["worst"])
