"""Identify a finite overlap on the proposed steering-motor removal path if there is one."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
from inset_stock_check import HERE,form
from inset_v11_screw_tip import transform,read

def run(scripts,src):
    sys.path.insert(0,str(scripts))
    from fusion_authority import register_path,verify
    from mechanical_motion import bounds,distance,common
    from showcase_contacts import load_solids,box_gap
    reg=register_path(HERE.parents[3]);authority=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    k=read("inset-oem-steering-key.json");d=HERE.parent/"private-inputs/matched-sds/corner.step"
    if authority!=k["authority"] or k["motorSHA256"]!=hashlib.sha256(src.read_bytes()).hexdigest() or k["donorStep"]!=hashlib.sha256(d.read_bytes()).hexdigest():raise ValueError("Changed source")
    stock=read("inset-stock-check.json");shop={r["name"]:r for r in read("inset-chassis-shop.json")["articles"]}
    donor=load_solids(d)[0]
    skip={"Motor, NEO [1.1.1.36.1]","Gear, 20DP, 14T, 8mm REV1 [1.1.1.15.1]","Machine Key, 2 X 2 X 12mm [1.1.1.39.1]","Retaining Ring, 8mm, 98430A132_98430A132 [1.1.1.16.1]","Shim, 8mm ID X 14mm OD X .5mm Thick, 98055A115_98055A115 [1.1.1.14.1]"}
    skip.update(r["case"]["screw"] for r in read("inset-corner-local-fits.json")["motorCases"] if r["corner"]=="FL" and ".36.1]" in r["case"]["host"])
    if len(skip)!=8 or skip-set(donor):raise ValueError("Unidentified service order")
    original=next(iter(load_solids(src)[0].values()))
    R=np.array(k["referenceShaftPoseMm"])@np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
    placements=read("inset-matched-corners.json")["newWorldMm"]
    boxes={n:bounds(s) for n,s in donor.items() if n not in skip};formed={}
    cases=[]
    for shift in [0,5,20,40,65]:
        tr=np.eye(4);tr[2,3]=-shift
        q=transform(original,R@tr);b=bounds(q);internal=[]
        for n,bb in boxes.items():
            if box_gap(bb,b)>1.e-4:continue
            # Skip expensive common calculations for known disjoint surfaces.
            if distance(q,donor[n])>1.e-5:continue
            v=[common(q,donor[n]),common(donor[n],q)]
            if max(v)>.0001:internal.append(dict(part=n,commonMm3=v))
        contacts={}
        for corner,W in placements.items():
            world=transform(q,np.asarray(W));bw=bounds(world);contacts[corner]=[]
            for n,r in stock["parts"].items():
                if box_gap(bw,r["bounds"])>.001:continue
                if n not in formed:
                    shape=form(shop[n]);formed[n]=transform(shape,np.asarray(shop[n]["matrixMm"])) if "matrixMm" in shop[n] else shape
                shape=formed[n]
                if distance(world,shape)>1.e-5:continue
                v=[common(world,shape),common(shape,world)]
                if max(v)>.0001:contacts[corner].append(dict(part=n,commonMm3=v))
        cases.append(dict(downwardMm=shift,commonInternal=internal,frameHits=contacts))
    return dict(authority=authority,motor=k["motorSHA256"],donor=k["donorStep"],priorRemovals=sorted(skip),rows=cases,
                scope="Straight downward withdrawal samples on original purchased motor BRep at +90 degree casing index. Assumes disconnection and prior extraction of the ring, pinion, key, shim and three motor screws. A positive blocks this staged route. Sampled zeros cannot certify continuous withdrawal. Lead protection, rotor phase and prerequisites are separate.")
if __name__=="__main__":
    r=run(*map(Path,sys.argv[1:]));(HERE/"inset-v11-motor-withdrawal.json").write_text(json.dumps(r,indent=1)+"\n")
