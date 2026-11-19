"""Certify continuous straight service legs against the stationary shop geometry."""
import argparse
from pathlib import Path
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from mounts import HERE, OUTER, POSITIONS, SPLIT, baseline, shop


def moved(shape, vector):
    return shop.cad.translated(shape,list(vector))


def shift_box(bounds, v):
    return tuple(b+v[i%3] for i,b in enumerate(bounds))


def certify(shape, obstacles, start, end):
    """Bound each boundary-face sweep. Distance is 1-Lipschitz under translation."""
    delta=tuple(b-a for a,b in zip(start,end))
    length=sum(x*x for x in delta)**.5
    obs={k:(v,shop.cad.bounds(v)) for k,v in obstacles.items()}
    queries=0
    candidates=0
    limit=.02
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    while faces.More():
        face=TopoDS.Face(faces.Current())
        faces.Next()
        bounds=shop.cad.bounds(face)
        first,last=shift_box(bounds,start),shift_box(bounds,end)
        union=[min(a,b) if i<3 else max(a,b) for i,(a,b) in enumerate(zip(first,last))]
        for name,(part,box) in obs.items():
            if shop.lower_bound(union,box)>limit+.001:
                continue
            candidates+=1
            intervals=[(0.,1.)]
            while intervals:
                lo,hi=intervals.pop()
                mid=(lo+hi)/2
                translation=tuple(a+mid*d for a,d in zip(start,delta))
                distance=shop.distance(moved(face,translation),part)
                queries+=1
                if distance>(hi-lo)*length/2+limit+.001:
                    continue
                if distance<limit or (hi-lo)*length<.01:
                    return {"clear":False,"witness":name,"atMm":translation,
                            "faceDistanceMm":distance,"queries":queries}
                intervals.extend(((lo,mid),(mid,hi)))
    return {"clear":True,"strictLowerMm":limit,"closeFacePairs":candidates,"queries":queries}


def run(artifacts,out):
    scene,_=baseline(artifacts)
    fit=shop.read(out/"fit.json")
    step=out/"supported-covers.step"
    if shop.digest(step)!=fit["stepSha256"]:
        raise ValueError("Wrong support revision")
    kit=shop.cad.read_named(step,[{"id":k} for k in fit["volumeMm3"]])
    assembly={**scene,**kit}
    stages={}
    for side,sy in (("L",1),("R",-1)):
        def stage(tag,shape,rest,a,b):
            result=certify(shape,rest,a,b)
            stages[f"{side}/{tag}"]=result
            print(side,tag,result,flush=True)
            return result["clear"]
        lower,upper=(kit[f"tray-{g}-{side}"] for g in ("lower","upper"))
        # The shells share a trim plane. Opposite shell material lies in the
        # opposite closed half-space during the separating vertical move.
        if abs(shop.cad.bounds(lower)[5]-SPLIT)>.001 or abs(shop.cad.bounds(upper)[2]-SPLIT)>.001:
            raise ValueError("Unexpected split-plane stock")
        rest={k:v for k,v in assembly.items() if not (k.endswith("-"+side)
              and (k.startswith("tray-") or k.startswith(("face-screw-","retainer-"))))}
        for x,z in POSITIONS:
            if z==SPLIT:
                continue
            tag=f"{x:g}_{z:g}-{side}"
            # Compact L-key reservation includes a 33 mm lever, full rotation and
            # withdrawal by 10 mm. The selected fastener itself is removed here.
            key=shop.fuse(shop.rod(x,sy,OUTER+3.5,OUTER+29,z,1.6),
                          shop.rod(x,sy,OUTER+17,OUTER+30,z,34.5))
            tool=shop.clearance(key,{k:v for k,v in assembly.items() if k!="face-screw-"+tag})
            stages[f"{side}/key-{x:g}-{z:g}"]=shop.prove(tool)
        upper_bolts={k:v for k,v in kit.items() if k.endswith("_88-"+side)
                     and k.startswith(("retainer-","face-screw-"))}
        stage("lower-down",lower,{**rest,**upper_bolts},(0,0,0),(0,0,-180))
        # Keep the upper shell fastened while following the earlier supported wheel
        # withdrawal. The screw solids here already enclose all wheel clock angles.
        retained={k:v for k,v in kit.items() if k.endswith("-"+side) and not
                  (k.startswith("drive-rail-") or k=="tray-lower-"+side or
                   (k.endswith("_64-"+side) and k.startswith(("retainer-","face-screw-"))))}
        for station,index in (("front",1),("rear",1),("center",2)):
            keys=[f"{prefix}-{station}-{side}" for prefix in ("wheel","hub","wheel-hardware-head","wheel-hardware-thread")]
            keys.append(f"spacer-{station}-{index}-{side}")
            for key in keys:
                stage("wheel-drop/"+key,scene[key],retained,(0,0,0),(0,0,-180))
            axle=f"axle-{station}-{side}"
            stage("shaft-out/"+station,scene[axle],retained,(0,0,0),(0,sy*180,0))
        stage("upper-lift",upper,rest,(0,0,0),(0,0,14))
        # Follow the prior wheel service order on a stand. Keep carriers, their
        # keepers, all six posts, both chains and their sprockets installed.
        service_parts={f"{prefix}-{station}-{side}" for station in ("front","rear","center")
                       for prefix in ("axle","wheel","hub","wheel-hardware-head","wheel-hardware-thread")}
        for station,index in (("front",1),("rear",1),("center",2)):
            service_parts.update(f"spacer-{station}-{j}-{side}" for j in (index,index+1))
        if not service_parts<=rest.keys():
            raise ValueError("Wheel service identities changed")
        stripped={k:v for k,v in rest.items() if k not in service_parts}
        stage("upper-out",upper,stripped,(0,0,14),(0,sy*40,14))
        stage("upper-down",upper,stripped,(0,sy*40,14),(0,sy*40,-180))
        # Adverse sequence: try to cross the bay without removing the wheels.
        control=moved(upper,(0,sy*16,14))
        stages[f"{side}/wheel-in-place-overlapMm3"]=shop.cad.intersection(control,rest["wheel-front-"+side])
    shop.write(out/"service.json",{"kitSha256":fit["stepSha256"],"stages":stages})
    if any(v.get("clear") is False for v in stages.values() if isinstance(v,dict)):
        raise ValueError("A service leg is unqualified; inspect its retained witness")
    if min(stages[s+"/wheel-in-place-overlapMm3"] for s in ("L","R"))<1:
        raise ValueError("The incorrect removal sequence did not contact the wheel")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("artifacts",type=Path)
    p.add_argument("--out",type=Path,default=HERE)
    a=p.parse_args()
    run(a.artifacts.resolve(),a.out.resolve())
