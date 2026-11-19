"""Original spring-seat shims and compressed spring envelopes inside the installed hardware."""
import argparse
import json
from pathlib import Path
from design import ROOT, SIDES, cad, finalize_step, registered_robot, sha
from mechanical import cylinder, cut
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

THICKNESS=.75
OUTER=3.2
INNER=1.7


def annulus(x,y,z,sign,length):
    axis=(0,sign,0)
    return cad.brep(cut(cylinder(x,y,z,axis,OUTER,length),cylinder(x,y-sign,z,axis,INNER,length+2)))


def study(output):
    robot=registered_robot()
    bodies={}
    dimensions={}
    for side in SIDES:
        sign=1 if side=="L" else -1
        for end in ("front","rear"):
            key=f"grip-{side}-{end}-spring"
            old=robot[key]; x,y,z,xx,yy,zz=cad.bounds(old)
            cx,cz=(x+xx)/2,(z+zz)/2
            start=y if side=="L" else yy
            length=yy-y
            if max(abs(length-14),abs(xx-x-2*OUTER),abs(zz-z-2*OUTER))>.00001:raise ValueError(key)
            shim=key+"-seat-shim"
            bodies[shim]=annulus(cx,start+sign*(length-THICKNESS),cz,sign,THICKNESS)
            bodies[key]=annulus(cx,start,cz,sign,length-THICKNESS)
            dimensions[key]= {"neutralLengthMm":length-THICKNESS,"lengthAt1p6OutwardMm":length-THICKNESS-1.6,
                    "lengthAt0p6InwardMm":length-THICKNESS+.6,"freeLengthAssumedMm":19.}
    issued=ROOT/"station-supply/receiver-stops.step"
    labels=[{"id":f"grip-{s}-inboard-hardstop"} for s in SIDES]
    source_stops=cad.read_named(issued,labels)
    for side in SIDES:
        k=f"grip-{side}-inboard-hardstop"
        bodies[k]=cad.translated(source_stops[k],(0,.15 if side=="L" else -.15,0))
    tags=[{"id":k,"group":"hardware"} for k in bodies]
    cad.write_named(output,list(bodies.values()),tags,{"hardware":"#83a8b9"})
    finalize_step(output)
    reopened=cad.read_named(output,tags)
    for key in dimensions:
        shim=key+"-seat-shim"
        before=cad.volume(robot[key])[0]
        a,b=reopened[key],reopened[shim]
        v=cad.volume(a)[0]+cad.volume(b)[0]
        if any(cad.solid_count(s)!=1 for s in (a,b)) or cad.intersection(a,b)>.001 or abs(v-before)>.001:
            raise ValueError(key+" material discrepancy")
        if abs(cad.intersection(robot[key],a)+cad.intersection(robot[key],b)-before)>.001:
            raise ValueError(key+" escaped original spring allocation")
    stops={}
    for side in SIDES:
        key=f"grip-{side}-inboard-hardstop"
        sign=1 if side=="L" else -1
        stop=reopened[key]; lug=robot[f"grip-{side}-moving-stop-lug"]
        x,y,z,xx,yy,zz=cad.bounds(stop)
        footprint=cad.brep(["box",[x,y,z-.1,xx-x,yy-y,.1]])
        support=cad.intersection(footprint,robot[f"grip-{side}-front-support-foot"])/.1
        checks={}
        for movement in (-.4,-.35,-.3,1.5):
            candidate=cad.translated(lug,(0,sign*movement,0))
            query=BRepExtrema_DistShapeShape(stop,candidate)
            if not query.IsDone():raise ValueError("Unresolved stop-lug distance")
            checks[str(movement)]={"gapMm":query.Value(),"volumeMm3":cad.intersection(stop,candidate)}
        stops[key]={"supportedAreaMm2":support,"lugOutwardMm":checks,"massChangeMm3":cad.volume(stop)[0]-cad.volume(source_stops[key])[0]}
    return {"priorStopSha256":sha(issued),"stopChecks":stops,"source":"design/competition-package/shop-duplex/red/source-register.json",
            "sha256":sha(ROOT/"design/competition-package/shop-duplex/red/source-register.json"),
            "nativeSha256":sha(output),"fourShimVolumeMm3":sum(cad.volume(v)[0] for k,v in reopened.items() if k.endswith("shim")),
            "parts":dimensions,"stockSpringForceN":5.,"revisedSpringForceN":5.75,"springsPerCartridge":2}

if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__);a.add_argument("--step",type=Path,required=True);a.add_argument("--output",type=Path,required=True)
    p=a.parse_args();p.output.write_text(json.dumps(study(p.step),indent=2)+"\n",encoding="utf-8",newline="\n")
