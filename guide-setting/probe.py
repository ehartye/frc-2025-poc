"""Probe both finished liner faces on the unloaded, stowed assembly in its chassis coordinates."""
import argparse
import math
from pathlib import Path

from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCP.gp import gp_Pnt
from setting_geometry import HERE, ROOT, SIDES, carriage, j
from parts import center_x, surface_y

HEIGHTS=(628.,687.,767.,787.,810.)
EDGES=(("rear",-16.),("front",16.))


def run(registry):
    fit=j.read(HERE/"fit.json")
    if j.digest(registry)!=fit["registrySha256"]:
        raise ValueError("Unregistered scene")
    scene,edges,reg=carriage.scene(registry,ROOT/"fixed-hoist")
    for k,box in reg["stowed"].items():
        if k in scene and max(abs(x-y) for x,y in zip(box,reg["bounds"][k]))>.001:
            scene[k]=carriage.bbox(box)
    braces=carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    scene.update({k:v for k,v in braces.items() if not k.startswith("support-machined-mast-")})
    source=ROOT/"guide-mounts"
    scene.update(carriage.selected(source,"guide-mounts.step","fit.json","stepSha256","volumesMm3"))
    setting=carriage.selected(HERE,"setting.step","fit.json","stepSha256","volumesMm3")
    scene.update({k:v for k,v in setting.items() if k in fit["changedInstalled"]})
    bounds={k:j.cad.bounds(v) for k,v in scene.items()}
    points={}
    radius=1.5
    def inspect(point,toward,target_name):
        tip=tuple(coord-radius*n for coord,n in zip(point,toward))
        away=[-v for v in toward]
        ball=BRepPrimAPI_MakeSphere(gp_Pnt(*tip),radius).Shape()
        # A ball moving 30 mm, including the entire straight 35 mm long stem.
        sweep=j.cad.brep(["cyl",[*tip,*away,radius,30.]])
        end=[coord+30*delta for coord,delta in zip(tip,away)]
        sweep=j.fuse(j.fuse(sweep,ball),BRepPrimAPI_MakeSphere(gp_Pnt(*end),radius).Shape())
        stem=j.cad.brep(["cyl",[*tip,*away,1.,65.]])
        gauge=j.fuse(sweep,stem)
        bb=j.cad.bounds(gauge)
        hits={}
        for k,v in scene.items():
            if j.lower_bound(bb,bounds[k])<.001:
                overlap=j.cad.intersection(gauge,v)
                if overlap>.001:
                    hits[k]=round(overlap,5)
        wire={}
        for key,e,b,r in edges:
            if j.lower_bound(bb,b)<r:
                sep=j.distance(gauge,e)-r
                if sep<-.001: wire[key]=min(sep,wire.get(key,0.))
        target=scene[target_name]
        # Force the actual ball 0.10 mm through the inward face: compare the exact spherical cap.
        witness=j.cad.intersection(j.cad.translated(ball,[.1*v for v in toward]),target)
        expected=math.pi*.1**2*(radius-.1/3)
        if abs(witness-expected)>.001:
            raise ValueError((target_name,point,"unregistered probe tip",witness))
        return {"pointMm":list(point),"tipCenterMm":list(tip),"normalTowardMaterial":list(toward),
                "intentionalPenetrationMm3":witness,"materialOverlapMm3":hits,"cableIntrusionMm":wire}

    for z in HEIGHTS:
        slope=.038 if z>770 else 0.
        denom=math.hypot(1,slope)
        for edge,offset in EDGES:
            station=f"{z:g}/{edge}"
            x=center_x(z)+offset
            row=points[station]={"xMm":x,"zMm":z,"nominalHalfWidthMm":surface_y(z),"sides":{}}
            for side,sign in SIDES:
                row["sides"][side]=inspect((x,sign*surface_y(z),z),(0,sign/denom,-slope/denom),f"guide-{side}-liner")
    datums={}
    for side,sign in SIDES:
        mast=f"guide-{side}-machined-mast"
        for z in (630,750,930):
            datums[f"{side}-fore-{z}"]=inspect((-20,sign*222.5,z),(-1,0,0),mast)
        for i,(x,z) in enumerate(((-57,641),(-33,740),(-57,838))):
            datums[f"{side}-inner-{i}"]=inspect((x,sign*210,z),(0,sign,0),mast)
        datums[f"{side}-top"]=inspect((-45,sign*211,1020),(0,0,-1),mast)
    blocked=inspect((-20,-222.5,845),(-1,0,0),"guide-R-machined-mast")
    if "wire/data-tail-10" not in blocked["materialOverlapMm3"]:
        raise ValueError("Missing conservative cable-envelope control")
    power=inspect((-20,-222.5,820),(-1,0,0),"guide-R-machined-mast")
    if "wire/power-tail-11" not in power["materialOverlapMm3"]:
        raise ValueError("Missing power tail control")
    result={"rejectedRightMastApproaches":{"845":blocked,"820":power},"settingStepSha256":fit["stepSha256"],"guideStepSha256":fit["guideSha256"],
            "registrySha256":fit["registrySha256"],"pose":"stowed, unloaded, setting pins out",
            "stemDiameterMm":2.,"stemLengthMm":35.,"tipRadiusMm":radius,"retractionMm":30.,"stations":points,"mastDatums":datums}
    j.write(HERE/"probe.json",result)
    issues={k:{s:r for s,r in v["sides"].items() if r["materialOverlapMm3"] or r["cableIntrusionMm"]}
            for k,v in points.items()}
    print({k:v for k,v in issues.items() if v})
    datum_issues={k:v for k,v in datums.items() if v["materialOverlapMm3"] or v["cableIntrusionMm"]}
    print(datum_issues)
    if any(issues.values()) or datum_issues:
        raise ValueError("Some survey approaches are obstructed")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry",type=Path)
    run(parser.parse_args().registry)
