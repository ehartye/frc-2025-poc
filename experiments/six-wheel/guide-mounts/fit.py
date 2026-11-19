"""Check the assembled guide kit against the complete registered stroke and harness edges."""
import argparse
from pathlib import Path
import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCP.gp import gp_Pnt
from OCP.BRepCheck import BRepCheck_Analyzer

from parts import HERE,ROOT,STATIONS,SIDES,MAST_BOLTS,carriage,j,make,xcyl,center_x,surface_y



def old_cable_crossing(out):
    # Evaluate the published Bezier centerline itself on the issued 370 mm straight slide.
    import math
    folder=ROOT/"station-guide"
    fixed=carriage.selected(folder,"narrow-face.step","fit-narrow.json","stepSha256","volumesMm3")
    control=ROOT/"camera-disconnects/route-controls.json"
    pts=np.asarray(j.read(control)["power"])
    n=len(pts)-1
    for t in np.linspace(0,1,1001):
        basis=[math.comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)]
        x,y,z=np.asarray(basis) @ pts
        stroke=center_x(z)-x
        if 0 <= stroke <=370 and 626<z<849 and 1.2< -y-surface_y(z)<2.0:
            point=[x+stroke,y,z]
            probe=BRepPrimAPI_MakeSphere(gp_Pnt(*point),.25).Shape()
            volume=j.cad.intersection(probe,fixed["guide-negative-y"])
            if abs(volume-4*math.pi*.25**3/3)<.0001:
                return {"controlSha256":j.digest(control),"t":float(t),"strokeMm":float(stroke),
                        "centerMm":list(point),"probeRadiusMm":.25,"insideOldFaceMm3":volume}
    raise ValueError("Expected the legacy centerline crossing")


def run(registry, out):
    out.mkdir(parents=True,exist_ok=True)
    parts=make()
    step=out/"guide-mounts.step"
    j.write_step(step, parts)
    j.finalize_step(step)
    reopened=j.cad.read_named(step,[{"id":k} for k in parts])
    volumes={}
    for k,v in reopened.items():
        if not BRepCheck_Analyzer(v).IsValid() or j.cad.solid_count(v)!=1:
            raise ValueError("Invalid exchanged solid "+k)
        volumes[k]=round(j.cad.volume(v)[0],5)
        if abs(volumes[k]-j.cad.volume(parts[k])[0])>.003:
            raise ValueError("Volume differs on reopen "+k)
    obstacles,edges,registered=carriage.scene(registry,ROOT/"fixed-hoist")
    support=carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    obstacles.update({k:v for k,v in support.items() if not k.startswith("support-machined-mast-")})
    # All original changing solids have full swept bounds, including the translating roof posts.
    obstacles.update(reopened)
    bounds={k:j.cad.bounds(v) for k,v in obstacles.items()}
    def check(body,omit=()):
        box=j.cad.bounds(body)
        hits={}
        for k,b in bounds.items():
            if k not in omit and j.lower_bound(box,b)<.002:
                volume=j.cad.intersection(body,obstacles[k])
                if volume>.001:
                    hits[k]=round(volume,4)
        closest={}
        for key,edge,b,r in edges:
            if j.lower_bound(box,b)<r+5:
                sep=j.distance(body,edge)-r
                closest[key]=min(round(sep,4),closest.get(key,float("inf")))
        return {"overlapMm3":hits,"nearHarnessMm":closest}
    fit={k:check(v,{k}) for k,v in reopened.items() if not k.endswith("machined-mast")}
    tools={}
    adverse={}
    bearing={}
    for side,sign in SIDES:
        tag="guide-"+side+"-"
        liner=reopened[tag+"liner"]
        backing=reopened[tag+"backing"]
        mast=reopened[tag+"machined-mast"]
        # Exact liner inner surface stays coincident with both facets of the legacy profile.
        original=ROOT/"station-guide"
        legacy=carriage.selected(original,"narrow-face.step","fit-narrow.json","stepSha256","volumesMm3")
        face=legacy["guide-positive-y" if sign>0 else "guide-negative-y"]
        bearing[tag+"linerOutsideOriginalMm3"]=round(j.cad.volume(liner)[0]-j.cad.intersection(liner,face),5)
        bearing[tag+"linerBearsOnBackingAt0p1Mm3"]=round(j.cad.intersection(j.cad.translated(liner,(0,sign*.1,0)),backing),3)
        for z in STATIONS:
            key=tag+str(z)+"-"
            arm=reopened[key+"mount-arm"]
            wedge=reopened[key+"wedge"]
            bearing[key+"armToMastAt0p1Mm3"]=round(j.cad.intersection(j.cad.translated(arm,(-.1,0,0)),mast),3)
            bearing[key+"wedgeToArmAt0p1Mm3"]=round(j.cad.intersection(j.cad.translated(wedge,(0,sign*.1,0)),arm),3)
            adverse[key+"straightWideArm"]=check(j.box(-14,center_x(z)+17,sign*160,sign*203,z-6,z+6),
                                                 set(reopened)|{n for n in obstacles if not n.startswith("roof-")})
            for i,(y,dz) in enumerate(MAST_BOLTS):
                hardware=key+"root-"+str(i)+"-"
                own={hardware+p for p in ("screw","locknut","washer","sleeve")}
                tools[hardware+"front-driver"]=check(xcyl(-11,sign*y,z+dz,5.5,77),own)
                tools[hardware+"rear-driver"]=check(xcyl(-125,sign*y,z+dz,5.5,49.2),own)
                # Front-directed withdrawal of head and shank, after removing the nut.
                sweep=j.fuse(xcyl(-14,sign*y,z+dz,4.05,72),xcyl(-79,sign*y,z+dz,2,132))
                tools[hardware+"extraction"]=check(sweep,own)
                bearing[hardware+"sleeveMastWitnessMm3"]=round(j.cad.intersection(
                    j.cad.translated(reopened[hardware+"sleeve"],(0,sign*.2,0)),mast),3)
    record={"stepSha256":j.digest(step),"registrySha256":j.digest(registry),
            "carriageSha256":j.digest(ROOT/"carriage-support/carriage-support.step"),
            "guideProfileSha256":j.digest(ROOT/"station-guide/narrow-face.step"),
            "registeredRobotShapes":len(obstacles)-len(reopened),"harnessEdges":len(edges),
            "oldCableWitness":old_cable_crossing(out),"volumesMm3":volumes,"fit":fit,"tools":tools,"negative":adverse,"interfaces":bearing}
    j.write(out/"fit.json",record)
    for label,rows in (("fit",fit),("tools",tools)):
        violations={k:v for k,v in rows.items() if v["overlapMm3"] or any(x<-.001 for x in v["nearHarnessMm"].values())}
        print(label,len(rows),"conflicts",violations,flush=True)
        if violations:
            raise ValueError(f"{label} contains an unresolved interference")
    print("bearings",bearing,flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("registry",type=Path)
    p.add_argument("--out",type=Path,default=HERE)
    a=p.parse_args()
    run(a.registry,a.out)
