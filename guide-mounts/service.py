"""Continuous guide-module removal, with the loose root screws removed and masts retained."""
import argparse
from pathlib import Path
import numpy as np

from parts import HERE, ROOT, SIDES, STATIONS, TOP, carriage, center_x, surface_y, j


def pieces():
    # Conservative disjoint entry regions around the welded arms, backing, liner and installed tip hardware.
    rows={}
    for z in STATIONS:
        inside=surface_y(z+6)+8.15
        x=center_x(z)
        rows[f"arm-{z}"]=(-14,x+17,inside,inside+30,z-6,z+6)
        rows[f"root-{z}"]=(-20,-14,inside,235,z-11,z+11)
        for dz in (-10,8):
            rows[f"root-rib-{z}-{dz}"]=(-42,-20,inside+2,209.5,z+dz,z+dz+2)
        rows[f"shoe-and-screws-{z}"]=(x-14,x+14,surface_y(z-4)+3,inside+12,z-5,z+5)
    heights=sorted({*range(625,TOP,5),770,TOP})
    for lo,hi in zip(heights,heights[1:]):
        rows[f"face-{lo}"]=(center_x(lo)-20,center_x(hi)+20,surface_y(lo),surface_y(hi)+7.8,lo,hi)
    return rows


def run(registry):
    obstacles,edges,reg=carriage.scene(registry,ROOT/"fixed-hoist")
    support=carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    obstacles.update(support)
    source=j.read(HERE/"fit.json")
    if j.digest(HERE/"guide-mounts.step")!=source["stepSha256"]:
        raise ValueError("Changed guide kit")
    kit=j.cad.read_named(HERE/"guide-mounts.step",[{"id":k} for k in source["volumesMm3"]])
    for side,sign in SIDES:
        del obstacles["support-machined-mast-"+side]
        for k,v in kit.items():
            if k.endswith("machined-mast") or k.endswith("-sleeve"):
                obstacles[k]=v
    def pose(extended):
        current=obstacles.copy()
        for k,v in reg["bounds"].items():
            if k not in current: continue
            a=np.array(reg["stowed"][k]); b=np.array(v)
            if np.max(abs(a-b))<.001:continue
            delta=b[3]-a[3]
            # Endpoint records are rigid +X translations for these bodies. Retain full bounds otherwise.
            if abs(delta-370)<.001 and np.max(abs(b-a-[0,0,0,370,0,0]))<.001:
                box=a+np.array([370,0,0,370,0,0]) if extended else a
                current[k]=carriage.bbox(box)
        return current
    # Independently cover every retained module article with the same material envelopes used for its sweeps.
    covered={}
    for side,sign in SIDES:
        tag="guide-"+side+"-"
        guards=[j.box(x,X,sign*y,sign*Y,z,Z) for x,X,y,Y,z,Z in pieces().values()]
        for k,body in kit.items():
            if not k.startswith(tag) or "-root-" in k or k.endswith("machined-mast"):
                continue
            bb=j.cad.bounds(body)
            rest=body
            for guard in guards:
                if j.lower_bound(bb,j.cad.bounds(guard))<.001 and j.cad.solid_count(rest):
                    rest=j.cut(rest,guard)
            volume=j.cad.volume(rest)[0] if j.cad.solid_count(rest) else 0.
            if volume>.001:
                raise ValueError((k,"module sweep misses material",volume))
            covered[k]=round(volume,5)
    result={}
    for extended,label in ((True,"extended"),(False,"stowed")):
        current=pose(extended)
        bounds={k:j.cad.bounds(s) for k,s in current.items()}
        for side,sign in SIDES:
            # Withdraw 26 mm forward so the back ribs clear the mast, then 385 mm outward.
            phases=(("unseat",(0,26,0,0)),("outboard",(26,26,0,385)),
                    ("inboard",(0,0,-65,0)),("forward",(0,670,-65,-65)))
            opposite="guide-"+("R" if side=="L" else "L")+"-"
            comparison={**current,**{k:v for k,v in kit.items() if k.startswith(opposite)}}
            bounds={k:j.cad.bounds(v) for k,v in comparison.items()}
            for phase,(xa,xb,ya,yb) in phases:
                hits={}
                for name,(x,X,y,Y,z,Z) in pieces().items():
                    body=j.box(x+xa,X+xb,sign*(y+ya),sign*(Y+yb),z,Z)
                    bb=j.cad.bounds(body)
                    for k,b in bounds.items():
                        if j.lower_bound(bb,b)<.001 and (v:=j.cad.intersection(body,comparison[k]))>.001:
                            hits.setdefault(k,[]).append([name,round(v,3)])
                    for k,edge,b,r in edges:
                        if j.lower_bound(bb,b)<r and (d:=j.distance(body,edge)-r)<-.001:
                            hits.setdefault(k+" envelope intrusion mm",[]).append([name,round(d,3)])
                result[f"{label}/{side}/{phase}"]=hits
    j.write(HERE/"service.json",{"fitSha256":j.digest(HERE/"fit.json"),"conservativeRegions":len(pieces()),
        "uncoveredVolumeMm3":covered,"forwardMm":26,"outboardMm":385,"inboardMm":65,"forwardAfterInboardMm":670,"extendedSlideMm":370,"collisions":result})
    print({k:list(v) for k,v in result.items()})
    for side,_ in SIDES:
        for phase in ("inboard","forward"):
            if result[f"stowed/{side}/{phase}"]:
                raise ValueError("Stowed guide extraction is obstructed")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("registry",type=Path)
    run(parser.parse_args().registry)
