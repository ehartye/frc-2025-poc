"""Check replacement articles against the registered robot stroke, guide kit and harness."""
import argparse
from pathlib import Path
from arm_route import ARM, BACK, NOSE, ROOT, carriage, inputs, j


def inspect(registry, out, layouts):
    scene, edges, _ = carriage.scene(registry, ROOT/"fixed-hoist")
    supports=carriage.selected(ROOT/"carriage-support","carriage-support.step","fit.json","stepSha256","volumesMm3")
    scene.update({k:v for k,v in supports.items() if not k.startswith("support-machined-mast-")})
    original=inputs()
    scene.update(original)
    for layout in layouts:
        folder=out/layout
        parts=carriage.selected(folder,"route.step","revision.json","stepSha256","volumesMm3")
        installed={**scene,**parts}
        bounds={k:j.cad.bounds(v) for k,v in installed.items()}
        hits={}
        near={}
        for k,body in parts.items():
            bb=bounds[k]
            for name,other in installed.items():
                if name==k or j.lower_bound(bb,bounds[name])>.002:
                    continue
                volume=j.cad.intersection(body,other)
                if volume>.001:
                    hits[k+"/"+name]=round(volume,5)
            for name,edge,box,radius in edges:
                if j.lower_bound(bb,box)<radius+10:
                    d=j.distance(body,edge)-radius
                    near[k+"/"+name]=min(near.get(k+"/"+name,float("inf")),round(d,5))
        arm=parts[ARM]
        wedge=parts[NOSE[0]]
        mast=installed["guide-L-machined-mast"]
        bearing={
            "rootMast":j.cad.intersection(j.cad.translated(arm,(-.1,0,0)),mast),
            "wedgeArm":j.cad.intersection(j.cad.translated(wedge,(0,.1,0)),arm),
            "wedgeBacking":j.cad.intersection(j.cad.translated(wedge,(0,-.1,0)),parts[BACK]),
            "linerBacking":j.cad.intersection(j.cad.translated(installed["guide-L-liner"],(0,.1,0)),parts[BACK])}
        if min(bearing.values()) < 1:
            raise ValueError(("Missing support land",layout,bearing))
        result={"stepSha256":j.digest(folder/"route.step"),"registrySha256":j.digest(registry),
            "installedSceneShapes":len(installed),"harnessEdges":len(edges),
            "intersectionMm3":hits,"nearCableClearanceMm":near,"landAt0p1Mm3":bearing}
        j.write(folder/"clearance.json",result)
        print(layout,"intersections",hits,"wire intrusions",{k:v for k,v in near.items() if v < -.001},flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("registry",type=Path)
    p.add_argument("output",type=Path)
    p.add_argument("layouts",nargs="+")
    a=p.parse_args()
    inspect(a.registry,a.output,a.layouts)
