"""Register the installed foreground on the fixed camera before observing the first field contact."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np

from cameras import HERE,ROOT,bridge,camera,sha,stored,mount
from sight import background
from images import silhouette


def run(cache,field):
    fixed=background(cache/"relieved-fixed-place.npz")
    nav=json.loads((HERE/"navigation.json").read_bytes())["cases"]
    src=bridge.mod("relief_reef",ROOT/"moving-observer/clearance.py")
    solids=src.reef_parts(field)
    if sha(field)!=src.FIRST_SHA:raise ValueError("Unknown field")
    mesh=[]
    for name,shape in solids.items():
        points,faces=mount.original.triangulate(shape)
        v=np.asarray(points).reshape(-1,3)/10
        mesh.append((name,v,faces))
    out={}
    with (HERE/"landing-windows.csv").open() as f:
        settled=next(r for r in csv.DictReader(f) if r["case"]=="coral-A/250/115")
    for piece in ("coral-A","coral-B"):
        ex,ey,h=next(r["finalBodyErrorMmDeg"] for r in nav if r["pipe"]==piece and r["mode"]=="nominal")
        angle=math.radians(h);c,s=math.cos(angle),math.sin(angle)
        cx=-490+ex;cy=(-180 if piece=="coral-A" else 220)+ey
        posed=[]
        for name,v,faces in mesh:
            x=v[:,0]-3453.050980729168-cx;y=-v[:,2]-cy
            local=np.column_stack((x*c+y*s,-x*s+y*c,v[:,1]))
            if local[:,0].max()< -315 or local[:,0].min()>1700:continue
            posed.append([name,"fixture","#a0a0a0",np.rint(local*10).astype(int).reshape(-1).tolist(),faces])
        environment=camera.images_for_pose({"parts":posed})
        combined=[(np.where(fd<rd,fg,rg),np.minimum(fd,rd)) for (rg,rd),(fg,fd) in zip(fixed,environment)]
        if piece=="coral-B":
            x=float(settled["x_m"])*1000-cx;y=float(settled["y_m"])*1000-cy
            pos=np.array([c*x+s*y,-s*x+c*y,float(settled["z_m"])*1000])
            a=np.array([float(settled["axis_"+k]) for k in "xyz"])
            axis=np.array([c*a[0]+s*a[1],-s*a[0]+c*a[1],a[2]])
            with_neighbor=silhouette(combined,pos,axis)
            # Preserve a conservative near bound for every foreground ray intercepted by
            # the earlier pipe. The first pipe lies well to the side of the second delivery.
            eye=camera.ORIGIN
            closest=(pos-eye)@np.array([camera.C,0,-camera.S])-162
            combined=[(image,np.where(image!=gray,np.minimum(depth,closest),depth)) for image,(gray,depth) in zip(with_neighbor,combined)]
        path=cache/(piece+"-reef-background.npz")
        out[piece]={"backgroundSha256":stored(path,combined),"reefBodies":len(posed),"stationaryChassisMmDeg":[cx,cy,h],
                    "fieldVisiblePixels":[int(np.count_nonzero(d<rd)) for (_,d),(_,rd) in zip(combined,fixed)]}
    (HERE/"field-images.json").write_text(json.dumps({"fieldSha256":sha(field),"settledFirstAtB":settled,"cases":out},indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("cache","field"):p.add_argument(n,type=Path)
    a=p.parse_args();run(a.cache,a.field)
