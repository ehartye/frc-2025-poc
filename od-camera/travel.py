"""Bound motion of new camera structure against every retained chassis/stage part through 370 mm."""
from bridge import *
from dock_tolerance import separation
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

def distance(a,b):
    q=BRepExtrema_DistShapeShape(a,b)
    if not q.IsDone():raise ValueError("Unresolved exact distance")
    return q.Value()

def run(kit):
    a=baseline(0,kit);b=baseline(370,kit);fixed=set(a)-retired(a)-{k for k in a if k.startswith("roof-")}
    roof=structure(kit[0]);minbox=(math.inf,"","");unknown=[];intersections=[];tested=[];minpair=(math.inf,"","")
    rotation=whole.original.rotation_bounds()
    for name in sorted(fixed):
        bounds=cad.bounds(a[name]);other=cad.bounds(b[name]);dx=other[0]-bounds[0]
        if any(abs((other[i]-bounds[i])-(dx if i in (0,3) else 0))>.01 for i in range(6)):
            # Rotating gears are covered conservatively by their union/translational radial reserve.
            if name=="slide-position-line":
                # One line endpoint rides each frame. The convex union of its two endpoint extents
                # in the OUTER frame bounds every intermediate straight draw-wire length.
                relative=[other[i]-(370 if i in (0,3) else 0) for i in range(6)]
                swept=[(min if i<3 else max)(bounds[i],relative[i]) for i in range(6)]
                for key,s in roof.items():
                    gap=separation(cad.bounds(s),swept)
                    minbox=min(minbox,(gap,key,name))
                    if gap<=.01:unknown.append([key,name,0,370,gap])
                continue
            if name not in rotation:raise ValueError("Missing bounded pose law: "+name)
        travel=370-dx
        if abs(travel)<.001:continue
        movingkey=name in rotation
        target=a[name]
        if movingkey:
            reserve=rotation[name]["bounds_mm"]
            bounds=tuple(reserve[i]+(560 if i in (2,5) else 0) for i in range(6))
            target=box(*bounds[:3],*[bounds[i+3]-bounds[i] for i in range(3)])
        for key,s in roof.items():
            bb=cad.bounds(s);swept=tuple(v+min(0,travel) if i==0 else v+max(0,travel) if i==3 else v for i,v in enumerate(bb))
            lower=separation(swept,bounds);minbox=min(minbox,(lower,key,name))
            if lower>3:continue
            n_eval=0;guarantees=[]
            @cache
            def at(q):
                nonlocal n_eval;n_eval+=1
                return distance(cad.translated(s,(q,0,0)),target)
            def visit(lo,hi):
                mid=(lo+hi)/2;v=at(mid);lo_bound=max(lower,v-(hi-lo)/2)
                if lo_bound>1.0:guarantees.append(lo_bound);return
                if hi-lo<.25:
                    guarantees.append(lo_bound)
                    if v<.01:
                        vol=cad.intersection(cad.translated(s,(mid,0,0)),target)
                        if vol>.05:intersections.append([key,name,mid,vol])
                    if lo_bound<=.01:unknown.append([key,name,lo,hi,lo_bound])
                    return
                visit(lo,mid);visit(mid,hi)
            visit(min(0,travel),max(0,travel))
            g=min(guarantees);minpair=min(minpair,(g,key,name));tested.append([key,name,g,n_eval])
    return {"bridgeSha":sha(HERE/"revised-bridge.step"),"broadMinimum":minbox,"worstRefined":minpair,
            "refined":tested,"unresolved":unknown,"intersections":intersections,
            "scope":"Relative rigid translation against 593-body retained frames; envelopes for registered rotating bodies. Outer-stage welded joints checked with static geometry."}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--lamp",type=Path,required=True);a=p.parse_args()
    (HERE/"travel.json").write_text(json.dumps(run(source(a.lamp)),indent=2)+"\n",encoding="utf8",newline="\n")
