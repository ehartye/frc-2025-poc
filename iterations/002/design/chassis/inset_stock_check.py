"""Independently form the shop features and inspect actual, named strut/deck interfaces."""
from pathlib import Path
import hashlib,json,math,sys
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeWire,BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism,BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GC import GC_MakeArcOfCircle
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Vec
import numpy as np
from inset_chassis_shop import HERE


def edges(p):
    if p["kind"]=="polygon":
        pts=np.asarray(p["points"])
        return [(a,b) for a,b in zip(pts,np.roll(pts,-1,axis=0))]
    x,y,X,Y=p["bounds"];r=p["radius"]
    if not r:return edges(dict(kind="polygon",points=[(x,y),(X,y),(X,Y),(x,Y)]))
    z=r/math.sqrt(2)
    return [((x+r,y),(X-r,y)),((X-r,y),(X-r+z,y+r-z),(X,y+r)),
            ((X,y+r),(X,Y-r)),((X,Y-r),(X-r+z,Y-r+z),(X-r,Y)),
            ((X-r,Y),(x+r,Y)),((x+r,Y),(x+r-z,Y-r+z),(x,Y-r)),
            ((x,Y-r),(x,y+r)),((x,y+r),(x+r-z,y+r-z),(x+r,y))]


def extrude(step,profile):
    axis=step["axis"];d=np.eye(3)[axis]*step["travel"]
    if "offsetXY" in step:d[:2]=step["offsetXY"]
    point=lambda uv:gp_Pnt(*np.insert(np.asarray(uv,dtype=float),axis,step["start"]))
    if profile["kind"]=="circle":
        return BRepPrimAPI_MakeCylinder(gp_Ax2(point(profile["center"]),gp_Dir(*d)),profile["radius"],abs(step["travel"])).Shape()
    w=BRepBuilderAPI_MakeWire()
    for e in edges(profile):
        pts=[point(a) for a in e]
        curve=BRepBuilderAPI_MakeEdge(*pts) if len(pts)==2 else BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(*pts).Value())
        w.Add(curve.Edge())
    return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(),gp_Vec(*d)).Shape()


def form(row,skipped=None):
    from component_shape import volume
    body=None
    for idx,feature in enumerate(row["steps"]):
        before=volume(body) if body is not None else 0.
        for profile in feature["profiles"]:
            tool=extrude(feature,profile);kind=feature["kind"]
            if kind=="new":body=tool;continue
            action=(BRepAlgoAPI_Cut if kind=="cut" else BRepAlgoAPI_Fuse)(body,tool)
            if not action.IsDone():raise ValueError("Boolean failed "+row["name"]+" "+feature["name"])
            body=action.Shape()
        if skipped is not None and feature["kind"]=="cut" and abs(volume(body)-before)<1e-6:
            skipped.append(idx)
    solids=[];it=TopExp_Explorer(body,TopAbs_SOLID)
    while it.More():
        solids.append(it.Current());it.Next()
    if len(solids)!=1 or not BRepCheck_Analyzer(solids[0]).IsValid():raise ValueError(("Invalid stock",row["name"],len(solids)))
    return solids[0]


def run(pkg,shared=False):
    sys.path.insert(0,str(pkg/"scripts"))
    from fusion_authority import register_path,verify
    from component_shape import volume
    from mechanical_motion import common,bounds
    from showcase_contacts import box_gap
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-005/lineage.json")
    file=HERE/"inset-chassis-shop.json";r=json.loads(file.read_bytes())
    if auth!=r["authority"]:raise ValueError("Different issue")
    parts={};records={};pairs=[]
    for row in r["articles"]:
        name=row["name"];skipped=[];s=form(row,skipped);parts[name]=s
        if "matrixMm" in row:
            from OCP.gp import gp_Trsf
            from OCP.TopLoc import TopLoc_Location
            tf=gp_Trsf();tf.SetValues(*np.array(row["matrixMm"])[:3].ravel())
            s=s.Moved(TopLoc_Location(tf));parts[name]=s
        records[name]=dict(group=row["group"],bounds=bounds(s),volumeMm3=volume(s),nominalMassKg=volume(s)*2.7e-6,alreadyRemovedCutIndices=skipped)
    for i,(name,a) in enumerate(parts.items()):
        for other,b in list(parts.items())[i+1:]:
            if box_gap(records[name]["bounds"],records[other]["bounds"])>.01:continue
            vol=common(a,b)
            pairs.append(dict(a=name,b=other,commonMm3=vol))
    result=dict(authority=auth,shopPlanSHA256=hashlib.sha256(file.read_bytes()).hexdigest(),parts=records,actualPairs=pairs,
        nominalStockKg=sum(r["nominalMassKg"] for r in records.values()),
        massByGroup={g:sum(r["nominalMassKg"] for r in records.values() if r["group"]==g) for g in sorted({r["group"] for r in records.values()})})
    if shared:return parts,result
    return result


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-stock-check.json").write_text(json.dumps(r,indent=2)+"\n")
    print(r["massByGroup"]);print([(p["a"],p["b"],round(p["commonMm3"],3)) for p in r["actualPairs"] if p["commonMm3"]>.01])
