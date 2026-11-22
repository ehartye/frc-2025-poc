"""Join occupied BReps and exact placements before and after the seven-part machining change."""
import hashlib,io,json,re,sys
from pathlib import Path
import numpy as np
from OCP.BRepTools import BRepTools
from OCP.TopTools import TopTools_FormatVersion
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf,gp_Vec
from inset_stock_check import HERE

def signature(body):
    output=io.BytesIO()
    BRepTools.Write_s(body.Located(TopLoc_Location()),output,False,False,TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    data=output.getvalue()
    if len(data)<100 or b"TShapes" not in data:raise ValueError("Empty shape serialization")
    tr=body.Location().Transformation()
    pose=np.array([[tr.Value(i,j) for j in range(1,5)] for i in range(1,4)])
    return hashlib.sha256(data).hexdigest(),pose

def run(scripts,step):
    sys.path.insert(0,str(scripts))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    from component_shape import measured_difference
    reg=register_path(HERE.parents[3]);auth=verify(HERE.parents[3],reg,reg.parent/"issue-006/lineage.json")
    before=reg.parent/"issue-006/assembly.step";a,ssa=load_solids(before);b,ssb=load_solids(step)
    roles=json.loads((HERE/"connected-654-exchange.json").read_bytes())
    changed=json.loads((HERE/"inset-corrected-seven-shapes.json").read_bytes())
    if changed["authority"]!=auth or roles["native"]!=auth["archiveSha256"] or (len(a),len(b),len(ssa),len(ssb))!=(654,654,0,0):raise ValueError("Changed source or roster")
    stripped=lambda n:re.sub(r" \[[0-9.]+\]$","",n)
    bt={n:(signature(v),np.array(bounds(v))) for n,v in b.items()}
    rows=[];unmatched=[];seen=set()
    for r in roles["pairs"]:
        role=r["target"];n=r["exchange"]
        if role in changed["forms"]:continue
        body=a[n];(h,pose)=signature(body);box=np.array(bounds(body))
        options=[]
        for nn,((hh,q),bbox) in bt.items():
            g=max(abs(box-bbox))
            if nn not in seen and stripped(nn)==stripped(n) and g<=.006:options.append((nn,hh,q,g))
        if len(options)!=1:
            unmatched.append(dict(role=role,candidates=[x[0] for x in options]));continue
        nn,hh,q,g=options[0];seen.add(nn)
        row=dict(role=role,before=n,after=nn,originalOccupiedRepresentation=h,newOccupiedRepresentation=hh,
                 maxPoseDelta=max(abs(pose-q).ravel()),worldBoundsDeltaMm=g)
        row["identicalBRepAndPose"]=(hh==h and row["maxPoseDelta"]<=1.e-10)
        rows.append(row)
    # Include neighbors reached by a 2 mm radial setup error on the actual old/new axes.
    # Separate occupied-form proof from any contact made by the cutter envelope.
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
    from component_shape import boolean_measure
    from showcase_contacts import box_gap
    from mechanical_motion import distance
    from itertools import product
    tools=[]
    for ysign in (-1,1):
        for x,y in [(234,145),(266,145),(234,132),(266,158)]:
            for start,end,r in [(174.0,187.6,3.575),(187.1,193.9,3.575),(202.9,205.6,6.35)]:
                tool=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x,ysign*y,start),gp_Dir(0,0,1)),r,end-start).Shape()
                tools.append((tool,bounds(tool)))
    proximate=[]
    for row in rows:
        old=a[row["before"]];bb=bounds(old)
        candidates=[obj for obj,ob in tools if box_gap(bb,ob)<=2.0]
        if not candidates:continue
        gap=min(distance(old,q) for q in candidates)
        if gap>2.0:continue
        new=b[row["after"]]
        oa=boolean_measure(old,new,BRepAlgoAPI_Cut);ne=boolean_measure(new,old,BRepAlgoAPI_Cut)
        proximate.append(dict(role=row["role"],minimumCutterGapMm=gap,absentMm3=oa[0],absentValid=oa[1],extraMm3=ne[0],extraValid=ne[1]))
    # Challenge the method with both translated and differently machined copies.
    original=next(r for r in roles["pairs"] if r["target"]=="Pan crossmember 250")["exchange"]
    one=a[original];h,pose=signature(one);move=gp_Trsf();move.SetTranslation(gp_Vec(.5,0,0))
    moved_hash,moved_pose=signature(one.Moved(TopLoc_Location(move)))
    negatives=dict(translatedSameStock=dict(localUnchanged=h==moved_hash,poseDelta=max(abs(pose-moved_pose).ravel())))
    compared=next(n for n in b if stripped(n)=="Pan crossmember 250")
    # Actual changed stock geometry must be detected even when its installed bounds are the same.
    y,q=signature(b[compared]);negatives["changedThroughHoles"]=dict(localUnchanged=h==y,boundsDeltaMm=max(abs(np.array(bounds(one))-np.array(bounds(b[compared])))))
    return dict(authority=auth,after=hashlib.sha256(step.read_bytes()).hexdigest(),paired=rows,unmatched=unmatched,
        exposedOriginalAndRevisedProfiles=proximate,controls=negatives, exactOccupiedAndPose=sum(r["identicalBRepAndPose"] for r in rows),
        scope="Hash of the complete analytic OCCT solid BRep with topology and face/edge trims, no mesh, plus independent location-matrix equality and measured world bounds. A positive identity establishes the same represented solid; a serialization difference requires geometric follow-up. Newly machined stock is outside the retained cohort.")
if __name__=="__main__":
    result=run(*map(Path,sys.argv[1:]));(HERE/"connected-seven-retained.json").write_text(json.dumps(result,indent=1)+"\n");print(result["exactOccupiedAndPose"],len(result["unmatched"]),result["controls"])
