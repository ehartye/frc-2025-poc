"""Verify exclusion from the red reef's scoring pipes using owner-supplied official geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"design/station-l1"))
import build_assembly as cad
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt

FIRST_SHA="19374bc5a48fda1b3ce0505ef604a47d227c904d1784b6514a2c7d654bf3d918"


def name(label):
    attr=TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else ""


def direct_children(table,label):
    seq=cad.Sequence_TDF_Label()
    table.GetComponents_s(label,seq,False)
    for i in range(1,seq.Length()+1):
        occ=seq.Value(i)
        ref=TDF_Label()
        if not table.GetReferredShape_s(occ,ref):
            raise ValueError("Unresolved STEP occurrence")
        yield occ,ref


def measure(step):
    if hashlib.sha256(step.read_bytes()).hexdigest()!=FIRST_SHA:
        raise ValueError("Unreviewed native field revision")
    doc,table=cad.xcaf_doc()
    reader=cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(step))!=cad.IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("Failed to read native field")
    roots=cad.Sequence_TDF_Label()
    table.GetFreeShapes(roots)
    if roots.Length()!=1 or name(roots.Value(1))!="FE-2025":
        raise ValueError("Unexpected field root")
    reef=[o for o in direct_children(table,roots.Value(1)) if name(o[1])=="GE-25300_Red"]
    if len(reef)!=1 or name(reef[0][0])!="NAUO36":
        raise ValueError("Unexpected red reef occurrence")
    parent,ref=reef[0]
    parent_loc=table.GetLocation_s(parent)
    shapes={}
    for occ,child in direct_children(table,ref):
        if name(child)!="GE-25301":
            continue
        # Referred shape is local. Accumulate both transforms to field coordinates.
        loc=parent_loc.Multiplied(table.GetLocation_s(occ))
        shapes[name(occ)]=table.GetShape_s(child).Moved(loc)
    if len(shapes)!=12:
        raise ValueError("Missing one or more of the 12 scoring pipe weldments")
    boxes={occ:cad.bounds(shape) for occ,shape in shapes.items()}
    heights=[b[4] for b in boxes.values()]
    # Check the independent arm-top and expected red-end field registration.
    if any(abs(h-1828.8)>15 for h in heights) or any(
            not (3200<b[0]<b[3]<5500 and -1400<b[2]<b[5]<1400) for b in boxes.values()):
        raise ValueError("Unexpected red reef location or L4 top elevation")
    receipt=json.loads((ROOT/"mission/registered-field-receipt.json").read_text())
    runs=[]
    for trip in receipt["trips"]:
        x,z=trip["departure"]["settled_target_xz_m"]
        y=trip["sector_m"]
        # Solid cylinder contains the pipe's full outside volume, including its hollow center.
        # Expand nominal radius and each end by 2 mm for this specific study.
        p=gp_Pnt(3453.050980729168+x*1000,z*1000,-y*1000-152.8125)
        tool=BRepPrimAPI_MakeCylinder(gp_Ax2(p,gp_Dir(0,0,1)),59.15,305.625).Shape()
        distances={}
        for occurrence,shape in shapes.items():
            dist=BRepExtrema_DistShapeShape(tool,shape)
            if not dist.IsDone() or dist.NbSolution()<1:
                raise ValueError("Native distance unresolved: "+occurrence)
            distances[occurrence]=round(dist.Value(),6)
        runs.append({"piece":trip["piece_id"],"target_x_y_z_m":[x,y,z],"pipe_distances_mm":distances,
                     "minimum_mm":min(distances.values())})
    # Negative control at actual native weldment material checks collision sensitivity.
    combo=next(iter(shapes.values()))
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_VERTEX
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    vertex=TopExp_Explorer(combo,TopAbs_VERTEX).Current()
    point=BRep_Tool.Pnt_s(TopoDS.Vertex(vertex))
    negative=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(point.X(),point.Y(),point.Z()-150),gp_Dir(0,0,1)),59.15,305.625).Shape()
    test=BRepExtrema_DistShapeShape(negative,combo)
    if not test.IsDone() or test.Value()>1e-5:
        raise ValueError("Intersection control failed")
    return {"schema":"reef-branch-exclusion/1","source_sha256":FIRST_SHA,"red_reef":"FE-2025/NAUO36",
            "scoring_weldment_part":"GE-25301","scoring_pipe_count":len(shapes),"expanded_radius_mm":59.15,
            "expanded_length_mm":305.625,"native_l4_top_range_mm":[min(heights),max(heights)],
            "native_material_control_gap_mm":test.Value(),"trips":runs}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("field_step",type=Path)
    p.add_argument("--output",type=Path)
    a=p.parse_args()
    raw=json.dumps(measure(a.field_step),indent=2,allow_nan=False)+"\n"
    if a.output:
        a.output.write_text(raw,encoding="utf-8",newline="\n")
    else:
        print(raw,end="")
