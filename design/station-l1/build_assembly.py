"""Build named STEP and editable OpenSCAD; independently reopen and measure exported solids."""
import argparse
import csv
from functools import cache
import hashlib
import json
import math
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakePrism
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color,Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Reader,STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs,STEPControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ColorGen
from OCP.gp import gp_Ax1,gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf,gp_Vec

from mechanical import HERE, MODEL, COLORS, COST, DENSITY, assembly, offset

SHELF = HERE.parents[1]/"cad/station-receiver/shelf-r6.step"
PARTS=assembly()


def translated(shape, delta):
    tf=gp_Trsf()
    tf.SetTranslation(gp_Vec(*delta))
    return BRepBuilderAPI_Transform(shape,tf,False).Shape()


@cache
def stock_shelf():
    if hashlib.sha256(SHELF.read_bytes()).hexdigest()!=MODEL["source"]["shelfSha256"]:
        raise ValueError("R6 shelf signature changed")
    reader=STEPControl_Reader()
    if reader.ReadFile(str(SHELF))!=IFSelect_RetDone or reader.TransferRoots()!=1:
        raise ValueError("R6 stock import")
    tf=gp_Trsf()
    tf.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,0,1)),math.pi)
    return translated(BRepBuilderAPI_Transform(reader.OneShape(),tf,False).Shape(),[165,0,-60])


def brep(spec):
    op,vals=spec
    if op=="shelf":
        return stock_shelf()
    if op=="box":
        return BRepPrimAPI_MakeBox(gp_Pnt(*vals[:3]),*vals[3:]).Shape()
    if op=="cyl":
        return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*vals[:3]),gp_Dir(*vals[3:6])),*vals[6:]).Shape()
    if op=="hex":
        x,y,z,af,length=vals
        wire=BRepBuilderAPI_MakePolygon()
        r=af/math.sqrt(3)
        for i in range(6):
            a=i*math.pi/3
            wire.Add(gp_Pnt(x+r*math.cos(a),y,z+r*math.sin(a)))
        wire.Close()
        return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(0,length,0)).Shape()
    result=brep(vals[0])
    operation={"cut":BRepAlgoAPI_Cut,"union":BRepAlgoAPI_Fuse,"intersect":BRepAlgoAPI_Common}[op]
    for form in vals[1:]:
        boolean=operation(result,brep(form))
        if not boolean.IsDone():
            raise ValueError("Failed CAD operation: "+op)
        result=boolean.Shape()
    return result


def volume(shape):
    props=GProp_GProps()
    BRepGProp.VolumeProperties_s(shape,props)
    return props.Mass(),props.CentreOfMass().Coord()


def bounds(shape):
    box=Bnd_Box()
    BRepBndLib.AddOptimal_s(shape,box,False,False)
    return (*box.CornerMin().Coord(),*box.CornerMax().Coord())


def solid_count(shape):
    ex=TopExp_Explorer(shape,TopAbs_SOLID)
    count=0
    while ex.More():
        if not BRepCheck_Analyzer(ex.Current()).IsValid():
            raise ValueError("Invalid solid on reopen")
        count+=1
        ex.Next()
    return count


def xcaf_doc():
    doc=TDocStd_Document(TCollection_ExtendedString("XCAF"))
    return doc,XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def write_named(path,shapes,parts=PARTS):
    doc,table=xcaf_doc()
    colors=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    for p,shape in zip(parts,shapes):
        label=table.AddShape(shape,False)
        TDataStd_Name.Set_s(label,TCollection_ExtendedString(p["id"]))
        rgb=bytes.fromhex(COLORS[p["group"]][1:])
        colors.SetColor(label,Quantity_Color(*(x/255 for x in rgb),Quantity_TOC_RGB),XCAFDoc_ColorGen)
    w=STEPCAFControl_Writer()
    if not w.Transfer(doc,STEPControl_AsIs) or w.Write(str(path))!=IFSelect_RetDone:
        raise ValueError("Named STEP export failed")
    # A canonical portable text file also keeps review diffs small.
    path.write_text("\n".join(line.rstrip() for line in path.read_text(encoding="ascii").splitlines())+"\n",encoding="ascii",newline="\n")


def read_named(path,parts=PARTS):
    reader=STEPCAFControl_Reader()
    doc,table=xcaf_doc()
    if reader.ReadFile(str(path))!=IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError("STEP reopen failed")
    labels=Sequence_TDF_Label()
    table.GetFreeShapes(labels)
    result={}
    for i in range(1,labels.Length()+1):
        label=labels.Value(i)
        name=TDataStd_Name()
        if not label.FindAttribute(TDataStd_Name.GetID_s(),name):
            raise ValueError("Unnamed exported part")
        key=name.Get().ToExtString()
        shape=table.GetShape_s(label)
        if key in result or solid_count(shape)!=1:
            raise ValueError("Duplicate ID or disconnected/invalid part: "+key)
        result[key]=shape
    if set(result)!={p["id"] for p in parts}:
        raise ValueError("STEP identity inventory differs from design input")
    return result


def occupied(a,b):
    return all(min(a[k+3],b[k+3])-max(a[k],b[k])>1.e-6 for k in range(3))


def intersection(a,b):
    boolean=BRepAlgoAPI_Common(a,b)
    if not boolean.IsDone():
        raise ValueError("CAD pair intersection failed")
    return volume(boolean.Shape())[0]


def check_pose(name,shapes):
    limits={key:bounds(shape) for key,shape in shapes.items()}
    pairs=[]
    exact=0
    for i,k in enumerate(shapes):
        for other in list(shapes)[i+1:]:
            if occupied(limits[k],limits[other]):
                exact+=1
                v=intersection(shapes[k],shapes[other])
                if v>0.001:
                    pairs.append([k,other,round(v,4)])
    service={}
    for s in MODEL["service"]:
        if s["pose"]!=name:
            continue
        clearance=brep(["box",s["box"]])
        blockers={}
        for k,shape in shapes.items():
            if occupied(bounds(clearance),limits[k]):
                v=intersection(clearance,shape)
                if v>0.001:
                    blockers[k]=round(v,3)
        service[s["id"]]={"overlapsMm3":blockers,"removedBeforeService":["battery-restraint"] if s["id"]=="battery-lift" else []}
    n=len(shapes)
    return {"shapeCount":n,"pairCount":n*(n-1)//2,"nativeBooleanPairs":exact,
            "interferencesOver0.001mm3":pairs,"service":service,
            "boundsMm":[round(f(limits[k][i] for k in shapes),3) for i,f in enumerate([min]*3+[max]*3)]}


def mass_rows(shapes,parts=PARTS):
    rows=[]
    for p in parts:
        vol,center=volume(shapes[p["id"]])
        kg=[vol*DENSITY[p["material"]]]*2 if p["material"] else list(p["mass"])
        rows.append([p["id"],p["group"],p["frame"],p["material"] or "equipment envelope",*kg,*p["costs"],*center,p["basis"]])
    # Includes unmodeled transmission/loom/protection/fasteners; location not verified.
    rows.append(["unmodeled-allocation","reserve","base","budget reserve",2,5,*COST["reserve"],0,0,235,
                 "Lumped allocation. Actual routing, transmission, omitted latches and hardware open"])
    return rows


def budget(rows,pose):
    masses=[sum(row[k] for row in rows) for k in (4,5)]
    md=sum(masses)/2
    cg=[sum((r[4]+r[5])/2*r[8+i] for r in rows)/md for i in range(3)]
    payload=MODEL["motion"]["pipeMass_kg"]
    center=[80+pose["stroke"],0,pose["height"]-60+57.15]
    loaded=[(v*md+p*payload)/(md+payload) for v,p in zip(cg,center)]
    return {"allAccountedMassKg":[round(m,3) for m in masses],"emptyMidpointCgMm":[round(v,2) for v in cg],
            "maxPayloadAtSeatKg":payload,"loadedMidpointCgMm":[round(v,2) for v in loaded],
            "estimatedUsd":[round(sum(r[i] for r in rows),2) for i in (6,7)]}


def write_scad(path,parts=PARTS):
    def scad(spec):
        op,v=spec
        if op=="box":
            return f"translate({v[:3]}) cube({v[3:]});"
        if op=="cyl":
            axis=v[3:6]
            rotation={(0,0,1):"[0,0,0]",(0,1,0):"[-90,0,0]",(1,0,0):"[0,90,0]"}[tuple(axis)]
            return f"translate({v[:3]}) rotate({rotation}) cylinder(r={v[6]}, h={v[7]});"
        if op=="hex":
            return f"translate({v[:3]}) rotate([-90,0,0]) cylinder(r={v[3]/math.sqrt(3)}, h={v[4]}, $fn=6);"
        if op=="shelf":
            return "shelf_stock();"
        return {"union":"union() {","cut":"difference() {","intersect":"intersection() {"}[op]+" "+" ".join(scad(x) for x in v)+" }"
    # Quarter-circle entrance, supplied shelf stock oriented with rounded nose at robot +X.
    header="""// Editable original assembly, millimetres. Preview selected pose (F5); render (F6).
// Generated from mechanical.py. Set selected_part to its CAD ID for individual review.
pose = "stowed"; // [stowed,receive,transport,place]
selected_part = "all";
$fn=72;
"""
    poses=MODEL["poses"]
    header+="heights="+str([poses[k]["height"] for k in poses])+"; strokes="+str([poses[k]["stroke"] for k in poses])+";\n"
    header+='p=pose=="receive"?1:pose=="transport"?2:pose=="place"?3:0;\nheight=heights[p]; stroke=strokes[p];\n'
    header+='module shelf_stock(){ translate([0,210,0]) rotate([90,0,0]) linear_extrude(420) polygon(concat([[-100,-75],[165,-75],[165,-66]],[for(a=[0:1:90]) [159+6*cos(a),-66+6*sin(a)]],[[-100,-60]])); }\n'
    lines=[header]
    for p in parts:
        frame=p["frame"]
        shift={"base":"[0,0,0]","lift":"[0,0,height]","middle":"[stroke/2,0,height]","slide":"[stroke,0,height]"}[frame]
        key=json.dumps(p["id"])
        lines.append(f'if(selected_part=="all" || selected_part=={key}) color("{COLORS[p["group"]]}") translate({shift}) '+scad(p["form"]))
    path.write_text("\n".join(lines)+"\n",encoding="utf-8")


def build(out,poses,parts=PARTS):
    out.mkdir(parents=True,exist_ok=True)
    shapes=[brep(p["form"]) for p in parts]
    write_scad(out/"station-l1.scad",parts)
    receipt={"basis":"Named STEP reopen, B-Rep solid volumes and finite 3D poses; equipment and costs remain allocations.",
             "shelfSourceSha256":MODEL["source"]["shelfSha256"],"poses":{}}
    for name in poses:
        pose=MODEL["poses"][name]
        dest=out/(name+".step")
        write_named(dest,[translated(s,offset(p,pose)) for p,s in zip(parts,shapes)],parts)
        seen=read_named(dest,parts)
        info=check_pose(name,seen)
        rows=mass_rows(seen,parts)
        receipt["poses"][name]={**info,**budget(rows,pose),"stepSha256":hashlib.sha256(dest.read_bytes()).hexdigest()}
        if name=="stowed":
            with (out/"bom.csv").open("w",newline="",encoding="utf-8") as f:
                writer=csv.writer(f)
                writer.writerow(["id","group","kinematicFrame","materialOrBasis","minMassKg","maxMassKg","lowBudgetUsd","highBudgetUsd","cgStowedXmm","cgStowedYmm","cgStowedZmm","qualification"])
                for row in rows:
                    writer.writerow([round(v,4) if isinstance(v,float) else v for v in row])
    (out/"review.json").write_text(json.dumps(receipt,indent=2)+"\n")
    return receipt


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--poses",nargs="+",choices=MODEL["poses"],default=list(MODEL["poses"]))
    args=parser.parse_args()
    print(json.dumps(build(args.output,args.poses),indent=2))
