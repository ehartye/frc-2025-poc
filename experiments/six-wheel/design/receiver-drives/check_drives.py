"""Reopen source robot, measure manufacturing/reserve pair volumes, export original named layer."""
from collections import Counter
import argparse,hashlib,json
from pathlib import Path
from itertools import combinations
from OCP.STEPCAFControl import STEPCAFControl_Writer,STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_AsIs
from OCP.IFSelect import IFSelect_RetDone
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.collections import Sequence_TDF_Label
from receiver_drives import HERE,REPLACES,added_shapes,parts,cad,block,AXES
from mechanical import cylinder

PKG=HERE.parent/"station-l1"


def interferences(a,b,internal=False):
    ab={k:cad.bounds(s) for k,s in a.items()}
    bb=ab if internal else {k:cad.bounds(s) for k,s in b.items()}
    pairs=combinations(a,2) if internal else ((i,j) for i in a for j in b)
    result={}
    for i,j in pairs:
        if cad.occupied(ab[i],bb[j]):
            vol=cad.intersection(a[i],b[j])
            if vol>1e-3:result[i+" / "+j]=round(vol,3)
    return result


def export(path,shapes):
    doc,table=cad.xcaf_doc()
    for name,shape in shapes.items():
        label=table.AddShape(shape,False)
        TDataStd_Name.Set_s(label,TCollection_ExtendedString(name))
    w=STEPCAFControl_Writer()
    if not w.Transfer(doc,STEPControl_AsIs) or w.Write(str(path))!=IFSelect_RetDone:raise ValueError("export")
    path.write_text("\n".join(l.rstrip() for l in path.read_text(encoding="ascii").splitlines())+"\n",encoding="ascii",newline="\n")
    doc,table=cad.xcaf_doc();r=STEPCAFControl_Reader()
    if r.ReadFile(str(path))!=IFSelect_RetDone or not r.Transfer(doc):raise ValueError("import")
    labels=Sequence_TDF_Label();table.GetFreeShapes(labels);seen={}
    for i in range(1,labels.Length()+1):
        item=labels.Value(i);name=TDataStd_Name()
        if not item.FindAttribute(TDataStd_Name.GetID_s(),name):raise ValueError("anonymous")
        seen[name.Get().ToExtString()]=table.GetShape_s(item)
    if set(seen)!=set(shapes) or any(cad.solid_count(body)!=1 for body in seen.values()):raise ValueError("inventory")
    return seen


def main(out,native_export=True):
    out.mkdir(parents=True,exist_ok=True);report={}
    for pose in ("stowed","receive","transport","place"):
        source=PKG/(pose+".step");base=cad.read_named(source)
        for key in REPLACES:del base[key]
        layer=added_shapes(pose)
        if pose=="place" and native_export:layer=export(out/"grip-slide-place.step",layer)
        report[pose]=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),parts=len(layer),
                          exterior=interferences(layer,base),interior=interferences(layer,layer,True))
    # Narrow continuous-motion questions at independent fractions of a tooth and the two
    # mechanical travel stops. Center-rack geometry also checks the native polar convention.
    probes={}
    for s in (0.,.43,1.57,3.14,90.,185.,277.,369.8,370.):
        p={"height":560,"stroke":s}
        shapes=added_shapes(p)
        pairs=(("slide-power-pinion","slide-driven-rack"),("slide-fixed-rack-pinion","slide-fixed-rack"),
               ("slide-inner-rack-pinion","slide-inner-rack"))
        inter={a:round(cad.intersection(shapes[a],shapes[b]),6) for a,b in pairs}
        probes[str(s)]=inter
    report["rack_phase_sweep_mm3"]=probes
    place=PKG/"place.step"
    base=cad.read_named(place)
    for key in REPLACES:del base[key]
    for delta in (-1.5,1.5):
        p=added_shapes("place",{"L":delta,"R":-delta})
        report["opposed_limit_"+str(delta)]={"base":interferences(p,base),"internal":interferences(p,p,True)}
    p=added_shapes("place")
    service={}
    for side,sgn in (("L",1),("R",-1)):
        for label,x in (("rear",-80),("front",140)):
            tool=cad.brep(cylinder(x+370,180*sgn,560-38,AXES["z"],3.5,150))
            before=interferences({"driver":tool},dict(base,**p))
            service[side+"-"+label]=list(before)
    report["M5_vertical_service_blockers"]=service
    report["place_step_sha256"]=hashlib.sha256((out/"grip-slide-place.step").read_bytes()).hexdigest() if native_export else None
    (out/"checks.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--out",type=Path,default=HERE)
    p.add_argument("--no-step",action="store_true");a=p.parse_args();main(a.out,not a.no_step)
