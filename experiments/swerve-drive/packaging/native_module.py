"""Read the external manufacturer's STEP with registered native assembly occurrence transforms."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from OCP.TopLoc import TopLoc_Location

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]/"six-wheel"
sys.path.insert(0,str(ROOT/"mission"))
from check_branch_exclusion import cad,direct_children,name
PIN="11f8381810846a2e52a23dbbe07f8271b314967624eee53f56e59a3ece05c75e"
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):
    if sha(path)!=PIN:raise ValueError("Unexpected module revision")
    doc,table=cad.xcaf_doc();reader=cad.STEPCAFControl_Reader()
    if reader.ReadFile(str(path))!=cad.IFSelect_RetDone or not reader.Transfer(doc):raise ValueError("Invalid module")
    roots=cad.Sequence_TDF_Label();table.GetFreeShapes(roots)
    if roots.Length()!=1:raise ValueError("Ambiguous module root")
    shapes={}
    def walk(ref,location,prefix):
        children=list(direct_children(table,ref))
        if not children:shapes[prefix]=table.GetShape_s(ref).Moved(location);return
        for i,(occ,child) in enumerate(children):
            walk(child,location.Multiplied(table.GetLocation_s(occ)),prefix+"/"+str(i)+":"+name(child))
    walk(roots.Value(1),TopLoc_Location(),name(roots.Value(1)))
    return shapes
def measure(src,dst):
    shapes=read(src);data={}
    for key,s in shapes.items():
        v,c=cad.volume(s);data[key]={"boundsMm":cad.bounds(s),"centerMm":c,"volumeMm3":v}
    dst.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8")
if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("module",type=Path);p.add_argument("out",type=Path)
    a=p.parse_args();measure(a.module,a.out)
