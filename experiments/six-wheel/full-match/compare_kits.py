"""Compare complete red and blue assemblies by named, reopened native geometry."""
import argparse
import hashlib
import io
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"design/station-l1"))
import build_assembly as cad
from OCP.BRepTools import BRepTools


def native_id(s):
    stream=io.BytesIO()
    BRepTools.Write_s(s,stream)
    return hashlib.sha256(stream.getvalue()).digest()


def load(folder,pose,color):
    path=folder/(pose+".step")
    registry=folder if (folder/"review.json").is_file() else folder.parent
    meta=json.loads((registry/"review.json").read_text())
    sha=hashlib.sha256(path.read_bytes()).hexdigest()
    if sha!=meta[pose]["export_sha256"]:
        raise ValueError("Selected pose differs from integration review")
    frames=json.loads((registry/"part-frames.json").read_text())
    inventory=cad.read_named(path,[{"id":k} for k in frames])
    return sha,{k.removesuffix("-"+color):v for k,v in inventory.items()}


def compare(red,blue,expected_count=401):
    result={}
    for pose in ("stowed","receive","transport","place"):
        rs,rr=load(red,pose,"red")
        bs,bb=load(blue,pose,"blue")
        if len(rr)!=expected_count or set(rr)!=set(bb):
            raise ValueError("Missing corresponding named body on one alliance")
        equal=0
        fallback={}
        for key,a in rr.items():
            b=bb[key]
            if native_id(a)==native_id(b):
                equal+=1
            else:
                va,vb=cad.volume(a)[0],cad.volume(b)[0]
                common=cad.intersection(a,b)
                fallback[key]=max(abs(va-common),abs(vb-common))
        if any(v>.05 for v in fallback.values()):
            raise ValueError("A physical alliance-kit difference requires an independent field sweep")
        result[pose]={"red_step_sha256":rs,"blue_step_sha256":bs,"body_count":len(rr),
                      "identical_native_serializations":equal,"fallback_boolean_count":len(fallback),
                      "maximum_remaining_material_difference_mm3":max(fallback.values(),default=0)}
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("red","blue","output"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--body-count",type=int,default=401)
    a=p.parse_args()
    a.output.write_text(json.dumps(compare(a.red,a.blue,a.body_count),indent=2)+"\n",encoding="utf8",newline="\n")
