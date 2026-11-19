"""Reopen all submitted poses and cross-check the editable source in a second kernel."""
import argparse
import json
import subprocess
import tempfile
from pathlib import Path
from drive_parts import parts
from check_package import audit
from build_assembly import bounds,read_named,volume


def check_axle(review,openscad):
    name="axle-center-L"
    source=read_named(review/"stowed.step",parts())[name]
    with tempfile.TemporaryDirectory() as temp:
        stl=Path(temp)/"axle.stl"
        subprocess.run([str(openscad),"-o",str(stl),"-D",f'selected_part="{name}"',
                        str((review/"station-l1.scad").resolve())],capture_output=True,check=True,timeout=120)
        vertices=[tuple(map(float,row.split()[1:])) for row in stl.read_text().splitlines()
                  if row.split()[:1]==["vertex"]]
    extent=[f(v[i] for v in vertices) for f in (min,max) for i in range(3)]
    exact=bounds(source)
    faceted=sum((a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+
                 a[2]*(b[0]*c[1]-b[1]*c[0]))/6 for a,b,c in zip(vertices[::3],vertices[1::3],vertices[2::3]))
    exact_volume=volume(source)[0]
    if max(abs(a-b) for a,b in zip(exact,extent))>.04 or abs(faceted/exact_volume-1)>.01:
        raise ValueError("Editable rounded hex axle differs from native geometry")
    return {"part":name,"extentMm":extent,"stepVolumeMm3":exact_volume,"facetedVolumeMm3":faceted}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--review",type=Path,default=Path(__file__).resolve().parent/"review")
    parser.add_argument("--plugin",required=True,type=Path)
    parser.add_argument("--openscad",required=True,type=Path)
    args=parser.parse_args()
    evidence=audit(args.review,args.plugin,args.openscad,parts())
    evidence["roundedHexAxle"]=check_axle(args.review,args.openscad)
    args.output.write_text(json.dumps(evidence,indent=2)+"\n")
