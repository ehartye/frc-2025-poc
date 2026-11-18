"""Editable packaging study of the selected screw pair and its mounting interfaces."""
import argparse
import json
from pathlib import Path
import sys

from native import HERE,HEIGHT,SCREW_R,NUT_R,NUT_LEN,cad,contact,cylinder,digest,relieved_plate

sys.path.insert(0,str(HERE.parent/"receiver-lift"))
from check_routing import write_step
from duplex import finalize_step


def ring(x,y,z,ro,ri,d):
    exterior=cylinder(x,y,z,ro,d)
    drill=cylinder(x,y,z-1,ri,d+2)
    op=cad.BRepAlgoAPI_Cut(exterior,drill)
    if not op.IsDone():
        raise ValueError("Bore failed")
    return op.Shape()


def add_shapes(height):
    result={}
    for tag,sign in (("L",1),("R",-1)):
        x,y=-142,255*sign
        center=height-140
        # Threads are pitch-independent outside/inside fit envelopes.
        # For assembly-side collision, native.py separately uses full solid cylinders.
        parts={"working-sleeve":ring(x,y,center-NUT_LEN/2,NUT_R,SCREW_R+.05,NUT_LEN),
               "safety-sleeve":ring(x,y,center+NUT_LEN/2+2,NUT_R,SCREW_R+.05,NUT_LEN),
               "spindle":cylinder(x,y,365,SCREW_R,520),
               "lower-support-reserve":ring(x,y,383,19,SCREW_R+.1,12),
               "upper-thrust-reserve":ring(x,y,873,19,SCREW_R+.1,12)}
        # Foot touches the relieved carriage plate at X=-98 on its inboard extension.
        # A standardized final carrier needs sized fasteners, axial and torque reaction.
        for name,z in (("working",center-13),("safety",center+29)):
            collar=ring(x,y,z,20,NUT_R+.10,8)
            strap=cad.brep(["box",[x+17,y-6,z,27,12,8]])
            parts[name+"-arm-reserve"]=cad.BRepAlgoAPI_Fuse(collar,strap).Shape()
        result.update({tag+"-"+key:s for key,s in parts.items()})
    return result


def review(folder,pose,path=None):
    meta=json.loads((folder/"part-frames.json").read_bytes())
    src=folder/(pose+".step")
    sha=digest(src)
    if sha!=json.loads((folder/"review.json").read_bytes())[pose]["export_sha256"]:
        raise ValueError("Unexpected registered robot")
    shapes=cad.read_named(src,[{"id":k} for k in meta])
    h=HEIGHT[pose]
    for sign,tag in ((1,"L"),(-1,"R")):
        k="backup-"+tag+"-carriage-plate"
        shapes[k]=relieved_plate(shapes[k],sign,h)
    bounds={k:cad.bounds(v) for k,v in shapes.items()}
    built=add_shapes(h)
    hits={k:contact(v,shapes,bounds) for k,v in built.items()}
    # The four mounting reserves are disconnected at their equipment-side interfaces.
    # Exporting them locates the outstanding load path. They are not an assembly proof.
    result={"pose":pose,"base":sha,"fixedAndMovingNativeOverlapsMm3":{k:v for k,v in hits.items() if v},
            "count":len(built),"loadPathToFrameVerified":False}
    result["rejectedLowerSupportAt365Mm"]={tag:contact(ring(-142,sgn*255,365,19,SCREW_R+.1,12),shapes,bounds)
                                          for sgn,tag in ((1,"L"),(-1,"R"))}
    if pose=="place":
        fixed={k:v for k,v in shapes.items() if meta[k]=="base"}
        result["simultaneousDropMm"]={}
        result["workingSleeveToLowerSupport"]={}
        for drop in (5.0,5.95,6.0,8.0):
            row={}
            for tag in ("L","R"):
                shifted=cad.translated(built[tag+"-working-sleeve"],(0,0,-drop))
                lower=built[tag+"-lower-support-reserve"]
                native=cad.BRepAlgoAPI_Common(shifted,lower)
                if not native.IsDone():
                    raise ValueError("Nut and bearing intersection failed")
                row[tag]=abs(cad.volume(native.Shape())[0])
            result["workingSleeveToLowerSupport"][str(drop)]=row
        for drop in (5,8,9,10,12):
            test={}
            for tag in ("L","R"):
                plate=shapes["backup-"+tag+"-carriage-plate"]
                test[tag]=contact(cad.translated(plate,(0,0,-drop)),fixed,bounds)
            result["simultaneousDropMm"][str(drop)]=test
    if path:
        out=built.copy()
        out.update({tag+"-relieved-plate":shapes["backup-"+tag+"-carriage-plate"] for tag in ("L","R")})
        write_step(path,out)
        finalize_step(path)
        reopened=cad.read_named(path,[{"id":k} for k in out])
        if any(cad.solid_count(s)!=1 for s in reopened.values()):
            raise ValueError("Invalid reopened solid")
        result["export"]={"sha":digest(path),"namedSolids":len(reopened)}
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("pose",choices=HEIGHT)
    p.add_argument("output",type=Path)
    p.add_argument("--step",type=Path)
    a=p.parse_args()
    red=HERE.parent/"competition-package"/"shop-duplex"/"red"
    a.output.write_text(json.dumps(review(red,a.pose,a.step),indent=2)+"\n",encoding="utf-8",newline="\n")
