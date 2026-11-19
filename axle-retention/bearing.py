"""Register the exact supplier bearing against the selected thrust faces."""
import argparse
import json
import math
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from fit import NOSE_HEIGHT, NOSE_IN, NOSE_OUT, STATIONS, THRUST_LENGTH, cad, digest, distance, ring
from keepers import INNER, RELIEF

BEARING_SHA = "3e16cb3f5f215288b8706eb8228d9580fea78c425eee536ef95195b5b98f02e6"
HALF_WIDTH = 3.96875
LAND = (7.129,9.525)
RUNOUT = .20


def installed(source,x,y,sign):
    transform = gp_Trsf()
    transform.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,0,0)),sign*math.pi/2)
    part = BRepBuilderAPI_Transform(source,transform,False).Shape()
    return cad.translated(part,[x,y-sign*HALF_WIDTH,76.2])


def run(source_path,kit_path):
    if digest(source_path)!=BEARING_SHA:
        raise ValueError("Supply the selected REV-21-1916 native model")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source_path))!=IFSelect_RetDone or not reader.TransferRoots():
        raise ValueError("Could not import bearing")
    source = reader.OneShape()
    if cad.solid_count(source)!=1:
        raise ValueError("Unexpected bearing representation")
    names = {f"axle-{p}-{s}" for p in STATIONS for s in ("L","R")}
    names.update("spacer-center-3-"+s for s in ("L","R"))
    names.update(f"{k}-{s}" for k in ("clip-sweep","clip-thrust") for s in ("L","R"))
    tags=[(p,s,f) for p in STATIONS for s in ("L","R") for f in ("inner","outer") if p!="center" or f=="outer"]
    for p,s,f in tags:
        for family in ("retaining-screw","select-shim"):
            names.add(f"{family}-{p}-{s}-{f}")
    kit=cad.read_named(kit_path,[{"id":k} for k in names])
    keeper_note=json.loads(kit_path.with_name("keepers.json").read_bytes())
    keeper_path=kit_path.with_name("bearing-keepers.step")
    if digest(keeper_path)!=keeper_note["stepSha256"]:
        raise ValueError("Stationary keeper exchange changed")
    keeper_kit=cad.read_named(keeper_path,[{"id":k} for k in keeper_note["volumeMm3"]])
    checks={}
    for p,side,face in tags:
        x = STATIONS[p]
        sy = 1 if side=="L" else -1
        sign = sy if face=="outer" else -sy
        y = sy*(325.35 if face=="outer" else 234.65)
        bearing = installed(source,x,y,sign)
        tag = f"{p}-{side}-{face}"
        trials=[("shim",kit["select-shim-"+tag],y,sign,.1)]
        if p=="center":
            flange_face = y-sign*2*HALF_WIDTH
            trials.append(("clip-thrust",kit["clip-thrust-"+side],flange_face,-sign,
                           abs(flange_face-sy*(315.18+THRUST_LENGTH))))
        for role,part,face_y,normal,gap in trials:
            nose = ring(x,face_y+normal*gap,normal,NOSE_HEIGHT,NOSE_IN,NOSE_OUT)
            gap_measured = distance(nose,bearing)
            # Only the thrust face carries the runout budget. Radial relocation of
            # the entire shaft would incorrectly drive its rounded hex into the bore.
            stopped = cad.translated(part,[0,-normal*(gap+.05),0])
            actual = cad.intersection(stopped,bearing)
            expected = math.pi*(NOSE_OUT**2-NOSE_IN**2)*.05
            late = []
            for angle in range(0,360,45):
                phi = math.radians(angle)
                eccentric = cad.translated(nose,[RUNOUT*math.cos(phi),-normal*(gap+.05),RUNOUT*math.sin(phi)])
                late.append(cad.intersection(eccentric,bearing))
            free = cad.intersection(cad.translated(part,[0,-normal*(gap-.035),0]),bearing)
            if (abs(gap_measured-gap)>.001 or free>.001 or abs(actual-expected)>.001
                    or max(abs(v-expected) for v in late)>.001):
                raise ValueError(f"Unexpected stop or contact area at {tag} {role}: {gap_measured} {free} {actual} {late}")
            checks[tag+"-"+role]={"gapMm":gap_measured,"beforeStopCommonMm3":free,
                                  "pastStopCommonMm3":actual,"eccentricContactRangeMm3":[min(late),max(late)]}
    fixed={}
    for pos,x in STATIONS.items():
        for side,sy in (("L",1),("R",-1)):
            for face,ds in (("outer",1),("inner",-1)):
                sign=sy*ds
                y=sy*(325.35 if face=="outer" else 234.65)
                b=installed(source,x,y,sign)
                tag=f"{pos}-{face}-{side}"
                cap=keeper_kit["bearing-keeper-"+tag]
                gap=distance(cap,b)
                negative=cad.translated(b,[0,-sign*(gap+.05),0])
                around=[]
                for angle in range(0,360,45):
                    a=math.radians(angle)
                    eccentric=cad.translated(cap,[.25*math.cos(a),0,.25*math.sin(a)])
                    around.append(cad.intersection(eccentric,negative))
                if not (.05<gap<.07 and min(around)>8):
                    raise ValueError("Outer-rim restraint failed: "+tag)
                fixed[tag]={"gapMm":gap,"eccentricPastStopVolumeRangeMm3":[min(around),max(around)]}
    return {"supplierSha256":BEARING_SHA,"kitSha256":digest(kit_path),"keeperSha256":digest(keeper_path),
            "seatRadiiMm":LAND,"noseRadiiMm":[NOSE_IN,NOSE_OUT],
            "allowedRadialEccentricityMm":RUNOUT,"remainingInnerOuterMarginMm":[NOSE_IN-RUNOUT-LAND[0],LAND[1]-NOSE_OUT-RUNOUT],
            "areaMm2":math.pi*(NOSE_OUT**2-NOSE_IN**2),"checks":checks,"keeperChecks":fixed,
            "keeperAt025RunoutMarginsMm":{"toShieldRadial":INNER-.25-12.700,
                                           "flangeToPocketRadial":RELIEF-.25-15.5575}}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("supplier_step","kit_step","output"):
        parser.add_argument(name,type=Path)
    a=parser.parse_args()
    a.output.write_text(json.dumps(run(a.supplier_step,a.kit_step),indent=2)+"\n",encoding="utf-8",newline="\n")
