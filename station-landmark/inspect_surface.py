"""Reopen the station apertures and the two selected bumper covers at the reoriented legal start."""
import argparse
import json
import math
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import parts,cad,FIRST_SHA
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_WIRE
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf
sys.path.insert(0,str(ROOT/"moving-harness"))
from continuous_screen import read_shop,SHOP_STEP
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.gp import gp_Pnt


def children(shape,typ):
    explorer=TopExp_Explorer(shape,typ)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def audit(field,artifacts):
    station=parts(field)
    key,part=next((k,v) for k,v in station.items() if k.endswith("/GE-25008"))
    apertures=[]
    for face in children(part,TopAbs_FACE):
        bounds=cad.bounds(face)
        if abs(bounds[0])>.001 or abs(bounds[3])>.001:continue
        for wire in children(face,TopAbs_WIRE):
            b=cad.bounds(wire)
            if abs((b[1]+b[4])/2)>400 or b[4]-b[1]>30:continue
            apertures.append([round((b[1]+b[4])/2,4),round((b[2]+b[5])/2,4),
                              round(b[4]-b[1],4),round(b[5]-b[2],4)])
    if len(apertures)!=12:raise ValueError("Expected the two inner and outer columns at three heights")
    x,y=393.375*25.4,158.5*25.4
    angle=math.atan2(248-158.5,440-393.375)
    source,_=read_shop(artifacts)
    strip=(690.875-144-65.5-88)
    # Erode by a 5 mm starting survey and 0.2 deg orientation about the chassis center.
    error=5+2*math.hypot(425,425)*math.sin(math.radians(.2)/2)
    band=BRepPrimAPI_MakeBox(gp_Pnt((strip-2)*25.4+error,0,-10),
                            gp_Pnt(strip*25.4-error,317*25.4,1070)).Shape()
    transform=gp_Trsf()
    c,s=math.cos(angle),math.sin(angle)
    transform.SetValues(c,-s,0,x,s,c,0,y,0,0,1,0)
    volumes={}
    for k,v in source.items():
        if k.startswith("bumper-cover-"):
            volumes[k]=cad.intersection(BRepBuilderAPI_Transform(v,transform,False).Shape(),band)
    return {"fieldSha256":FIRST_SHA,"shopSha256":SHOP_STEP,"occurrence":key,"nativeBoxMm":cad.bounds(part),
            "frontWireCenterYzAndSizeMm":sorted(apertures),"startXYin":[393.375,158.5],
            "startHeadingDeg":math.degrees(angle),"startingTapeErosionMm":error,"startTapeCommonMm3":volumes}


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("field_step",type=Path);cli.add_argument("artifacts",type=Path)
    a=cli.parse_args()
    (HERE/"native.json").write_text(json.dumps(audit(a.field_step,a.artifacts),indent=2)+"\n",encoding="utf8",newline="\n")
