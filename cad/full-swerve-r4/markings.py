"""Place the four project-owned white 1780 markings on the issued red cloth."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Pnt, gp_Vec
from OCP.TopLoc import TopLoc_Location
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/"cad/full-swerve-r3"))
import build as base

CONFIG="fixed-low-straight-arm-swerve-R4-stowed-red-1780"
# The 2024 project's original Fusion sketch contours, in mm, give the 1 a foot
# and preserve both counters in the 8. Each marking reads rightwards from outside.
GLYPHS=[
    ([(15,80),(30,105),(45,105),(45,15),(60,15),(60,0),(15,0),(15,15),(30,15),(30,80)],[]),
    ([(0,105),(60,105),(60,90),(25,0),(8,0),(43,90),(0,90)],[]),
    ([(0,0),(60,0),(60,105),(0,105)], [[(15,15),(45,15),(45,45),(15,45)],[(15,60),(45,60),(45,90),(15,90)]]),
    ([(0,0),(60,0),(60,105),(0,105)], [[(15,15),(45,15),(45,90),(15,90)]]),
]
# Surface normal, reading direction, center of label, and physical cover under it.
SITES={
    "front":((1,0),(0,1),(425,0),"front"),
    "rear":((-1,0),(0,-1),(-425,0),"rear"),
    "left":((0,1),(-1,0),(200,425),"front"),
    "right":((0,-1),(1,0),(-200,-425),"rear"),
}
HEIGHT=105.
BASE_Z=59.5
INK_MM=.05
PITCH=80.

def prism(outline,n,r,center,index):
    points=BRepBuilderAPI_MakePolygon()
    for u,v in outline:
        u=u+PITCH*index-150
        points.Add(gp_Pnt(center[0]+u*r[0],center[1]+u*r[1],BASE_Z+v))
    points.Close()
    face=BRepBuilderAPI_MakeFace(points.Wire()).Face()
    return BRepPrimAPI_MakePrism(face,gp_Vec(n[0]*INK_MM,n[1]*INK_MM,0)).Shape()

def generate(rects):
    parts={}
    evidence={}
    for side,(n,right,center,cover) in SITES.items():
        cloth=rects[f"bumper-cover-{cover}-red"]
        for index,(outline,holes) in enumerate(GLYPHS):
            glyph=prism(outline,n,right,center,index)
            for contour in holes:
                glyph=BRepAlgoAPI_Cut(glyph,prism(contour,n,right,center,index)).Shape()
            key=f"bumper-lettering-{side}-{index+1}-{'1780'[index]}"
            if base.cad.solid_count(glyph)!=1:raise ValueError(key)
            vol=base.cad.volume(glyph)[0]
            inside=base.cad.translated(glyph,[-INK_MM*n[0],-INK_MM*n[1],0])
            overlap=base.cad.volume(BRepAlgoAPI_Common(cloth,inside).Shape())[0]
            penetration=base.cad.volume(BRepAlgoAPI_Common(cloth,glyph).Shape())[0]
            if overlap/vol<.9999 or penetration>.0001:raise ValueError((key,overlap/vol,penetration))
            parts[key]=glyph
            evidence[key]={"cover":f"bumper-cover-{cover}-red","outwardNormal":list(n)+[0],
                           "readerRight":list(right)+[0],"baseZMm":BASE_Z,"topZMm":BASE_Z+HEIGHT,
                           "thicknessMm":INK_MM,"fullAreaBackingFraction":overlap/vol,
                           "nominalHardBodyOverlapMm3":penetration,"color":"#FFFFFF","massBasis":"surface marking"}
    return parts,evidence

def baked(shape):
    # Remove imported top-level placements, then apply the world transform to
    # the BRep itself so Fusion keeps each source label on its actual body.
    loc=shape.Location()
    if loc.IsIdentity():return shape
    solid=BRepBuilderAPI_Transform(shape.Located(TopLoc_Location()),loc.Transformation(),True).Shape()
    if not solid.Location().IsIdentity():raise ValueError("World-coordinate bake failed")
    if max(abs(a-b) for a,b in zip(base.cad.bounds(shape),base.cad.bounds(solid)))>.0001:raise ValueError("Changed geometry")
    return solid

def run(private):
    old=ROOT/"cad/full-swerve-r3"
    source=old/"full-robot.step"
    previous=base.read(old/"appearances.json")
    if base.sha(source)!=previous["sourceSha256"]:raise ValueError("Unrecognized R3")
    reference=base.named(source)
    objects={}
    entries={r["id"]:r for r in previous["solids"]}
    roster={}
    for key,shape in reference.items():
        row=entries[key]
        solids=list(base.subsolids(shape))
        if len(solids)!=1:raise ValueError("R3 solid register mismatch: "+key)
        objects[key]=(baked(solids[0]),row["source"],row["representation"],False)
        roster[key]=row["role"]
    ink,evidence=generate(reference)
    for key,shape in ink.items():
        objects[key]=(shape,"cad/full-swerve-r4/markings.py","appearance-overlay",False)
        roster[key]="bumper-lettering"
    resolved=base.resolve_project_palette(ROOT)
    # The named writer accepts explicit per-solid roles before consulting source conventions.
    colors=resolved["colors"]
    output=HERE/"full-robot.step"
    rows=base.write_step(output,objects,colors,roster)
    base.save(HERE/"appearances.json",{"schema":"agent-frc/assembly-appearances@1","sourceSha256":base.sha(output),
        "configuration":CONFIG,"units":"mm","solids":rows})
    base.save(HERE/"lettering.json",{"source2024":"https://github.com/ehartye/frc-2024-poc/blob/main/docs/media/robot-native.glb",
        "sourceNode":"fixed-number","sRGB":"#FFFFFF","solidCount":len(ink),"previousSourceSha256":previous["sourceSha256"],
        "registeredSourceSha256":base.sha(output),"glyphPitchMm":PITCH,"rightHandedFrame":"X forward, Y left, Z up","impressions":evidence})
    private.mkdir(exist_ok=True,parents=True)
    base.save(private/"effective-palette.json",resolved)
    native={k:v for k,v in objects.items() if not v[2].startswith("public-")}
    base.write_step(private/"colored-project.step",native,colors,roster)
    print(len(rows),len(native),base.sha(output),flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("private",type=Path)
    a=p.parse_args()
    run(a.private)

