"""Reopen the exact station and robot, inspect outward fiducial faces and cast finite sightlines."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

from pose import CENTER,EYE,SECOND_EYE,HEIGHT,LANDING,POINTS,ROOT,RX,TARGET_Z,WIDTH,axes,image,local
sys.path.insert(0,str(ROOT/"station-supply"))
from native_station import parts,FIRST_SHA,cad
from entry_window import registered_robot
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE,TopAbs_VERTEX
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Trsf,gp_Lin,gp_Dir,gp_Pnt

sys.path.insert(0,str(ROOT/"release-observer"))
import stereo
from check_routing import write_step
from duplex import finalize_step

OCCS=("NAUO1463","NAUO1465","NAUO1466","NAUO1464")
POSES={"nominal":(0,0,0),"positive":(10,10,3),"negative":(-10,-10,-3)}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def face_center(shape):
    far=cad.bounds(shape)[3]
    found=[]
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    while faces.More():
        face=faces.Current()
        b=cad.bounds(face)
        if abs(b[0]-far)<.001 and abs(b[3]-far)<.001:
            data=GProp_GProps()
            BRepGProp.SurfaceProperties_s(face,data)
            points=set()
            ex=TopExp_Explorer(face,TopAbs_VERTEX)
            while ex.More():
                p=BRep_Tool.Pnt_s(TopoDS.Vertex(ex.Current())).Coord()
                points.add(tuple(round(x,6) for x in p))
                ex.Next()
            center=data.CentreOfMass().Coord()
            ordered=sorted(points,key=lambda p:math.atan2(p[2]-center[2],p[1]-center[1]))
            found.append((data.Mass(),center,ordered))
        faces.Next()
    if not found: raise ValueError("No front face")
    return max(found)


def intersects_box(start,end,box):
    lo,hi=0.,1.
    for i in range(3):
        v=end[i]-start[i]
        if abs(v)<1e-10:
            if not box[i]-.01<=start[i]<=box[i+3]+.01: return False
        else:
            a,b=sorted(((box[i]-.01-start[i])/v,(box[i+3]+.01-start[i])/v))
            lo,hi=max(lo,a),min(hi,b)
            if lo>hi: return False
    return True


def sight(objects,start,end):
    delta=[b-a for a,b in zip(start,end)]
    dist=math.hypot(*delta)
    line=gp_Lin(gp_Pnt(*start),gp_Dir(*delta))
    blockers=[]
    for k,(s,b) in objects.items():
        if intersects_box(start,end,b):
            test=IntCurvesFace_ShapeIntersector()
            test.Load(s,1e-5)
            test.Perform(line,.02,dist-.05)
            if test.NbPnt():blockers.append(k)
    return blockers


def transformed(shape,matrix):
    xf=gp_Trsf()
    xf.SetValues(*matrix)
    return BRepBuilderAPI_Transform(shape,xf,False).Shape()


def camera_parts(eye,label):
    # Enclose the 38 mm board and adjustable commercial lens. Optical entrance is the forward face.
    r,d,f=axes(eye)
    # Right/down/forward is right-handed. Local package coordinates x=right,y=down,z=forward.
    mat=tuple(v for i in range(3) for v in (r[i],d[i],f[i],eye[i]))
    case=cad.brep(["box",[-21,-21,-48,42,42,48]])
    casing=transformed(case,mat)
    cx,cy,cz=[e-50*f0 for e,f0 in zip(eye,f)]
    x0=min(-34 if eye[1]>0 else -67,cx-5)
    x1=max(-21,cx+5)
    y0=min(eye[1]-11,cy-5)
    y1=max(eye[1]+11,cy+5)
    foot=cad.brep(["box",[x0,y0,1020,x1-x0,y1-y0,4]])
    post=cad.brep(["box",[cx-4,cy-4,1024,8,8,cz-1024]])
    seat=transformed(cad.brep(["box",[-17,-17,-54,34,34,6]]),mat)
    bracket=cad.BRepAlgoAPI_Fuse(cad.BRepAlgoAPI_Fuse(foot,post).Shape(),seat).Shape()
    bracket=cad.BRepAlgoAPI_Cut(bracket,casing).Shape()
    return {label+"-camera-and-lens-reserve":casing,label+"-pedestal":bracket}


def placed(shape,pose):
    dx,dy,a=pose
    s,c=math.sin(math.radians(a)),math.cos(math.radians(a))
    return transformed(shape,(-c,-s,0,RX+c*(LANDING+dx)+s*dy,
                               s,-c,0,-s*(LANDING+dx)+c*dy,0,0,1,0))


def indexed(shapes):
    return {k:(v,cad.bounds(v)) for k,v in shapes.items()}


def review(source,save):
    field=parts(source)
    robot=registered_robot()
    stops=ROOT/"station-supply/receiver-stops.step"
    corrected=cad.read_named(stops,[{"id":"grip-"+k+"-inboard-hardstop"} for k in ("L","R")])
    robot.update(corrected)
    lookup={k.split("/")[0]:k for k in field}
    station_features=[]
    for occ,p in zip(OCCS,POINTS):
        area,centroid,vertices=face_center(field[lookup[occ]])
        if math.dist(centroid,p)>.002:raise ValueError((occ,centroid,p))
        spans={}
        for name,eye in (("right",EYE),("left",SECOND_EYE)):
            u,v=zip(*(image(vertex,(0,0,0),eye) for vertex in vertices))
            spans[name]=[max(u)-min(u),max(v)-min(v)]
        station_features.append({"occurrence":lookup[occ],"frontFaceAreaMm2":area,"centerStationMm":centroid,
                                 "sampledApparentSpansPx":spans})
    panel=cad.bounds(field[lookup["NAUO1507"]])
    panel_points=[(panel[3],y,TARGET_Z+z) for y in (-100,0,100) for z in (-100,0,100)]
    features=list(POINTS)+panel_points
    installed=indexed(robot)
    add=camera_parts(EYE,"right")|camera_parts(SECOND_EYE,"left")
    mounting={}
    for name,shape in add.items():
        box=cad.bounds(shape)
        overlaps={}
        for key,(src,bb) in installed.items():
            if all(min(box[i+3],bb[i+3])-max(box[i],bb[i])>.001 for i in range(3)):
                volume=cad.intersection(shape,src)
                if volume>.01:overlaps[key]=volume
        mounting[name]=overlaps
    # Nominal legacy unit sees the corner pixels only if each eye clears the image boundary.
    old=[[stereo.project(local(p),eye) for p in POINTS] for eye in (-37.5,37.5)]
    registered={}
    for label,pose in POSES.items():
        volumes=indexed({k:placed(s,pose) for k,s in field.items()})|installed|indexed(add)
        rays=[]
        for label_eye,eye in (("right",EYE),("left",SECOND_EYE)):
            for p in features:
                uv=image(p,pose,eye)
                rays.append({"eye":label_eye,"pixel":uv,"inFrame":0<=uv[0]<WIDTH and 0<=uv[1]<HEIGHT,
                             "blockers":sight(volumes,eye,local(p,*pose))})
        registered[label]=rays
    result={"field":FIRST_SHA,"robotBodyCount":len(robot),"stationOccurrences":len(field),"stopStep":sha(stops),
            "entryStudy":sha(ROOT/"station-supply/entry-window.json"),"mounts":mounting,"boltFaces":station_features,
            "panelCadBounds":panel,"legacyPixels":old,"candidateRays":registered}
    if save:
        write_step(save,add)
        finalize_step(save)
        restored=cad.read_named(save,[{"id":k} for k in add])
        result["originalReserveStep"]={"sha256":sha(save),"solids":sum(cad.solid_count(s) for s in restored.values())}
    return result


if __name__=="__main__":
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("--field",type=Path,required=True)
    a.add_argument("--step",type=Path)
    a.add_argument("--output",type=Path,required=True)
    p=a.parse_args()
    p.output.write_text(json.dumps(review(p.field,p.step),indent=2)+"\n",encoding="utf-8",newline="\n")
