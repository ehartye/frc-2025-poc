"""Screen the actual axis-aligned receiver travel between the four registered poses."""
import argparse
from itertools import combinations
import json
from pathlib import Path

from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Vec
from build import flatten, native, pose_layers, rotation_bounds

VOLUME_TOLERANCE=0.001


def translation(name,frame,start,end,turns):
    if frame=="variable-rope" and start[0]!=end[0]:
        return None
    if name in turns and start[1]!=end[1]:
        return None
    if frame=="variable-draw-wire" and start[1]!=end[1]:
        return None
    dx=(end[1]-start[1])*{"base":0,"variable-rope":0,"variable-draw-wire":0,"lift":0,"middle":0.5,"slide":1}[frame]
    return (dx,0 if frame=="base" else end[0]-start[0])


def swept_prism(shape,delta):
    """For a true X-prismatic solid, sweep its real end profile over its relative travel."""
    box=native.bounds(shape)
    length=box[3]-box[0]
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    while faces.More():
        face=TopoDS.Face(faces.Current())
        faces.Next()
        face_box=native.bounds(face)
        if max(abs(face_box[k]-box[0]) for k in (0,3))>1e-6:
            continue
        original=BRepPrimAPI_MakePrism(face,gp_Vec(length,0,0)).Shape()
        common=native.intersection(original,shape)
        if max(abs(native.volume(part)[0]-common) for part in (original,shape))>1e-6:
            continue
        extended=BRepPrimAPI_MakePrism(face,gp_Vec(length+abs(delta),0,0)).Shape()
        return native.translated(extended,(min(0,delta),0,0))
    return None


def paths():
    poses=native.MODEL["poses"]
    high,low=poses["receive"]["height"],poses["place"]["height"]
    return {
        "station-extension":((high,0),(high,poses["receive"]["stroke"])),
        "lift-with-slide-retracted":((low,0),(high,0)),
        "target-extension":((low,0),(low,poses["place"]["stroke"])),
    }


def swept_bounds(name,shape,frame,start,end,turns):
    """Enclose the entire segment, including changing line length and gear angle."""
    if name in turns:
        x,y,z,xx,yy,zz=turns[name]["bounds_mm"]
        dx=start[1]*{"middle":0.5,"lift":0}[frame]
        b=(x+dx,y,z+start[0],xx+dx,yy,zz+start[0])
    else:
        b=native.bounds(shape)
    dx=(end[1]-start[1])*{"base":0,"lift":0,"variable-rope":0,"variable-draw-wire":1,"middle":0.5,"slide":1}[frame]
    dz=0 if frame=="base" else end[0]-start[0]
    # Only the lower end of each return line changes with carriage height.
    return (b[0]+(0 if frame=="variable-draw-wire" else min(0,dx)),b[1],b[2]+min(0,dz),b[3]+max(0,dx),b[4],
            b[5]+(0 if frame=="variable-rope" else max(0,dz)))


def screen():
    report={}
    turns=rotation_bounds()
    for leg,(start,end) in paths().items():
        layers,_,_,frames=pose_layers(*start)
        shapes=flatten(layers)
        moves={name:translation(name,frame,start,end,turns) for name,frame in frames.items()}
        bounds={name:swept_bounds(name,shape,frames[name],start,end,turns)
                for parts in layers.values() for name,shape in parts.items()}
        exact={}
        unknown=[]
        equal=0
        rejected=0
        reference_conflicts={}
        prisms={}
        for first,second in combinations(layers.values(),2):
            for a in first:
                for b in second:
                    if not native.occupied(bounds[a],bounds[b]):
                        rejected+=1
                        continue
                    # Equal translations preserve the already verified relative shape.
                    if moves[a] is not None and moves[a]==moves[b]:
                        equal+=1
                        if native.occupied(native.bounds(first[a]),native.bounds(second[b])):
                            volume=native.intersection(first[a],second[b])
                            if volume>VOLUME_TOLERANCE:
                                reference_conflicts[a+" / "+b]=round(volume,8)
                        continue
                    key=a+" / "+b
                    if moves[a] is not None and moves[b] is not None and moves[a][1]==moves[b][1]:
                        for mover,stationary in ((a,b),(b,a)):
                            delta=moves[mover][0]-moves[stationary][0]
                            if (mover,delta) not in prisms:
                                prisms[mover,delta]=swept_prism(shapes[mover],delta)
                            swept=prisms[mover,delta]
                            if swept is not None:
                                exact[key]=round(native.intersection(swept,shapes[stationary]),8)
                                break
                    if key not in exact:
                        unknown.append(key)
        report[leg]={"start":start,"end":end,"disjoint_swept_boxes":rejected,"constant_relative_pose":equal,
                     "reference_conflicts_mm3":reference_conflicts,"exact_swept_common_mm3":exact,"unresolved":unknown}
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    report=screen()
    args.output.write_text(json.dumps(report,indent=2)+"\n")
    if any(r["unresolved"] or r["reference_conflicts_mm3"] or any(v>VOLUME_TOLERANCE for v in r["exact_swept_common_mm3"].values()) for r in report.values()):
        raise SystemExit(1)
