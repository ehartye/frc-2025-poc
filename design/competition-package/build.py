"""Combine the design layers at the four named carriage poses and export their CAD surfaces."""
import argparse
import base64
import gzip
import hashlib
import json
import math
import sys
from functools import cache
from itertools import combinations
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / "station-l1"
LIFT = HERE.parent / "receiver-lift"
OBSERVER = ROOT / "release-observer"
DRIVE = HERE.parent / "drivetrain"
for directory in [BASE, LIFT, OBSERVER, DRIVE, HERE.parent/"receiver-drives", HERE.parent/"electrical-inspection"]:
    sys.path.insert(0, str(directory))

import build_assembly as native
import receiver_drives
import package as electrical
from drive_parts import parts as drive_parts
import check_routing
import drive_parts as drive_model
import mechanical
from check_routing import routing, clashes, write_step
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_register():
    """Record the exact geometry inputs without writing local checkout paths."""
    generators=[native, mechanical, drive_model, check_routing, receiver_drives, electrical]
    files=[(f"design/{Path(module.__file__).parent.name}/{Path(module.__file__).name}",
            Path(module.__file__)) for module in generators]
    inputs=[BASE/"assembly.json", BASE/"b4-source.json", DRIVE/"review/review.json",
            OBSERVER/"mount-check.json", OBSERVER/"stereo-mount.step", Path(__file__), HERE/"viewer.template.html"]
    return dict(sorted((name,sha(path)) for name,path in files+[(p.relative_to(ROOT).as_posix(),p) for p in inputs]))


@cache
def registered_observer():
    manifest = json.loads((OBSERVER/"mount-check.json").read_text())
    path = OBSERVER/"stereo-mount.step"
    if sha(path) != manifest["mount_sha256"]:
        raise ValueError("The stereo mount differs from the verified export")
    reader=STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone or reader.TransferRoots()!=len(manifest["part_order"]):
        raise ValueError("The observer STEP roots do not match the mount inventory")
    return {name:reader.Shape(i) for i,name in enumerate(manifest["part_order"],1)}


def triangulate(shape):
    """Store indexed vertices to 0.1 mm for the viewer. All engineering checks use native solids."""
    BRepMesh_IncrementalMesh(shape,0.7,False,0.25,False)
    faces=TopExp_Explorer(shape,TopAbs_FACE)
    vertices, indices, seen = [], [], {}
    while faces.More():
        face=TopoDS.Face(faces.Current())
        location=TopLoc_Location()
        mesh=BRep_Tool.Triangulation_s(face,location)
        if mesh:
            face_nodes=[]
            for i in range(1, mesh.NbNodes()+1):
                p=mesh.Node(i).Transformed(location.Transformation())
                point=tuple(round(value*10) for value in p.Coord())
                if point not in seen:
                    seen[point]=len(vertices)//3
                    vertices.extend(point)
                face_nodes.append(seen[point])
            for i in range(1,mesh.NbTriangles()+1):
                sequence=mesh.Triangle(i).Get()
                if face.Orientation()==TopAbs_REVERSED: sequence=sequence[::-1]
                tri=[face_nodes[j-1] for j in sequence]
                if len(set(tri))==3: indices.extend(tri)
        faces.Next()
    if not indices: raise ValueError("An occupied body has no drawable surface")
    return [vertices, indices]


def category(part):
    group = part["group"]
    if group in {"wheel","drive","chain","pulley","shaft","bearing"}: return "Drive"
    if part["frame"]!="base": return "Receiver"
    if group in {"power","compute","battery"}: return "Electrical"
    return "Chassis"


@cache
def source_drive(pose):
    source=DRIVE/"review"/(pose+".step")
    receipt=json.loads((DRIVE/"review/review.json").read_text())["poses"][pose]
    if sha(source)!=receipt["stepSha256"]: raise ValueError("Unverified drive pose")
    inventory=drive_parts()
    return native.read_named(source,inventory), {p["id"]:p for p in inventory}


def pose_layers(height_mm, stroke_mm):
    if not (math.isfinite(height_mm) and 560<=height_mm<=940 and math.isfinite(stroke_mm) and 0<=stroke_mm<=370):
        raise ValueError("Carriage position is outside the registered range")
    desc={"height":height_mm,"stroke":stroke_mm}
    poses=native.MODEL["poses"]
    source_pose=next((name for name,p in poses.items() if p==desc),"stowed")
    observed, info=source_drive(source_pose)
    previous=poses[source_pose]
    families={"Drive and chassis":{}, "Lift":routing(height_mm), "Sensing":{},
              "Receiver drive":receiver_drives.added_shapes(desc), "Electrical":electrical.electrical_parts(desc)}
    groups,colors,frames={},{},{}
    retained={p["id"] for p in native.PARTS}
    replaced={"lift-drive"} | receiver_drives.REPLACES | electrical.REPLACED
    if missing:=replaced-observed.keys():
        raise ValueError(f"Missing original bodies designated for replacement: {sorted(missing)}")
    for name,shape in observed.items():
        if name in replaced: continue
        part=info[name]
        if previous!=desc and part["frame"]!="base":
            start=native.offset(part,previous)
            delta=[b-a for a,b in zip(start,native.offset(part,desc))]
            shape=native.translated(shape,delta)
        families["Drive and chassis"][name]=shape
        groups[name]=category(part) if name in retained else ("Electrical" if part["group"]=="power" else "Drive")
        colors[name]=native.COLORS[part["group"]]
        frames[name]=part["frame"]
    for name in families["Lift"]:
        groups[name],colors[name]="Lift","#d47d3d"
        frames[name]="variable-rope" if name.startswith("return-line") else "lift" if name.startswith("anchor-tab") else "base"
    for key,shape in registered_observer().items():
        name="observer-"+key
        moving=key.startswith("moving-pad-")
        if moving: shape=native.translated(shape,(stroke_mm-370,0,height_mm-560))
        families["Sensing"][name]=shape
        groups[name],colors[name],frames[name]="Sensing","#378985","slide" if moving else "base"
    for name,(frame,kind,_) in receiver_drives.parts().items():
        groups[name],colors[name]="Receiver","#99763c" if "pinion" in name or "rack" in name else "#5c7e69"
        frames[name]="variable-draw-wire" if kind=="measurement-line" else frame
    for name in families["Electrical"]:
        frame = "base"
        if name in {"qtrx-seat","qtrx-entry","shelf-harness-moving-anchor"}:
            frame="slide"
        elif name=="lift-harness-moving-anchor":
            frame="lift"
        sensing=name in {"qtrx-seat","qtrx-entry"}
        groups[name],colors[name],frames[name]=("Sensing","#378985",frame) if sensing else ("Electrical","#537798",frame)
    return families,groups,colors,frames


def flatten(families):
    result={}
    for name,parts in families.items():
        duplicate=result.keys() & parts.keys()
        if duplicate: raise ValueError("Duplicate part IDs in "+name+": "+str(duplicate))
        result.update(parts)
    return result


def pose_shapes(height_mm,stroke_mm):
    """Return named native solids in robot XYZ millimetres, including nonrigid line endpoints."""
    return flatten(pose_layers(height_mm,stroke_mm)[0])


def rotation_bounds():
    """Bounds in unshifted frame coordinates, inclusive of all pinion angles."""
    return {
        name:{"frame":receiver_drives.parts()[name][0],
              "bounds_mm":[x-11,y,z-11,x+11,y+8,z+11],
              "axis":[0,1,0],"outer_radius_mm":11,"axial_width_mm":8}
        for name,(x,y,z,_) in receiver_drives.PINION_CENTERS.items()
    }


def combine(step_dir=None):
    payload, summary = {}, {}
    for pose, desc in native.MODEL["poses"].items():
        source=DRIVE/"review"/(pose+".step")
        families,groups,colors,frames=pose_layers(desc["height"],desc["stroke"])
        shapes=flatten(families)
        conflicts={}
        tested=0
        for a,b in combinations(families.values(),2):
            conflicts.update(clashes(a,b))
            tested+=len(a)*len(b)
        if conflicts: raise ValueError("Assembled layers overlap: "+str(conflicts))
        meshes=[]
        for name,shape in shapes.items():
            verts,idx=triangulate(shape)
            meshes.append([name,groups[name],colors[name],verts,idx])
        payload[pose]={"height":desc["height"],"stroke":desc["stroke"],"parts":meshes}
        summary[pose]={"base_step_sha256":sha(source),"bodies":len(shapes),
            "cross_layer_pairs":tested,"interferences_mm3":conflicts,
            "triangles":sum(len(row[4])//3 for row in meshes)}
        summary[pose]["families"]={name:len(parts) for name,parts in families.items()}
        service={}
        for name,box,detached in electrical.service_volumes():
            envelope=native.brep(["box",box])
            service[name]=clashes({name:envelope},{k:v for k,v in shapes.items() if k not in detached})
        summary[pose]["service_interferences_mm3"]=service
        if any(service.values()): raise ValueError(f"A body blocks a service path in {pose}: {service}")
        if step_dir:
            path=step_dir/(pose+".step")
            write_step(path,shapes)
            summary[pose]["export_sha256"]=sha(path)
    return payload, summary, frames


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--step",action="store_true")
    args=parser.parse_args()
    args.output.mkdir(exist_ok=True,parents=True)
    payload,summary,frames=combine(args.output if args.step else None)
    (args.output/"source-register.json").write_text(json.dumps(source_register(),indent=2)+"\n",encoding="utf-8",newline="\n")
    (args.output/"part-frames.json").write_text(json.dumps(frames,indent=2)+"\n",encoding="utf-8",newline="\n")
    (args.output/"rotation-bounds.json").write_text(json.dumps(rotation_bounds(),indent=2)+"\n",encoding="utf-8",newline="\n")
    packed=gzip.compress(json.dumps(payload,separators=(",",":")).encode(),mtime=0)
    html=(HERE/"viewer.template.html").read_text(encoding="utf-8").replace("@@MESH@@",base64.b64encode(packed).decode())
    (args.output/"robot.html").write_text(html,encoding="utf-8",newline="\n")
    (args.output/"review.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8",newline="\n")
    print(json.dumps(summary,indent=2))


if __name__=="__main__":
    main()
