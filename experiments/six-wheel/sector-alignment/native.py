"""Reopen the actual red reef panel and aft-mount stowed robot for the fixed stereo sight check."""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "bridge-tolerance"))
from design import mount, parts
sys.path.insert(0, str(ROOT / "design/fixed-no-air"))
import whole
sys.path.insert(0, str(ROOT / "moving-observer"))
from clearance import reef_parts, FIRST_SHA
from imaging import images_for_pose, ORIGIN, EYES, DX, DY, DZ

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def measured_face(points):
    # Retain only the convex front outline and its front plane; engraved internal marks
    # and the remaining manufacturer geometry are unnecessary for this lighting model.
    yz=sorted(set((y,z) for _,y,z in points))
    def half(seq):
        edge=[]
        for pt in seq:
            while len(edge)>=2:
                a,b=edge[-2:]
                if (b[0]-a[0])*(pt[1]-a[1])>(b[1]-a[1])*(pt[0]-a[0]):break
                edge.pop()
            edge.append(pt)
        return edge[:-1]
    x=min(v[0] for v in points)
    return [[x,y,z] for y,z in half(yz)+half(reversed(yz))]

def build(field, lamp, out):
    source = json.loads((ROOT / "design/fixed-no-air/whole-robot.json").read_bytes())
    if sha(field) != FIRST_SHA or sha(lamp) != source["supplied_lamp_sha256"]:
        raise ValueError("Unregistered field or signal light")
    cad = mount.cad
    roof_path = ROOT / "bridge-tolerance/moving-bridge.step"
    kit = cad.read_named(roof_path, [{"id": k} for k in [*parts()[0], "receiver-R6-drilled-for-camera"]])
    deck = cad.read_named(whole.LIGHT / "deck-mount.step", [{"id": k} for k in ("front-electronics-platform", "light-spacer")])
    body = whole.mounted(whole.product(lamp), whole.X, whole.Y, whole.PANEL)
    _, robot = whole.model(0, kit, deck, body)
    if len(robot) != 593:
        raise ValueError("Stowed body inventory changed")
    mesh = {"parts":[]}
    for k, shape in sorted(robot.items()):
        vertices, faces = mount.original.triangulate(shape)
        mesh["parts"].append([k, "robot", "#59695e", vertices, faces])
    depth = images_for_pose(mesh)
    reef = reef_parts(field)
    bounds = {k: list(cad.bounds(s)) for k,s in reef.items()}
    panel = next(k for k in bounds if k.endswith("ID10"))
    box = bounds[panel]
    panel_xyz = [box[0], -(box[2]+box[5])/2, (box[1]+box[4])/2]
    # Any ray to the black marker crosses the last 120 mm ahead of the field plane
    # below 580 mm and within +/-260 mm native Z when registered within the test range.
    # Resolve the actual foreground faces inside this guarded volume, including hardware.
    candidates = {k:v for k,v in bounds.items() if k!=panel and v[0]<box[0]+.1 and v[3]>box[0]-120 and v[1]<580 and v[4]>180 and v[2]<260 and v[5]>-260}
    out.mkdir(parents=True, exist_ok=True)
    names = ("left","right")
    images = {}
    for n,eye,(gray,z) in zip(names,EYES,depth):
        # Includes every retained transverse pipe in the issued custody envelope.
        # Expand the finite cylinder over the modeled center, yaw and radius uncertainty.
        origin = ORIGIN + [0,eye,0]
        lo=np.array([-5.,-168.,482.]); hi=np.array([168.,168.,633.])
        near=np.full(z.shape,-np.inf);far=np.full(z.shape,np.inf)
        for i,d in enumerate((DX,DY,DZ)):
            with np.errstate(divide="ignore"):
                s=(lo[i]-origin[i])/d; q=(hi[i]-origin[i])/d
            near=np.maximum(near,np.minimum(s,q));far=np.minimum(far,np.maximum(s,q))
        blocked=(far>=near)&(near>0)&(near<z)
        images[n]=gray.copy();images[n][blocked]=170
        images[n+"_z"]=np.where(blocked,near,z)
    file=out/"fixed-stowed.npz"
    np.savez_compressed(file,**images)
    def native_to_world(b):
        return [b[0]-2963.050980729168, -b[5], b[1], b[3]-2963.050980729168, -b[2], b[4]]
    record={"fieldSHA256":sha(field),"robotBodies":len(robot),"bridgeSHA256":sha(roof_path),"lampSHA256":sha(lamp),
            "panel":panel,"nativePanelBoundsMm":box,"targetFrameCenterMm":[panel_xyz[0]-2963.050980729168,*panel_xyz[1:]],
            "foregroundBoxesMm":{k:native_to_world(v) for k,v in candidates.items()},
            "carryBoxMm":[-5,-168,482,168,168,633],"imageArraysSHA256":{n:hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest() for n,a in images.items()}}
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_VERTEX
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    face_witness={}
    for name in candidates:
        if "Bolt_" not in name:continue
        shape=reef[name]; ex=TopExp_Explorer(shape,TopAbs_VERTEX);pts=set()
        while ex.More():
            pt=BRep_Tool.Pnt_s(TopoDS.Vertex(ex.Current()))
            if pt.X()<candidates[name][0]+.5:
                pts.add((round(pt.X()-2963.050980729168,4),round(-pt.Z(),4),round(pt.Y(),4)))
            ex.Next()
        face_witness[name]=measured_face(pts)
    guard=cad.brep(["box",[box[0]-3.4,panel_xyz[2]-110,-panel_xyz[1]-110,3.397,220,220]])
    fascia=next(k for k in candidates if k.startswith("NAUO1053/"))
    overlap=cad.intersection(reef[fascia],guard)
    if overlap>.001:raise ValueError("Tag aperture blocked by fascia")
    record["fasciaWindow"]={"occurrence":fascia,"squareMm":220,"frontSlabMm":3.397,"intersectMm3":overlap}
    record["boltFacesMm"]=face_witness
    (HERE/"scene.json").write_text(json.dumps(record,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for n in ("field","lamp","out"):p.add_argument(n,type=Path)
    a=p.parse_args();build(a.field,a.lamp,a.out)
