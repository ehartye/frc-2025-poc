"""Measure the orientation of a confirmed pipe using its roof-camera tangents."""
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"conditioned-match"))
import roof


def prepare(assets):
    register=json.loads((ROOT/"selected-optics/mount.json").read_bytes())
    roof.optics.ORIGIN=np.array(register["registrations"]["roof"]["assumedRectifiedOriginMm"])
    roof.optics.configure()
    folder=assets/"integrated-slide/issue/marked"
    source=json.loads((folder/"scenes.json").read_bytes())
    def load(name):
        path=folder/(name+".npz")
        if roof.hashlib.sha256(path.read_bytes()).hexdigest()!=source["backgroundSha256"][name]:
            raise ValueError("Unregistered moving camera background")
        return roof.background(path)
    registered=load("barrel-moving-0")
    backed=load("barrel-moving-reference")
    parts=roof.mount.cad.read_named(ROOT/"station-guide/narrow-face.step",[{"id":"guide-positive-y"},{"id":"guide-negative-y"}])
    mesh=[]
    for key,shape in parts.items():
        vertices,triangles=roof.mount.original.triangulate(shape)
        mesh.append([key,"guide","#a9a49c",vertices,triangles])
    guide=roof.camera.images_for_pose({"parts":mesh})
    actual=[];reference=[]
    for (gray,z),(_,guaranteed),(g,near) in zip(registered,backed,guide):
        image=np.where(near<z,g,gray)
        actual.append((image,np.minimum(near,z)))
        reference.append((image,np.minimum(near,guaranteed)))
    return actual,reference


def yaw_interval(images,reference):
    lower,upper=-math.inf,math.inf
    for im,(gray,z) in zip(images,reference):
        valid=z<3490
        changed=(im>=185)&(im.astype(int)-gray.astype(int)>36)&valid
        middle=im.shape[1]//2
        count=0
        for span in (105,110,119,130,140):
            rows=[]
            for c in (middle-span,middle+span):
                ys=np.flatnonzero(changed[:,c])
                if len(ys)<35 or np.any(np.diff(ys)!=1):
                    break
                end=ys[-1]+1
                if end+3>=im.shape[0] or not np.all(valid[end-3:end+3,c]):
                    break
                rows.append(end)
            if len(rows)==2:
                delta=rows[1]-rows[0]
                lower=max(lower,math.atan((delta-3)/(2*span)))
                upper=min(upper,math.atan((delta+3)/(2*span)))
                count+=1
        if count==0:
            return None
    return (lower,upper) if lower<=upper else None


def serve(assets,elastic=None,head="roof"):
    import hashlib
    if elastic:
        sys.path.insert(0,str(elastic))
        from carry_images import camera_views
        actual,reference=camera_views(assets,assets/"elastic-carry",head)
    else:
        actual,reference=prepare(assets)
    previous = {}
    for line in sys.stdin:
        q = np.array(json.loads(line), dtype=float)
        images = roof.silhouette(actual, q[:3], q[3:])
        key = hashlib.sha256(b"".join(image.tobytes() for image in images)).hexdigest()
        if key not in previous:
            fit = roof.fit_row(images, reference, True, 0)
            previous[key] = [fit, yaw_interval(images, reference), key]
        print(json.dumps(previous[key]),flush=True)


if __name__ == "__main__":
    serve(Path(sys.argv[1]),Path(sys.argv[2]) if len(sys.argv)>2 else None,sys.argv[3] if len(sys.argv)>3 else "roof")
