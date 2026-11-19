"""Register the low-height fixed assembly and its two stationary slide positions."""
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"design/fixed-height"))
from native import hardware
sys.path.insert(0,str(ROOT/"design/competition-package/shop-duplex"))
from assembly import pose_layers,original
sys.path.insert(0,str(ROOT/"stereo-custody"))
from imaging import images_for_pose


def render_pose(stroke):
    layers,groups,colors,frames=pose_layers(560,stroke,"red")
    parts=original.flatten(layers)
    machined,added=hardware(parts,height=560)
    parts.update(machined)
    parts.update(added)
    mesh=[]
    for name,shape in sorted(parts.items()):
        verts,triangles=original.triangulate(shape)
        mesh.append([name,groups.get(name,"Fixed lock"),colors.get(name,"#737d85"),verts,triangles])
    encoded=json.dumps(mesh,separators=(",",":")).encode()
    registration={"height_mm":560,"stroke_mm":stroke,"bodies":len(parts),"machined":sorted(machined),
                  "added":sorted(added),"mesh_sha256":hashlib.sha256(encoded).hexdigest(),
                  "front_mm":max(original.native.bounds(s)[3] for s in parts.values())}
    return images_for_pose({"parts":mesh}),registration
