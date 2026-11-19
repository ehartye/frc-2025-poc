"""Render the original station feed with the aft camera's specific stowed full-robot mesh."""
import argparse
import json
import sys
from pathlib import Path
import numpy as np
from observe import HERE, ROOT, bridge, mount, camera, digest


def run(cache, historical, lamp_path):
    sys.path.insert(0,str(ROOT/"design/fixed-no-air"))
    import whole
    register=json.loads((ROOT/"design/fixed-no-air/whole-robot.json").read_bytes())
    if digest(lamp_path)!=register["supplied_lamp_sha256"]:raise ValueError("Lamp changed")
    names=[*bridge.parts()[0],"receiver-R6-drilled-for-camera"]
    roof=mount.cad.read_named(ROOT/"bridge-tolerance/moving-bridge.step",[{"id":k} for k in names])
    deck=mount.cad.read_named(whole.LIGHT/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    _,combined=whole.model(0,roof,deck,whole.mounted(whole.product(lamp_path),whole.X,whole.Y,whole.PANEL))
    if len(combined)!=593:raise ValueError("Incomplete stowed robot")
    _,groups,colors,_=mount.pose_layers(560,0,"red")
    mesh=[]
    for key,shape in sorted(combined.items()):
        v,t=mount.original.triangulate(shape)
        mesh.append([key,groups.get(key,"Mount"),colors.get(key,"#737d85"),v,t])
    views=camera.images_for_pose({"parts":mesh})
    gray=[g for g,_ in views]

    module=bridge.mod("fixed_readout",ROOT/"stereo-custody/observe.py")
    profiles=json.loads((historical/"source.json").read_bytes())["pieces"]
    out={}
    for key,rec in profiles.items():
        tr=np.load(historical/(key+"-trajectory.npy"))
        frames=[]
        for tm in range(0,int(rec["first_in_band_ms"])+230,17):
            coords=[np.interp(tm,tr[:,0],s) for s in tr[:,1:5].T] if tm>=tr[0,0] else None
            pixels=camera.exposure(views,None if coords is None else coords[:3],0 if coords is None else coords[3])
            frames.append([tm,module.read_pair(pixels,gray,560,0)])
        out[key]=frames
    (cache/"receive.json").write_text(json.dumps(out,allow_nan=False),encoding="utf8")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("cache","historical","lamp"):p.add_argument(name,type=Path)
    a=p.parse_args();run(a.cache,a.historical,a.lamp)

