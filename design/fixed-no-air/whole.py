"""Register the fixed changes against the complete moving-observer CAD configuration."""
import argparse
import base64
import gzip
import hashlib
import html
import io
import json
import os
from pathlib import Path
import re
import sys

from service import AIR, HERE, LIGHT, ROOT, PANEL, X, Y, cad, digest, mounted, product, read
from assembly import original, pose_shapes
sys.path.insert(0,str(HERE.parent/"fixed-height"))
from native import hardware
from OCP.BRepTools import BRepTools
from OCP.TopTools import TopTools_FormatVersion


def fingerprint(shape):
    stream = io.BytesIO()
    BRepTools.Write_s(shape,stream,False,False,TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def maximum(shapes):
    tips = {k:cad.bounds(v)[3] for k,v in shapes.items()}
    x = max(tips.values())
    return {"x_mm":x,"bodies":sorted(k for k,v in tips.items() if abs(v-x)<.001)}


def model(stroke, roof, deck, lamp):
    base = pose_shapes(560,stroke,"red")
    changes,add = hardware(base,height=560)
    old = {**base,**changes,**add}
    old.pop("receiver-R6-ported")
    for name,shape in roof.items():
        key = name if name=="receiver-R6-drilled-for-camera" else "roof-"+name
        old[key] = cad.translated(shape,[stroke,0,0])
    if len(old)!=598:
        raise ValueError("The moving optical source has changed")
    new = {k:v for k,v in old.items() if k not in AIR|{"rsl-reserve"}}
    new.update(deck)
    new["am-3583"] = lamp
    return old,new


def run(source, viewer):
    folder = ROOT/"moving-observer"
    roof = cad.read_named(folder/"moving-bridge.step",[{"id":s} for s in
                         [*read(folder/"mount.json")["parts"],"receiver-R6-drilled-for-camera"]])
    deck = cad.read_named(LIGHT/"deck-mount.step",[{"id":n} for n in ("front-electronics-platform","light-spacer")])
    lamp = mounted(product(source),X,Y,PANEL)
    kit = HERE.parent/"competition-package/shop-duplex/red"
    frames = read(kit/"part-frames.json")
    if any(frames[k]!="base" for k in AIR|{"rsl-reserve","front-electronics-platform"}):
        raise ValueError("An omitted or replaced article moved with the slide")
    template = (kit.parents[1]/"viewer.template.html").read_text(encoding="utf8")
    prior = (kit/"robot.html").read_text(encoding="utf8")
    packed = re.search(r'<script id="mesh" type="application/octet-stream">([^<]+)</script>',prior)[1]
    style = {row[0]:row[1:3] for row in json.loads(gzip.decompress(base64.b64decode(packed)))["stowed"]["parts"]}
    cases, scenes, display = {},{},{}
    for pose,stroke in (("receive",0),("transport",185),("place",370)):
        old,new = model(stroke,roof,deck,lamp)
        old_hashes = {k:fingerprint(v) for k,v in sorted(old.items())}
        new_hashes = {k:fingerprint(v) for k,v in sorted(new.items())}
        changed = {k for k in old.keys() & new.keys() if old_hashes[k]!=new_hashes[k]}
        if changed!={"front-electronics-platform"} or old.keys()-new.keys()!=AIR|{"rsl-reserve"}:
            raise ValueError("Unaccounted geometry change")
        fixed_names = AIR|{"rsl-reserve","front-electronics-platform","am-3583","light-spacer"}
        pair = {k:old_hashes[k] for k in old.keys() & new.keys() if k not in changed}
        receipt = {"stroke_mm":stroke,"source_bodies":len(old),"match_bodies":len(new),
                   "source_front":maximum(old),"match_front":maximum(new),
                   "prior_fixed_change_front":maximum({k:v for k,v in old.items() if k in fixed_names}),
                   "new_fixed_change_front":maximum({k:v for k,v in new.items() if k in fixed_names}),
                   "removed":{k:old_hashes[k] for k in sorted(old.keys()-new.keys())},
                   "old_deck":old_hashes["front-electronics-platform"],
                   "new_or_changed":{k:new_hashes[k] for k in sorted((new.keys()-old.keys())|changed)},
                   "identical_brep_sha256":dict(sorted(pair.items()))}
        cases[pose]=receipt
        if viewer:
            scenes[pose] = (stroke,new)
    if viewer:
        # Meshing updates flags on shared OpenCascade topology. Finish every
        # native receipt before triangulating anything for display.
        for pose,(stroke,new) in scenes.items():
            parts=[]
            for k,v in new.items():
                group,color=style.get(k,("Carriage locks","#b48a4f"))
                if k.startswith("roof-") or k=="receiver-R6-drilled-for-camera":
                    group,color="Moving camera",("#278f99" if k.startswith("roof-") else "#889e97")
                if k in deck or k=="am-3583":
                    group,color="Signal light",("#f4ac32" if k=="am-3583" else "#688a8e")
                parts.append([k,group,color,*original.triangulate(v)])
            display[pose]={"height":560,"stroke":stroke,"parts":parts}
    if viewer:
        viewer.parent.mkdir(parents=True,exist_ok=True)
        def relative(p):
            return html.escape(Path(os.path.relpath(p,viewer.parent)).as_posix(),quote=True)
        edits={"Station receiver":"Fixed-height scoring robot",
               "Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.":
                   "Follow the second camera through extension. Isolate the moving camera or the signal light to see how each is mounted.",
               "const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};":
                   "const names={receive:'Pickup',transport:'Midstroke',place:'Placement'};",
               "The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.":
                   "The carriage stays at 560 mm while the shelf travels 370 mm. The compressor, tank and distribution hardware are omitted.",
               "../../mission/moving-field.html":relative(ROOT/"fixed-height-match/replay.html"),
               'href="README.md"':'href="'+relative(HERE/"README.md")+'"',
               "@@MESH@@":base64.b64encode(gzip.compress(json.dumps(display,separators=(",",":")).encode(),mtime=0)).decode()}
        for key,value in edits.items():
            if key not in template:
                raise ValueError("Viewer template changed")
            template=template.replace(key,value)
        viewer.write_bytes(template.encode())
    sources=[folder/"moving-bridge.step",folder/"mount.json",folder/"mount.py",LIGHT/"deck-mount.step",
             LIGHT/"review.json",ROOT/"design/fixed-height/native.py",kit/"part-frames.json"]
    return {"source_sha256":{p.relative_to(ROOT).as_posix():digest(p) for p in sources},
            "supplied_lamp_sha256":digest(source),"cases":cases}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("signal_light_step",type=Path)
    parser.add_argument("--viewer",type=Path)
    args=parser.parse_args()
    result=run(args.signal_light_step,args.viewer.resolve() if args.viewer else None)
    (HERE/"whole-robot.json").write_bytes((json.dumps(result,indent=2)+"\n").encode())
