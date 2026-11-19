"""Generate a local interactive CAD review using a supplied AndyMark light STEP."""
import argparse
import base64
import gzip
import html
import json
import os
import re
from pathlib import Path

from inspect_light import HERE, KIT, PANEL, X, Y, cad, configuration, mounted, product, read
from assembly import original


def build(source, output):
    lamp=mounted(product(source),X,Y,PANEL)
    kit=cad.read_named(HERE/"deck-mount.step",[{"id":name} for name in ("front-electronics-platform","light-spacer")])
    prior=(KIT/"robot.html").read_text(encoding="utf8")
    packed=re.search(r'<script id="mesh" type="application/octet-stream">([^<]+)</script>',prior)[1]
    previous=json.loads(gzip.decompress(base64.b64decode(packed)))
    style={p[0]:p[1:3] for p in previous["stowed"]["parts"]}
    review=read(HERE/"review.json")["cases"]
    data={}
    for pose,mode,height,stroke in [("stowed","high",940,0),("receive","low-receive",560,0),("place","low-place",560,370)]:
        assembly=configuration(mode)
        assembly.pop("rsl-reserve")
        assembly.update(kit)
        assembly["am-3583"]=lamp
        if len(assembly)!=review[mode]["installed_model_bodies"]:
            raise ValueError("Displayed robot differs from inspected robot")
        parts=[]
        for key,shape in assembly.items():
            group,color=style.get(key,("Carriage locks","#bd8540"))
            if key in kit or key=="am-3583":
                group="Signal light mount"
                color={"am-3583":"#ffac24","light-spacer":"#aab7b8","front-electronics-platform":"#708b91"}[key]
            vertices,faces=original.triangulate(shape)
            parts.append([key,group,color,vertices,faces])
        data[pose]={"height":height,"stroke":stroke,"parts":parts}
    template=(KIT.parents[1]/"viewer.template.html").read_text(encoding="utf8")
    template=template.replace("Station receiver","Signal light inspection").replace(
        "Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit.",
        "Compare the two starting configurations and the extended receiver. Choose Signal light mount to isolate the lamp, keyed deck and spacer.")
    template=template.replace("const names={stowed:'Stowed',receive:'Receive',transport:'Carry',place:'Place'};",
        "const names={stowed:'High park',receive:'Low pickup',place:'Low placement'};")
    template=template.replace("pose='place'","pose='stowed'").replace("setPose('place')","setPose('stowed')")
    template=template.replace("const info={","const info={'Signal light mount':'The right-side tab carries the lamp above the deck and leaves space for its connectors underneath.',")
    template=template.replace("The lift changes height while the slide moves the shelf toward the station or trough. Use the pose controls to compare their motions.",
        "The high and low builds use different fixed carriage locks. Only the low build extends its shelf.")
    def relative(path):
        return html.escape(Path(os.path.relpath(path,output.parent)).as_posix(),quote=True)
    template=template.replace("../../mission/moving-field.html",relative(HERE.parents[1]/"fixed-height-match/replay.html"))
    template=template.replace('href="README.md"','href="'+relative(HERE/"README.md")+'"')
    compressed=gzip.compress(json.dumps(data,separators=(",",":")).encode(),mtime=0)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_bytes(template.replace("@@MESH@@",base64.b64encode(compressed).decode()).encode())


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vendor_step",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    build(args.vendor_step,args.output.resolve())
