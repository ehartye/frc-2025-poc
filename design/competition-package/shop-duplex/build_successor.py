"""Export a complete selected alliance and independently reopen every original solid."""
import argparse
import base64
import gzip
import json
from itertools import combinations
from pathlib import Path

from assembly import BASE,HERE,cad,original,pose_layers,sha,source_register
from mechanical import cylinder
from check_routing import clashes


def write_json(path,record):
    path.write_text(json.dumps(record,indent=2,allow_nan=False)+"\n",encoding="utf-8",newline="\n")


def verify_service(shapes,desc):
    services={}
    for name,spec,removed in original.electrical.service_volumes():
        services[name]=clashes({name:cad.brep(["box",spec])},{k:v for k,v in shapes.items() if k not in removed})
    for side in [-1,1]:
        pin=cad.brep(cylinder(-196,side*282,desc["height"]-165,[1,0,0],4,130))
        services["duplex-pin-"+str(side)]=clashes({"swept-pin":pin},shapes)
    if any(services.values()):raise ValueError(("Blocked service",services))
    return services


def pack_view(path,model,color):
    template=(BASE/"viewer.template.html").read_text(encoding="utf-8")
    other="red" if color=="blue" else "blue"
    template=template.replace("<h1>Station receiver</h1>",f'<h1>Station receiver · {color} alliance</h1>')
    old="Follow the shelf from pickup to placement. Rotate the robot or isolate a subsystem to inspect how its parts fit."
    template=template.replace(old,'Inspect the outer carrier clamps, retained receiver blocks and duplex lift. '
                              +f'<a href="../{other}/robot.html">View the {other} robot</a>.')
    template=template.replace("const info={","const info={'Shop joints':'Outer carrier clamps, pinned receiver anchors and drilled moving channels.',"
                              +"'Lift retention':'Secondary per-side lift lines, shaft extension, return sheaves and service pin rails.',")
    packed=gzip.compress(json.dumps(model,separators=(",",":")).encode(),mtime=0)
    path.write_text(template.replace("@@MESH@@",base64.b64encode(packed).decode()),encoding="utf-8",newline="\n")


def build(out,alliance):
    out.mkdir(parents=True,exist_ok=True)
    summaries,views={},{}
    for pose,dimensions in cad.MODEL["poses"].items():
        layers,groups,colors,frames=pose_layers(dimensions["height"],dimensions["stroke"],alliance)
        bodies=original.flatten(layers)
        checked=0
        for first,second in combinations(layers.values(),2):
            checked+=len(first)*len(second)
            if problems:=clashes(first,second):raise ValueError((pose,"installation conflict",problems))
        labels=[{"id":k,"group":k} for k in bodies]
        path=out/(pose+".step")
        cad.write_named(path,list(bodies.values()),labels,colors)
        from duplex import finalize_step
        finalize_step(path)
        restored=cad.read_named(path,labels)
        worst=0
        for k,shape in restored.items():
            before=cad.volume(bodies[k])[0]
            error=abs(cad.volume(shape)[0]-before)
            if error>max(.05,before*1e-8):raise ValueError((k,"volume changed on reopen",error))
            worst=max(worst,error)
        view=[]
        for k,shape in bodies.items():
            vertices,triangles=original.triangulate(shape)
            view.append([k,groups[k],colors[k],vertices,triangles])
        views[pose]=dict(**dimensions,parts=view)
        summaries[pose]={"alliance":alliance,"bodies":len(bodies),"export_sha256":sha(path),
                         "cross_layer_pairs":checked,"interferences_mm3":{},
                         "max_reopened_volume_error_mm3":round(worst,8),
                         "service_interferences_mm3":verify_service(bodies,dimensions),
                         "family_counts":{k:len(v) for k,v in layers.items()},
                         "triangles":sum(len(item[4])//3 for item in view)}
        print(pose,summaries[pose]["export_sha256"],flush=True)
    write_json(out/"review.json",summaries)
    write_json(out/"part-frames.json",frames)
    write_json(out/"rotation-bounds.json",original.rotation_bounds())
    write_json(out/"source-register.json",source_register())
    pack_view(out/"robot.html",views,alliance)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",type=Path)
    parser.add_argument("--alliance",choices=["blue","red"],required=True)
    args=parser.parse_args()
    build(args.output,args.alliance)
