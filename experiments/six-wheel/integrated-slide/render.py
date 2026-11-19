"""Render the source-registered shelf mount in the released two-camera scene."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"selected-optics"))
import scenes


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def select(model, artifacts, lamp):
    stages=scenes.selected_poses(model,artifacts,lamp)
    sys.path.insert(0,str(ROOT/"sp1-mount"))
    from sp1_hardware import REMOVED, supplied
    receipt=json.loads((ROOT/"sp1-mount/fit.json").read_bytes())
    step=ROOT/"sp1-mount/sp1-mount.step"
    if digest(step)!=receipt["stepSha256"]:
        raise ValueError("Different SP1 manufacturing kit")
    new=model.cad.read_named(step,[{"id":k} for k in receipt["volumeMm3"]])
    for stroke,group in zip((0,370),stages):
        for key in REMOVED:
            del group[key]
        bounds=receipt["boundsMm"][str(stroke)]
        added={k:model.cad.translated(v,[stroke if k.startswith(("sp1-eye","sp1-shelf")) else 0,0,0]) for k,v in new.items()}
        added.update(supplied(stroke))
        for k,shape in added.items():
            if max(abs(a-b) for a,b in zip(model.cad.bounds(shape),bounds[k]))>.00001:
                raise ValueError("Unregistered SP1 placement: "+k)
        group.update(added)
    return stages


def run(args):
    args.assembly=ROOT/"selected-match/clearance.py"
    if args.stage=="endpoints":
        args.stowed_robot_only=False
        scenes.render(args,select)
        out=args.out/"scenes.json"
    else:
        import stroke
        stroke.run(args,select)
        out=args.out/"sweep.json"
    data=json.loads(out.read_bytes())
    data["sp1Sources"]={f:digest(ROOT/f) for f in (
        "sp1-mount/sp1-mount.step","sp1-mount/fit.json","sp1-mount/sp1_hardware.py",
        "selected-optics/fixed-datum.step","selected-optics/mount.json","integrated-slide/render.py")}
    out.write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=["endpoints","sweep"])
    for key in ("artifacts","lamp","field","out"):
        parser.add_argument(key,type=Path)
    parser.add_argument("--fixed-depth",type=float,choices=[-.75,0,.75],default=0.)
    run(parser.parse_args())
