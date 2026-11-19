"""Register the current slide instrumentation and fixed-pupil shop scene before projecting the added guide."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"integrated-slide"))
from render import scenes,select


def summarize(out):
    data=json.loads((out/"station-background.json").read_bytes())
    for key in ("integrated-slide/render.py","sp1-mount/fit.json","sp1-mount/sp1-mount.step"):
        data["sourceSha256"][key]=hashlib.sha256((ROOT/key).read_bytes()).hexdigest()
    (Path(__file__).parent/"background.json").write_text(json.dumps(data,indent=2)+"\n",encoding="utf8",newline="\n")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts","lamp","field","out"):parser.add_argument(key,type=Path)
    args=parser.parse_args()
    args.assembly=ROOT/"selected-match/clearance.py"
    args.fixed_depth=0.
    args.stowed_robot_only=True
    scenes.render(args,select)
    summarize(args.out)
