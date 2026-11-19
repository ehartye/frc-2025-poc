"""Cache the registered stowed scene from the issued sensor and receiver design."""
import argparse
import json
from pathlib import Path
import sys

def run(artifacts, lamp, target):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"experiments/six-wheel/fixed-camera-route"))
    import scene
    from OCP.BRepTools import BRepTools
    stowed, _ = scene.get(artifacts, lamp)
    if len(stowed) != 991:
        raise ValueError("Expected 991 issued scene articles")
    target.mkdir(parents=True,exist_ok=True)
    index={}
    for number,(key,shape) in enumerate(stowed.items()):
        filename=f"{number:04}.brep"
        BRepTools.Write_s(shape,str(target/filename))
        index[key]=filename
    (target/"index.json").write_text(json.dumps(index,indent=2)+"\n")

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("artifacts","lamp","target"):
        parser.add_argument(key,type=Path)
    args=parser.parse_args()
    run(args.artifacts,args.lamp,args.target)
